/*
 * USB PD protocol layer (PRL), following the spec's PRL_Tx/PRL_Rx/PRL_HR
 * as closely as the PHY split allows. The PHY (interface/PdPhy.hpp) owns the bit
 * level - CRC checking, GoodCRC autoresponse, per-attempt outcome
 * detection - and everything above lives here: MessageID stamping, the
 * RetryCounter with the retransmission loop, the protocol timers,
 * duplicate rejection on receive, and the MessageID lifecycle across
 * soft and hard resets. Chunked extended messages are not handled yet.
 *
 * Transmit is a state machine (state/ProtocolLayer.hpp) with the
 * pending message and RetryCounter in machine-owned context, in the
 * form the PHY asks for (usbc::retransmission). By default wait_for_phy_response
 * runs CRCReceiveTimer (tReceive): a failed attempt - reported by the
 * PHY or by the timer - retransmits with the same MessageID while
 * RetryCounter allows (guarded self-transition), then lands in
 * transmission_error. A PHY retransmitting itself (a TCPC in the TCPCI
 * sense) gets the table without the retransmission rows: the state
 * waits for the PHY's one outcome under t_transmit_outcome, and its
 * failure is the transmission error. transmission_error reports
 * onTxError, increments the MessageIDCounter, and rests until the
 * policy engine transmits again or resets. Success and discard
 * outcomes arrive through the PHY's callbacks and report
 * onTxDone/onTxDiscarded; the MessageIDCounter increments on these
 * completions only - never per attempt. transmit() while a message is
 * in flight is refused - the policy engine serializes its requests.
 *
 * Hard reset: transmitHardReset() hands the signal to the PHY and
 * waits in wait_for_hard_reset_complete, bounded by
 * HardResetCompleteTimer (tHardResetComplete); PHY confirmation or the
 * timer completes it and reports onHardResetSent.
 *
 * One layer per port, observing the connection machine: the PD
 * connection's entry claims the PHY (the layer hears it only while it
 * holds it: claimPhy() registers its callbacks, releasePhy() clears
 * them), its exit - the one real detach - releases the PHY and forgets
 * the partner (MessageIDs, revision), and the attach's plug
 * orientation selects the PHY's CC line. The client - the policy
 * engine the layer reports to - is set by the engine going live
 * (setClient with the roles its header carries) and cleared by the
 * engine retiring; a dual-role port's two engines take turns on the
 * one layer, so a power role swap continues the connection's MessageID
 * lifecycle and revision by construction (6.7.1: a swap is no reset
 * trigger). The engines' layer actions reach the layer through their
 * own observer, which forwards them only for the client.
 *
 * Execution contract: the transmit functions, reset, claim and release
 * run in the stack's context, and the PHY delivers its callbacks
 * there. The TIMER policy fires fsm::timeout from its own execution
 * context (mtl timer contract), and the integrator serializes it with
 * the stack's calls; client callbacks on timeout paths (onTxError,
 * onHardResetSent) originate from that serialized timer context.
 *
 * Revision negotiation lives here, at the same choke point as the
 * MessageID: the layer starts at rev 3.x, adopts the lowest revision
 * seen in received SOP messages, stamps it into every transmitted
 * header, and switches nRetryCount (2 for rev 3.x, 3 for rev 2.0) per
 * message - a retransmitting PHY reads it from the header. The layer
 * owns the PHY's message header info (the GoodCRC header): the
 * client's power and data roles from setClient / setDataRole and the
 * revision as negotiated. The negotiated revision survives a soft
 * reset (reset()) and restarts at rev 3.x on a hard reset and on
 * detach.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/annotation/TypeC.hpp>
#include <usbc/interface/PdPhy.hpp>
#include <usbc/spec/Message.hpp>
#include <usbc/spec/Parameters.hpp>
#include <usbc/state/ProtocolLayer.hpp>

#include <mtl/StateMachine.hpp>

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace usbc {

namespace concepts {

template<typename T>
concept prl_client = requires(T client, pd_message const& message) {
    client.onMessage(message);
    client.onTxDone();
    client.onTxDiscarded();
    client.onTxError();
    client.onHardReset();     // hard reset received from the partner
    client.onHardResetSent(); // own hard reset signal is on the wire
};

// The port's protocol layer as a policy engine uses it: transmitting
// messages, signals and the hard reset, the soft reset scope, the
// negotiated revision, the header's data role, BIST test data (the
// client switch is templated on the client and asked of the concrete
// layer)
template<typename T>
concept protocol_layer = requires(
    T layer,
    pd_message const& message,
    transmit_signal signal,
    sop_type sop,
    data_role role,
    bool enable
) {
    { layer.transmit(message) } -> std::convertible_to<bool>;
    { layer.transmit(signal) } -> std::convertible_to<bool>;
    { layer.transmitHardReset() } -> std::convertible_to<bool>;
    layer.reset(sop);
    { layer.revision() } -> std::same_as<pd_revision>;
    layer.setDataRole(role);
    layer.setBistTestData(enable);
};

} // namespace concepts

namespace prl {

// Hands a state's pd_message instance value to the PHY on entry; a
// pd_message in a state's values() is the marker that makes it a
// transmitting one. A refused hand-off is not reported: the state's
// timeout turns it into a retry or a transmission error
template<concepts::pd_phy PHY>
struct phy_driver : fsm::observing<phy_driver<PHY>> {
    explicit phy_driver(PHY& phy_ref) : phy(phy_ref) {}

    using observes = fsm::annotations<pd_message>;

    void notifyEntry(pd_message const& message) { phy.transmit(message); }

    PHY& phy;
};

} // namespace prl

// The client is type-erased behind plain function pointers (no RTTI,
// no virtuals): the layer is fully
// role-independent, and templating it on the client duplicated the
// whole PRL - tx machine, receive drain, counters - once per policy
// engine (measured ~2 kB flash in a DRP image)
template<concepts::pd_phy PHY, fsm::concepts::timer TIMER, typename... OBSERVERs>
class ProtocolLayer : public fsm::observing<ProtocolLayer<PHY, TIMER, OBSERVERs...>> {
public:
    // The transmit machine in the PHY's form: with or without the
    // retransmission loop
    static constexpr retransmission who_retransmits = retransmission_of<PHY>;
    using wait_for_phy_response = prl::state::wait_for_phy_response<who_retransmits>;

    // The observers ride along in the transmit machine after the
    // layer's own. No client yet: the engine going live sets itself
    ProtocolLayer(PHY& phy, TIMER& timer, OBSERVERs&... observers)
        : phy_(phy), timer_(timer), timed_(timer_), sm_(timed_, driver_, reporter_, observers...)
    {
    }

    // The engine the layer reports to, with the roles its header
    // carries; set by the engine going live, cleared by the one
    // retiring (only its own entry: a DRP's engines hand over in either
    // order), asked by the engines' layer ports before forwarding an
    // action
    template<concepts::prl_client CLIENT>
    void setClient(CLIENT& client, power_role power_role, data_role data_role)
    {
        client_ = &client;
        hooks_ = &hooks_for<CLIENT>;
        power_role_ = power_role;
        data_role_ = data_role;
        stampHeader();
    }

    template<concepts::prl_client CLIENT>
    void clearClient(CLIENT const& client)
    {
        if (clientIs(client)) {
            client_ = nullptr;
            hooks_ = nullptr;
        }
    }

    template<concepts::prl_client CLIENT>
    bool clientIs(CLIENT const& client) const
    {
        return client_ == &client;
    }

    // An agreed DR_Swap: the header follows
    void setDataRole(data_role data_role)
    {
        data_role_ = data_role;
        stampHeader();
    }

    // The PHY reports to the layer holding it: the PD connection's
    // entry claims it, its exit releases it (observed below); the
    // layer listens for SOP messages and hard resets
    void claimPhy()
    {
        phy_.setCallbacks(phy_callbacks, this);
        phy_.setReceiveDetect(receive_detect::sop | receive_detect::hard_reset);
    }
    void releasePhy() { phy_.setCallbacks({}, nullptr); }

    // The connection machine's facts: the connection brackets the
    // layer's hold of the PHY, its exit is the one real detach - the
    // partner is forgotten with it; the attached cable's CC wire
    // carries the messaging. The layer is injected ahead of the
    // engines, so the line is selected before an engine's first
    // transmission
    using observes = fsm::annotations<tc::pd_connection, plug_orientation>;

    void notifyEntry(tc::pd_connection) { claimPhy(); }
    void notifyExit(tc::pd_connection)
    {
        releasePhy();
        resetAll();
    }
    void notifyEntry(plug_orientation line) { phy_.selectCcLine(line); }

    // Stamps the MessageID and the negotiated revision; the rest of
    // the header is the caller's. False when a message or hard reset
    // is already in flight - asked of the state, since the queued
    // machine's process() reports acceptance, not the transition
    bool transmit(pd_message message)
    {
        if (!tx_ready_) { // a message or hard reset is in flight
            return false;
        }
        auto header = pd_header::decode(message.header);
        header.message_id = tx_counter_[index(message.sop)];
        header.revision = revision_;
        message.header = header.encode();
        return sm_.process(prl::event::tx_request{message});
    }

    bool transmitHardReset()
    {
        bool const accepted = phy_.transmit(transmit_signal::hard_reset);
        sm_.process(prl::event::hard_reset_request{});
        resetAll();
        return accepted;
    }

    // The other signals (BIST carrier) go straight to the PHY
    bool transmit(transmit_signal signal) { return phy_.transmit(signal); }

    // BIST Test Data: a PHY that can discard for the engine does
    void setBistTestData(bool enable)
    {
        if constexpr (requires { phy_.setBistTestData(enable); }) {
            phy_.setBistTestData(enable);
        }
    }

    // Soft reset scope: the MessageID lifecycle of one SOP* type; the
    // negotiated revision survives a soft reset
    void reset(sop_type sop)
    {
        tx_counter_[index(sop)] = 0;
        rx_id_[index(sop)].reset();
    }

    // The negotiated revision: lowest seen since attach or hard reset
    pd_revision revision() const { return revision_; }

private:
    // --- the PHY's reports --------------------------------------------------

    static constexpr pd_phy_callbacks phy_callbacks{
        .transmitted =
            [](void* layer, transmit_outcome outcome) {
                static_cast<ProtocolLayer*>(layer)->onTransmitted(outcome);
            },
        .received =
            [](void* layer, pd_message const& message) {
                static_cast<ProtocolLayer*>(layer)->onReceived(message);
            },
        .hard_reset_received =
            [](void* layer) { static_cast<ProtocolLayer*>(layer)->onHardResetReceived(); },
    };

    void onTransmitted(transmit_outcome outcome)
    {
        switch (outcome) {
        case transmit_outcome::success:
            if (auto const* pending = sm_.template getIf<wait_for_phy_response>()) {
                increment(pending->context.message.sop);
                sm_.process(prl::event::phy_success{});
                report(&client_hooks::tx_done);
            } else {
                sm_.process(prl::event::phy_success{}); // hard reset confirmation
            }
            break;
        case transmit_outcome::discarded:
            if (auto const* pending = sm_.template getIf<wait_for_phy_response>()) {
                increment(pending->context.message.sop);
                sm_.process(prl::event::phy_discarded{});
                report(&client_hooks::tx_discarded);
            }
            break;
        case transmit_outcome::failed:
            sm_.process(prl::event::phy_failed{}); // retry or transmission_error
            break;
        }
    }

    // PRL_Rx: a retransmission of a message already delivered is
    // dropped by its MessageID, a lower revision in a SOP message is
    // adopted
    void onReceived(pd_message const& message)
    {
        auto const header = pd_header::decode(message.header);
        auto& stored = rx_id_[index(message.sop)];
        if (stored == header.message_id) {
            return;
        }
        stored = header.message_id;
        if (message.sop == sop_type::sop && header.revision < revision_) {
            setRevision(header.revision); // lowest common revision
        }
        if (hooks_ != nullptr) {
            hooks_->message(client_, message);
        }
    }

    void onHardResetReceived()
    {
        sm_.process(prl::event::reset{});
        resetAll();
        report(&client_hooks::hard_reset);
    }

    // The erased client surface
    struct client_hooks {
        void (*message)(void*, pd_message const&);
        void (*tx_done)(void*);
        void (*tx_discarded)(void*);
        void (*tx_error)(void*);
        void (*hard_reset)(void*);
        void (*hard_reset_sent)(void*);
    };

    template<typename CLIENT>
    static constexpr client_hooks hooks_for = {
        [](void* client, pd_message const& message) {
            static_cast<CLIENT*>(client)->onMessage(message);
        },
        [](void* client) { static_cast<CLIENT*>(client)->onTxDone(); },
        [](void* client) { static_cast<CLIENT*>(client)->onTxDiscarded(); },
        [](void* client) { static_cast<CLIENT*>(client)->onTxError(); },
        [](void* client) { static_cast<CLIENT*>(client)->onHardReset(); },
        [](void* client) { static_cast<CLIENT*>(client)->onHardResetSent(); },
    };

    // A report reaches the client while there is one: the transmit
    // machine's own outcomes (a timeout's transmission error, a hard
    // reset completing) may land after the engine retired
    void report(void (*client_hooks::*hook)(void*))
    {
        if (hooks_ != nullptr) {
            (hooks_->*hook)(client_);
        }
    }
    // Reports the outcomes the state machine reaches on its own -
    // possibly from the serialized timer context. Each observation
    // delivers its own type, so the notify hooks cannot collide. Also
    // answers the table's retry question from the layer's revision
    struct client_reporter : fsm::observing<client_reporter> {
        explicit client_reporter(ProtocolLayer& prl_ref) : prl(prl_ref) {}

        // PRL_Tx_Check_RetryCounter: nRetryCount of the negotiated
        // revision - two retries under PD3, three under PD2
        bool check(prl::retries_left, wait_for_phy_response const& state) const
        {
            return state.context.retry_counter < prl.retryLimit();
        }

        // entering a state whose value is the failed SOP* is the
        // transmission error
        void notifyEntry(sop_type sop) { prl.giveUp(sop); }

        // the tx_ready annotation marks the states accepting a
        // tx_request: transmit()'s gate
        void notifyEntry(prl::tx_ready) { prl.tx_ready_ = true; }
        void notifyExit(prl::tx_ready) { prl.tx_ready_ = false; }

        // leaving a hard_reset_sent-annotated state completes the hard
        // reset - via PHY confirmation, the timer, or a reset event
        void notifyExit(prl::hard_reset_sent) { prl.report(&client_hooks::hard_reset_sent); }

        ProtocolLayer& prl;
    };

    static constexpr std::size_t index(sop_type sop) { return static_cast<std::size_t>(sop); }

    void increment(sop_type sop)
    {
        auto& counter = tx_counter_[index(sop)];
        counter = (counter + 1u) & 0x7u;
    }

    // PRL_Tx_Transmission_Error: MessageIDCounter increments, PE informed
    void giveUp(sop_type sop)
    {
        increment(sop);
        report(&client_hooks::tx_error);
    }

    std::uint8_t retryLimit() const
    {
        return revision_ == pd_revision::rev_3_x ? spec::n_retry_count : spec::n_retry_count_rev2;
    }

    void setRevision(pd_revision rev)
    {
        if (revision_ == rev) {
            return;
        }
        revision_ = rev;
        stampHeader(); // the GoodCRC header carries the negotiated revision
    }

    // The PHY's message header info: the client's roles, the
    // negotiated revision
    void stampHeader()
    {
        phy_.setMessageHeaderInfo(
            {.power = power_role_, .data = data_role_, .revision = revision_}
        );
    }

    // A hard reset and the detach forget the partner: MessageIDs and
    // the revision restart
    void resetAll()
    {
        tx_counter_ = {};
        rx_id_ = {};
        setRevision(pd_revision::rev_3_x);
    }

    PHY& phy_;
    void* client_ = nullptr;
    client_hooks const* hooks_ = nullptr;
    power_role power_role_ = power_role::sink;
    data_role data_role_ = data_role::ufp;
    fsm::QueuedTimer<TIMER> timer_;
    fsm::timed<fsm::QueuedTimer<TIMER>&> timed_;
    prl::phy_driver<PHY> driver_{phy_};
    client_reporter reporter_{*this};
    bool tx_ready_ = false; // set by the initial state's entry below
    // Queued: the client's reaction to a report (a tx_error's soft
    // reset, say) transmits from within the delivering hook - the
    // queue turns that re-entrancy into ordered delivery
    fsm::QueuedMachine<
        prl::tx_table<who_retransmits>,
        4,
        fsm::inline_work,
        fsm::no_lock,
        fsm::timed<fsm::QueuedTimer<TIMER>&>,
        prl::phy_driver<PHY>,
        client_reporter,
        OBSERVERs...>
        sm_;
    std::array<std::uint8_t, prl::sop_count> tx_counter_{};
    std::array<std::optional<std::uint8_t>, prl::sop_count> rx_id_{};
    pd_revision revision_ = pd_revision::rev_3_x;
};

} // namespace usbc
