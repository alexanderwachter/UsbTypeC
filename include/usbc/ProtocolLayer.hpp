/*
 * USB PD protocol layer (PRL), following the spec's PRL_Tx/PRL_Rx/PRL_HR
 * as closely as the driver split allows. The driver owns the bit level -
 * CRC checking, GoodCRC autoresponse, per-attempt outcome detection -
 * and everything above lives here: MessageID stamping, the RetryCounter
 * with the retransmission loop, the protocol timers, duplicate
 * rejection on receive, and the MessageID lifecycle across soft and
 * hard resets. Chunked extended messages are not handled yet.
 *
 * Transmit is a state machine with the pending message and RetryCounter
 * in machine-owned context. wait_for_phy_response runs CRCReceiveTimer
 * (tReceive): a failed attempt - reported by the driver or by the timer
 * - retransmits with the same MessageID while RetryCounter allows
 * (guarded self-transition), then lands in transmission_error, which
 * reports onTxError, increments the MessageIDCounter, and rests until
 * the policy engine transmits again or resets. Success and discard
 * outcomes arrive as alerts and report onTxDone/onTxDiscarded; the
 * MessageIDCounter increments on these completions only - never per
 * attempt. transmit() while a message is in flight is refused - the
 * policy engine serializes its requests.
 *
 * Hard reset: transmitHardReset() hands the signal to the driver and
 * waits in wait_for_hard_reset_complete, bounded by
 * HardResetCompleteTimer (tHardResetComplete); PHY confirmation or the
 * timer completes it and reports onHardResetSent.
 *
 * Execution contract: the transmit functions, reset, and onAlert run
 * in the stack's
 * context. The TIMER policy fires fsm::timeout from its own execution
 * context (mtl timer contract), and the integrator serializes it with
 * the stack's calls; client callbacks on timeout paths (onTxError,
 * onHardResetSent) originate from that serialized timer context.
 *
 * Revision negotiation lives here, at the same choke point as the
 * MessageID: the layer starts at rev 3.x, adopts the lowest revision
 * seen in received SOP messages, stamps it into every transmitted
 * header, and switches nRetryCount (2 for rev 3.x, 3 for rev 2.0)
 * per message. A client providing onRevision(pd_revision) learns of
 * adoptions - the policy engines update the TCPC's GoodCRC header
 * there. The negotiated revision survives a soft reset (reset()),
 * restarts at rev 3.x on a hard reset, and the policy engine clears
 * it on detach (resetRevision()).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Message.hpp>
#include <usbc/Spec.hpp>
#include <usbc/Tcpc.hpp>

#include <mtl/StateMachine.hpp>

#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace usbc {

namespace concepts {

template<typename T>
concept prl_client = requires(T client, pd_message const& message) {
    client.onMessage(message);
    client.onTxDone();
    client.onTxDiscarded();
    client.onTxError();
    client.onHardReset();      // hard reset received from the partner
    client.onHardResetSent(); // own hard reset signal is on the wire
};

} // namespace concepts

namespace prl {

inline constexpr std::uint8_t n_retry_count = spec::n_retry_count; // nRetryCount, PD rev 3.x

inline constexpr std::size_t sop_count = 5; // SOP* types with own MessageID lifecycles

// The per-SOP* MessageID state: the transmit counters and the receive
// side's stored copies. Handed across a power-role swap like the
// negotiated revision: 6.7.1 resets the counters only at power-on,
// Hard Reset, Cable Reset or Soft_Reset - a PR_Swap continues them,
// even though this stack changes engines (and with them protocol
// layer instances) mid-swap
struct message_id_state {
    std::array<std::uint8_t, sop_count> tx_counter{};
    std::array<std::optional<std::uint8_t>, sop_count> rx_id{};
};

inline constexpr auto t_receive             = std::chrono::milliseconds{1}; // tReceive
inline constexpr auto t_hard_reset_complete = std::chrono::milliseconds{5}; // tHardResetComplete

// Shared by the transmitting states: the message in flight survives
// the timeout-driven retransmission transitions. The retry limit is
// per message - it follows the negotiated revision
struct tx_context {
    pd_message message{};
    std::uint8_t retry_counter = 0;
    std::uint8_t retry_limit   = n_retry_count;
};

namespace event {

struct tx_request {
    pd_message message;
    std::uint8_t retry_limit = n_retry_count;
};
struct phy_success {};
struct phy_discarded {};
struct phy_failed {};
struct hard_reset_request {};
struct reset {};

} // namespace event

// Annotation tag: leaving a state carrying it completes a hard reset
struct hard_reset_sent {
    constexpr bool operator==(hard_reset_sent const&) const = default;
};

// Annotation tag: a state carrying it accepts a tx_request - the
// layer's transmit() gate, tracked by the client reporter
struct tx_ready {
    constexpr bool operator==(tx_ready const&) const = default;
};

// Commands to the protocol layer: a policy engine state carries one as
// a static prl_action member and the ProtocolLayer, injected into that
// machine as an observer, executes it on entry. The note strings feed
// the states' dot_action diagram labels
struct reset_action {
    static constexpr std::string_view note = "resets the protocol layer";
    // re-resetting an already reset layer changes nothing: a wildcard's
    // entry may re-notify it
    constexpr bool operator==(reset_action const&) const = default;
};
struct hard_reset_action { // not repeatable: a notification transmits
    static constexpr std::string_view note = "requests a hard reset";
    constexpr bool operator==(hard_reset_action const&) const = default;
};

namespace state {

struct wait_for_message_request {
    static constexpr auto annotations = fsm::annotate(tx_ready{});
};

struct wait_for_phy_response {
    static constexpr auto timeout = t_receive; // CRCReceiveTimer

    wait_for_phy_response(event::tx_request const& event, tx_context& ctx) : context(ctx)
    {
        context.message       = event.message;
        context.retry_counter = 0;
        context.retry_limit   = event.retry_limit;
    }
    // Re-entry is the retransmission: same message, same MessageID
    explicit wait_for_phy_response(tx_context& ctx) : context(ctx) { ++context.retry_counter; }

    // the message in flight, observed by the phy driver
    pd_message const& values() const { return context.message; }

    tx_context& context;
};

// PRL_Tx_Transmission_Error folded with the idle wait: reported on
// entry, rests until the policy engine transmits again or resets
struct transmission_error {
    static constexpr auto annotations = fsm::annotate(tx_ready{});

    explicit transmission_error(tx_context& ctx) : context(ctx) {}

    // the failed message's SOP*, observed by the client reporter
    sop_type values() const { return context.message.sop; }

    tx_context& context;
};

struct wait_for_hard_reset_complete {
    static constexpr auto timeout = t_hard_reset_complete; // HardResetCompleteTimer

    // leaving this state completes the hard reset, whichever edge takes
    // it out; observed on exit by the client reporter
    static constexpr auto annotations = fsm::annotate(hard_reset_sent{});
};

} // namespace state

// PRL_Tx_Check_RetryCounter as a transition guard
struct retries_left {
    static bool check(state::wait_for_phy_response const& state)
    {
        return state.context.retry_counter < state.context.retry_limit;
    }
};

// The spec timer range of every timed state, checked against the table
using prl_timer_ranges = mtl::typelist<
    fsm::timed_by<state::wait_for_phy_response, spec::t_receive>,
    fsm::timed_by<state::wait_for_hard_reset_complete, spec::t_hard_reset_complete>>;

// A named struct, not an alias: the short name replaces the fully
// spelled table type in every mangled symbol
struct tx_table : fsm::transition_table<
    fsm::initial<state::wait_for_message_request>,
    fsm::transition<fsm::from<state::wait_for_message_request>, fsm::on<event::tx_request>,
                    fsm::to<state::wait_for_phy_response>>,
    fsm::transition<fsm::from<state::transmission_error>, fsm::on<event::tx_request>,
                    fsm::to<state::wait_for_phy_response>>,
    // no GoodCRC in time: retransmit while RetryCounter allows, else error
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<fsm::timeout>,
                    fsm::to<state::wait_for_phy_response>, fsm::guard<retries_left>>,
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<fsm::timeout>,
                    fsm::to<state::transmission_error>>,
    // the driver may report a failed attempt before tReceive expires
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<event::phy_failed>,
                    fsm::to<state::wait_for_phy_response>, fsm::guard<retries_left>>,
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<event::phy_failed>,
                    fsm::to<state::transmission_error>>,
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<event::phy_success>,
                    fsm::to<state::wait_for_message_request>>,
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<event::phy_discarded>,
                    fsm::to<state::wait_for_message_request>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::hard_reset_request>,
                    fsm::to<state::wait_for_hard_reset_complete>>,
    fsm::transition<fsm::from<state::wait_for_hard_reset_complete>, fsm::on<event::phy_success>,
                    fsm::to<state::wait_for_message_request>>,
    fsm::transition<fsm::from<state::wait_for_hard_reset_complete>, fsm::on<fsm::timeout>,
                    fsm::to<state::wait_for_message_request>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::reset>,
                    fsm::to<state::wait_for_message_request>>> {};
// timeout bounds and reachability checked in test/compliance.cpp

// Hands a state's pd_message instance value to the TCPC on entry; a
// pd_message in a state's values() is the marker that makes it a
// transmitting one. A refused hand-off is not reported:
// CRCReceiveTimer turns it into a retry
template<concepts::pd_transport TCPC>
struct phy_driver : fsm::observing<phy_driver<TCPC>> {
    explicit phy_driver(TCPC& tcpc_ref) : tcpc(tcpc_ref) {}

    using observes = mtl::typelist<pd_message>;

    void notifyEntry(pd_message const& message) { tcpc.transmit(message); }

    TCPC& tcpc;
};

} // namespace prl

// The client is type-erased behind plain function pointers (the
// setIdleHook idiom - no RTTI, no virtuals): the layer is fully
// role-independent, and templating it on the client duplicated the
// whole PRL - tx machine, receive drain, counters - once per policy
// engine (measured ~2 kB flash in a DRP image)
template<concepts::pd_transport TCPC, fsm::concepts::timer TIMER>
class ProtocolLayer : public fsm::observing<ProtocolLayer<TCPC, TIMER>> {
public:
    template<concepts::prl_client CLIENT>
    ProtocolLayer(TCPC& tcpc, TIMER& timer, CLIENT& client)
        : tcpc_(tcpc), client_(&client), hooks_(&hooks_for<CLIENT>), timer_(timer),
          timed_(timer_)
    {
    }

    // The protocol layer is itself an observer of the policy engine's
    // machine: states command it through the action elements of their
    // annotation sets and transmit through a pd_message in their
    // values(). The observing contract delivers the static set before
    // the instance values, so a state carrying both resets first and
    // its message goes out with MessageID 0
    void notifyEntry(prl::reset_action) { reset(sop_type::sop); }
    void notifyEntry(prl::hard_reset_action) { transmitHardReset(); }
    void notifyEntry(pd_message const& message) { transmit(message); }

    // Stamps the MessageID and the negotiated revision; the rest of
    // the header is the caller's. False when a message or hard reset
    // is already in flight - asked of the state, since the queued
    // machine's process() reports acceptance, not the transition
    bool transmit(pd_message message)
    {
        if (!tx_ready_) { // a message or hard reset is in flight
            return false;
        }
        auto header       = pd_header::decode(message.header);
        header.message_id = tx_counter_[index(message.sop)];
        header.revision   = revision_;
        message.header    = header.encode();
        return sm_.process(prl::event::tx_request{message, retryLimit()});
    }

    bool transmitHardReset()
    {
        bool const accepted = tcpc_.transmit(transmit_signal::hard_reset);
        sm_.process(prl::event::hard_reset_request{});
        resetAll();
        return accepted;
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

    // Detach forgets the partner; the DRP facade seeds the retiring
    // engine's negotiated revision into the relieving one on a power
    // role swap (the revision holds for the connection)
    void resetRevision() { setRevision(pd_revision::rev_3_x); }
    void seedRevision(pd_revision rev) { setRevision(rev); }

    // The MessageID lifecycle, handed over the same way: the swap is
    // no reset trigger (6.7.1), the counters continue
    prl::message_id_state messageIds() const { return {tx_counter_, rx_id_}; }
    void seedMessageIds(prl::message_id_state const& ids)
    {
        tx_counter_ = ids.tx_counter;
        rx_id_      = ids.rx_id;
    }

    void onAlert(alert_status alerts)
    {
        if (any(alerts & alert_status::hard_reset_received)) {
            sm_.process(prl::event::reset{});
            resetAll();
            hooks_->hard_reset(client_);
        }
        if (any(alerts & alert_status::transmit_success)) {
            if (auto const* pending = sm_.template getIf<prl::state::wait_for_phy_response>()) {
                increment(pending->context.message.sop);
                sm_.process(prl::event::phy_success{});
                hooks_->tx_done(client_);
            } else {
                sm_.process(prl::event::phy_success{}); // hard reset confirmation
            }
        }
        if (any(alerts & alert_status::transmit_discarded)) {
            if (auto const* pending = sm_.template getIf<prl::state::wait_for_phy_response>()) {
                increment(pending->context.message.sop);
                sm_.process(prl::event::phy_discarded{});
                hooks_->tx_discarded(client_);
            }
        }
        if (any(alerts & alert_status::transmit_failed)) {
            sm_.process(prl::event::phy_failed{}); // retry or transmission_error
        }
        if (any(alerts & alert_status::message_received)) {
            drainReceived();
        }
    }

private:
    // The erased client surface; revision is nullptr for a client
    // without the optional onRevision hook
    struct client_hooks {
        void (*message)(void*, pd_message const&);
        void (*tx_done)(void*);
        void (*tx_discarded)(void*);
        void (*tx_error)(void*);
        void (*hard_reset)(void*);
        void (*hard_reset_sent)(void*);
        void (*revision)(void*, pd_revision);
    };

    template<typename CLIENT>
    static constexpr auto revisionHook()
    {
        if constexpr (requires(CLIENT client, pd_revision rev) { client.onRevision(rev); }) {
            return +[](void* client, pd_revision rev) {
                static_cast<CLIENT*>(client)->onRevision(rev);
            };
        } else {
            return static_cast<void (*)(void*, pd_revision)>(nullptr);
        }
    }

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
        revisionHook<CLIENT>(),
    };
    // Reports the outcomes the state machine reaches on its own -
    // possibly from the serialized timer context. Each observation
    // delivers its own type, so the notify hooks cannot collide
    struct client_reporter : fsm::observing<client_reporter> {
        explicit client_reporter(ProtocolLayer& prl_ref) : prl(prl_ref) {}

        // entering a state whose value is the failed SOP* is the
        // transmission error
        void notifyEntry(sop_type sop) { prl.giveUp(sop); }

        // the tx_ready annotation marks the states accepting a
        // tx_request: transmit()'s gate
        void notifyEntry(prl::tx_ready) { prl.tx_ready_ = true; }
        void notifyExit(prl::tx_ready) { prl.tx_ready_ = false; }

        // leaving a hard_reset_sent-annotated state completes the hard
        // reset - via PHY confirmation, the timer, or a reset event
        void notifyExit(prl::hard_reset_sent) { prl.hooks_->hard_reset_sent(prl.client_); }

        ProtocolLayer& prl;
    };

    static constexpr std::size_t index(sop_type sop) { return static_cast<std::size_t>(sop); }

    void increment(sop_type sop)
    {
        auto& counter = tx_counter_[index(sop)];
        counter       = (counter + 1u) & 0x7u;
    }

    // PRL_Tx_Transmission_Error: MessageIDCounter increments, PE informed
    void giveUp(sop_type sop)
    {
        increment(sop);
        hooks_->tx_error(client_);
    }

    void drainReceived()
    {
        pd_message message;
        while (tcpc_.receive(message)) {
            auto const header = pd_header::decode(message.header);
            auto& stored      = rx_id_[index(message.sop)];
            if (stored == header.message_id) {
                continue; // retransmission of a message already delivered
            }
            stored = header.message_id;
            if (message.sop == sop_type::sop && header.revision < revision_) {
                setRevision(header.revision); // lowest common revision
            }
            hooks_->message(client_, message);
        }
    }

    std::uint8_t retryLimit() const
    {
        return revision_ == pd_revision::rev_3_x ? spec::n_retry_count
                                                 : spec::n_retry_count_rev2;
    }

    void setRevision(pd_revision rev)
    {
        if (revision_ == rev) {
            return;
        }
        revision_ = rev;
        if (hooks_->revision != nullptr) {
            hooks_->revision(client_, rev); // e.g. refresh the GoodCRC header
        }
    }

    void resetAll()
    {
        tx_counter_ = {};
        rx_id_      = {};
        setRevision(pd_revision::rev_3_x); // re-negotiated after a hard reset
    }

    TCPC& tcpc_;
    void* client_;
    client_hooks const* hooks_;
    fsm::QueuedTimer<TIMER> timer_;
    fsm::timed<fsm::QueuedTimer<TIMER>&> timed_;
    prl::phy_driver<TCPC> driver_{tcpc_};
    client_reporter reporter_{*this};
    bool tx_ready_ = false; // set by the initial state's entry below
    // Queued: the client's reaction to a report (a tx_error's soft
    // reset, say) transmits from within the delivering hook - the
    // queue turns that re-entrancy into ordered delivery
    fsm::QueuedMachine<prl::tx_table, 4, fsm::inline_work, fsm::no_lock,
                       fsm::timed<fsm::QueuedTimer<TIMER>&>, prl::phy_driver<TCPC>,
                       client_reporter>
        sm_{timed_, driver_, reporter_};
    std::array<std::uint8_t, prl::sop_count> tx_counter_{};
    std::array<std::optional<std::uint8_t>, prl::sop_count> rx_id_{};
    pd_revision revision_ = pd_revision::rev_3_x;
};

} // namespace usbc
