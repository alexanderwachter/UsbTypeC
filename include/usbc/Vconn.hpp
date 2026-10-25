/*
 * The VCONN source role as its own machine: the specification's
 * PE_VCS sub-diagram, lifted out of the two policy engines and
 * instantiated once (deviation: the spec draws it inside each engine,
 * entered from Ready - here the engines keep only the VCONN_Swap
 * message states and this machine owns the role, the switch, and the
 * tVCONNSourceTimeout deadline; it survives the engines' teardown
 * across a power role swap, where the spec keeps VCONN untouched).
 *
 * The role begins with the source: a source-role attach with Ra on
 * the non-CC line turns VCONN on, a sink attach leaves it off, and
 * nothing moves it afterwards except a negotiated VCONN_Swap. Every
 * state annotates the switch position; the injected vconn_port
 * observes it (change-suppressed by the observing contract) - the
 * one user object that connects the switch hardware AND arbitrates
 * taking the role (the TCPC interface carries no VCONN; boards route
 * the switch wherever it lives). The sourcing and swap states are
 * the VCONN feature: without an enabling observer the machine runs
 * vconn_off alone and ignores every event.
 *
 * Integration: the machine observes the connection machine (a fresh
 * source-role attach with Ra turns VCONN on, the PD connection's exit
 * turns it off) and both engines' machines (the VCONN_Swap exchange's
 * progress: agreed, the partner's PS_RDY, our PS_RDY on the wire; a
 * hard reset issued by an engine acknowledges the failed hand-off),
 * and answers the engines' question whether this port holds the VCONN
 * Source role. Its own directions sit in the states' annotations:
 * pe_vcs_turn_on_vconn asks for our PS_RDY (the live engine transmits
 * it), the tVCONNSourceTimeout expiry asks for a hard reset - the
 * PdDrp facade executes both on the live engine. Everything runs in
 * the stack's serialized context; the timer fires from its own
 * context (mtl timer contract).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/PolicyEngine.hpp>
#include <usbc/ProtocolLayer.hpp>
#include <usbc/Spec.hpp>
#include <usbc/TypeC.hpp>

#include <mtl/StateMachine.hpp>

#include <chrono>
#include <concepts>

namespace usbc {

namespace vconn {

inline constexpr auto t_source_timeout = std::chrono::milliseconds{120}; // tVCONNSourceTimeout

namespace event {

struct attached_source { // source-role attach; ra: the cable asks
    bool ra;             // to be powered
};
struct detached {};
struct swap_agreed {};         // a VCONN_Swap was accepted (either side)
struct partner_ps_rdy {};      // the new VCONN source announced itself
struct ps_rdy_sent {};         // our announcement is on the wire
struct swap_failed_handled {}; // the requested hard reset is underway

} // namespace event

// The switch annotation, a strong type: the set's element type is
// the key
struct vconn_switch {
    bool on;
    constexpr bool operator==(vconn_switch const&) const = default;
};

// The VCONN Source role (spec term) held: annotated on the states
// holding it - an accepted swap away still counts, VCONN is on until
// the hand-off - and read off the machine by the facade's
// isVconnSource(). Turning the switch on is not holding the role yet
struct source_role {
    constexpr bool operator==(source_role const&) const = default;
};

} // namespace vconn

namespace concepts {

// The one injected object that both connects the VCONN switch
// hardware - observing the states' switch annotation - and answers
// the engines' question whether this port may take the role; the
// answer is what enables the whole feature
template<typename T>
concept vconn_port = requires(T port, vconn::vconn_switch vconn, pe::vconn_swap_allowed question) {
    port.notifyEntry(vconn);
    { port.check(question) } -> std::convertible_to<bool>;
};

} // namespace concepts

namespace vconn {

namespace state {

struct vconn_off {
    static constexpr auto annotations = fsm::annotate(
        vconn_switch{false}
    );

    // a source attach without Ra: nothing to power, stay off
    void handle(event::attached_source const&) {}
};

// The VCONN Source (spec term): sourcing steadily until a swap
struct vconn_source {
    using feature = pe::vconn_feature;

    static constexpr auto annotations = fsm::annotate(
        vconn_switch{true},
        source_role{}
    );
};

// PE_VCS_Turn_On_VCONN: the switch is on; the port announces it with
// PS_RDY through the active engine
struct pe_vcs_turn_on_vconn {
    using feature = pe::vconn_feature;

    static constexpr auto annotations = fsm::annotate(
        vconn_switch{true},
        pe::announce_vconn_on{}
    );
};

// PE_VCS_Wait_For_VCONN: we relinquish - still sourcing until the new
// VCONN source's PS_RDY, due within tVCONNSourceTimeout
struct pe_vcs_wait_for_vconn {
    using feature = pe::vconn_feature;

    static constexpr auto annotations = fsm::annotate(
        vconn_switch{true},
        source_role{}
    );

    static constexpr auto timeout = t_source_timeout; // VCONNOnTimer
};

// The spec's timeout outcome is a hard reset (PE_VCS_Wait_For_VCONN
// -> Hard_Reset): the port-level integration executes it and
// acknowledges; VCONN stays with us
struct pe_vcs_timeout {
    using feature = pe::vconn_feature;

    static constexpr auto annotations = fsm::annotate(
        vconn_switch{true},
        pe::request_hard_reset{},
        source_role{}
    );
};

} // namespace state

struct ra_present {
    static bool check(state::vconn_off const&, event::attached_source const& event)
    {
        return event.ra;
    }
};

using vconn_timer_ranges =
    fsm::timer_ranges<fsm::timed_by<state::pe_vcs_wait_for_vconn, spec::t_vconn_source_timeout>>;

using vconn_transitions = fsm::transition_table<
    fsm::initial<state::vconn_off>,
    fsm::transition<fsm::from<state::vconn_off>, fsm::on<event::attached_source>, fsm::guard<ra_present>, fsm::to<state::vconn_source>>,
    fsm::internal_transition<fsm::from<state::vconn_off>, fsm::on<event::attached_source>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::detached>, fsm::to<state::vconn_off>>,
    // the agreed swap: take when off, relinquish when sourcing
    fsm::transition<fsm::from<state::vconn_off>, fsm::on<event::swap_agreed>, fsm::to<state::pe_vcs_turn_on_vconn>>,
    fsm::transition<fsm::from<state::vconn_source>, fsm::on<event::swap_agreed>, fsm::to<state::pe_vcs_wait_for_vconn>>,
    fsm::transition<fsm::from<state::pe_vcs_turn_on_vconn>, fsm::on<event::ps_rdy_sent>, fsm::to<state::vconn_source>>,
    fsm::transition<fsm::from<state::pe_vcs_wait_for_vconn>, fsm::on<event::partner_ps_rdy>, fsm::to<state::vconn_off>>,
    fsm::transition<fsm::from<state::pe_vcs_wait_for_vconn>, fsm::on<fsm::timeout>, fsm::to<state::pe_vcs_timeout>>,
    fsm::transition<fsm::from<state::pe_vcs_timeout>, fsm::on<event::swap_failed_handled>, fsm::to<state::vconn_source>>>;

// A named struct, not an alias: the short name replaces the fully
// spelled table type in every mangled symbol
struct vconn_table : vconn_transitions {};
// timeout bounds and reachability checked in test/compliance.cpp

} // namespace vconn

// The observers carry the vconn_port that enables the feature and
// drives the switch; without one the machine runs vconn_off alone
template<fsm::concepts::timer TIMER, typename... OBSERVERs>
class VconnMachine : public fsm::observing<VconnMachine<TIMER, OBSERVERs...>> {
public:
    VconnMachine(TIMER& timer, OBSERVERs&... observers)
        : timer_(timer), timed_(timer_), sm_(timed_, observers...)
    {
    }

    // The connection machine's facts: only a fresh source-role attach
    // with Ra starts sourcing (a swap's completion preserves whatever
    // the role is), the one real detach ends it. Observed ahead of the
    // source engine: switching VCONN reconfigures the CC lines, which
    // aborts a transmission in flight
    void notifyEntry(tc::attached_as_source const& attach)
    {
        if (attach.origin == tc::attach_origin::fresh_attach) {
            attachedSource(tc::raPresent(attach.cc));
        }
    }
    void notifyExit(tc::pd_connection) { detached(); }

    // The engines' VCONN_Swap progress, observed in both engines'
    // machines; a hard reset issued by an engine is the escalation of
    // a failed hand-off, which leaves VCONN here
    void notifyEntry(pe::vconn_swap_agreed) { swapAgreed(); }
    void notifyEntry(pe::vconn_partner_on) { partnerPsRdy(); }
    void notifyEntry(pe::vconn_ps_rdy_sent) { psRdySent(); }
    void notifyEntry(prl::hard_reset_action) { swapFailureHandled(); }

    // The engines' question on the partner's VCONN_Swap: a holder of
    // the role accepts, relinquishing needs no capability
    bool check(pe::vconn_source_role_held) const { return isVconnSource(); }

    // The same, as calls
    void attachedSource(bool ra) { sm_.process(vconn::event::attached_source{ra}); }
    void detached() { sm_.process(vconn::event::detached{}); }
    void swapAgreed() { sm_.process(vconn::event::swap_agreed{}); }
    void partnerPsRdy() { sm_.process(vconn::event::partner_ps_rdy{}); }
    void psRdySent() { sm_.process(vconn::event::ps_rdy_sent{}); }
    void swapFailureHandled() { sm_.process(vconn::event::swap_failed_handled{}); }

    // Whether this port holds the VCONN Source role, as the states
    // annotate it (an accepted swap away from giving it up still
    // counts - VCONN is on until the hand-off)
    bool isVconnSource() const { return sm_.template annotation<vconn::source_role>().has_value(); }

private:
    fsm::QueuedTimer<TIMER> timer_;
    fsm::timed<fsm::QueuedTimer<TIMER>&> timed_;
    // Queued: the tVCONNSourceTimeout expiry's hard-reset request is
    // acted on from the observing hook, which feeds back into this
    // machine (swapFailureHandled) - ordered delivery, no facade pump
    fsm::QueuedMachine<
        vconn::vconn_table,
        2,
        fsm::inline_work,
        fsm::no_lock,
        fsm::timed<fsm::QueuedTimer<TIMER>&>,
        OBSERVERs...>
        sm_;
};

} // namespace usbc
