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
 * state annotates the switch position; the vconn_driver observer
 * applies it with change suppression through the injected vconn_port
 * - the one user object that connects the switch hardware AND
 * arbitrates taking the role (the TCPC interface carries no VCONN;
 * boards route the switch wherever it lives).
 *
 * Integration (the PdDrp facade wires all of it): feed the Type-C
 * attach resolution through attachedSource(ra)/detached(), and the
 * engines' VCONN_Swap progress through swapAgreed()/partnerPsRdy()/
 * psRdySent(). Observers injected into the machine see the states'
 * portReport() requests: pe_vcs_turn_on_vconn asks for our PS_RDY
 * (the active engine transmits it), and the tVCONNSourceTimeout
 * expiry asks for a hard reset, acknowledged with
 * swapFailureHandled(). Everything runs in the stack's serialized
 * context; the timer fires from its own context (mtl timer contract).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/PolicyEngine.hpp>
#include <usbc/Spec.hpp>

#include <mtl/StateMachine.hpp>

#include <chrono>
#include <concepts>

namespace usbc {

namespace concepts {

// The one injected object that both connects the VCONN switch
// hardware and arbitrates taking the role - its presence among a
// port's observers enables the whole feature
template<typename T>
concept vconn_port = requires(T port, bool on, vconn_source_role role) {
    { port.setVconn(on) } -> std::convertible_to<bool>;
    { port.allowSwap(role) } -> std::convertible_to<bool>;
};

} // namespace concepts

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
    static constexpr auto annotations = fsm::annotate(
        vconn_switch{true},
        source_role{}
    );
};

// PE_VCS_Turn_On_VCONN: the switch is on; the port announces it with
// PS_RDY through the active engine
struct pe_vcs_turn_on_vconn {
    static constexpr auto annotations = fsm::annotate(
        vconn_switch{true},
        pe::announce_vconn_on{}
    );
};

// PE_VCS_Wait_For_VCONN: we relinquish - still sourcing until the new
// VCONN source's PS_RDY, due within tVCONNSourceTimeout
struct pe_vcs_wait_for_vconn {
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

// Applies each state's switch annotation through the injected
// vconn_port hardware connector; suppressed while unchanged
template<typename VCONN_PORT>
struct vconn_driver : fsm::observing<vconn_driver<VCONN_PORT>> {
    explicit vconn_driver(VCONN_PORT& port_ref) : port(port_ref) {}

    void notifyEntry(vconn_switch vconn)
    {
        if (vconn.on != applied) {
            applied = vconn.on;
            port.setVconn(vconn.on);
        }
    }

    VCONN_PORT& port;
    bool applied = false;
};

} // namespace vconn

template<concepts::vconn_port VCONN_PORT, fsm::concepts::timer TIMER, typename... OBSERVERs>
class VconnMachine {
public:
    VconnMachine(VCONN_PORT& vconn_port, TIMER& timer, OBSERVERs&... observers)
        : driver_(vconn_port), timer_(timer), timed_(timer_), sm_(timed_, driver_, observers...)
    {
    }

    // The Type-C resolution: only a source-role attach with Ra starts
    // sourcing; a swap entry preserves whatever the role is
    void attachedSource(bool ra) { sm_.process(vconn::event::attached_source{ra}); }
    void detached() { sm_.process(vconn::event::detached{}); }

    // The engines' VCONN_Swap progress
    void swapAgreed() { sm_.process(vconn::event::swap_agreed{}); }
    void partnerPsRdy() { sm_.process(vconn::event::partner_ps_rdy{}); }
    void psRdySent() { sm_.process(vconn::event::ps_rdy_sent{}); }
    void swapFailureHandled() { sm_.process(vconn::event::swap_failed_handled{}); }

    // Whether this port holds the VCONN Source role, as the states
    // annotate it (an accepted swap away from giving it up still
    // counts - VCONN is on until the hand-off)
    bool isVconnSource() const { return sm_.template annotation<vconn::source_role>().has_value(); }

private:
    vconn::vconn_driver<VCONN_PORT> driver_;
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
        vconn::vconn_driver<VCONN_PORT>,
        OBSERVERs...>
        sm_;
};

} // namespace usbc
