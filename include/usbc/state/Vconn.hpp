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
 * vconn_off alone and ignores every event. Its own directions sit in
 * the states' annotations: pe_vcs_turn_on_vconn asks for our PS_RDY
 * (the live engine transmits it), the tVCONNSourceTimeout expiry asks
 * for a hard reset - the PdDrp facade executes both on the live
 * engine. The machine running the table is usbc::VconnMachine
 * (layer/Vconn.hpp).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/annotation/PolicyEngine.hpp>
#include <usbc/annotation/Vconn.hpp>
#include <usbc/interface/Questions.hpp>
#include <usbc/spec/Parameters.hpp>

#include <mtl/StateMachine.hpp>

#include <chrono>

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

} // namespace usbc
