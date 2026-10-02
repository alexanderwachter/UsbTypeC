/*
 * USB Type-C sink connection layer: Unattached.SNK, AttachWait.SNK and
 * Attached.SNK per the USB Type-C Cable and Connector Specification,
 * driving a tcpc and a vbus driver. No PD involved - the layer detects
 * attach/detach, resolves plug orientation and the Rp current
 * advertisement, and switches the sink power path.
 *
 * The machine keeps the latest CC status and VBUS presence in
 * machine-owned context; VBUS and CC updates that must not disturb the
 * running debounce are internal transitions. AttachWait.SNK debounces
 * CC for tCCDebounce (one debounce time is used for attach and detach;
 * the spec's shorter tPDDebounce for detach detection is not used yet).
 * After a stable single-Rp result the machine attaches as soon as VBUS
 * is present - immediately at the debounce timeout or later on the
 * VBUS event. Attached.SNK watches vSinkDisconnect instead of vSafe5V;
 * the watched level is a state annotation and the vbus_watcher observer
 * re-arms the vbus driver on changes.
 *
 * Integration: hand over initialized drivers and a caller-owned timer
 * policy instance. Construction rests in the spec's Disabled state
 * (open terminations, nothing monitored); start() is the go-live
 * moment - it fires the started event, whose transition applies the
 * Unattached.SNK terminations and monitoring, then registers the tcpc
 * alert handler and the vbus callback and seeds the CC state when a
 * source is already present. The drivers must invoke their callbacks
 * from the stack's serialized context (the Zephyr adapters deliver on
 * a workqueue), and the timer is serialized with them by the
 * integrator (mtl timer contract). Attach results reach injected
 * observers watching the attached state's attachedInfo(); their
 * notifications may originate from the timer context on
 * debounce-timeout paths.
 *
 * DRP and Try.SNK/Try.SRC build on this layer's states in
 * TypeCDrp.hpp. Not covered yet: debug and audio accessories (both-Rp
 * results detach), Rp change notification while attached, and VCONN (a
 * sink without cable communication needs none).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Tcpc.hpp>
#include <usbc/TypeC.hpp>
#include <usbc/Vbus.hpp>

#include <mtl/StateMachine.hpp>
#include <mtl/Typelist.hpp>

#include <concepts>
#include <tuple>
#include <type_traits>

namespace usbc {

namespace tc {

constexpr bool singleRp(cc_status status)
{
    return isRp(status.cc1) != isRp(status.cc2);
}

constexpr plug_orientation orientationOf(cc_status status)
{
    return isRp(status.cc1) ? plug_orientation::cc1 : plug_orientation::cc2;
}

constexpr rp_value advertisementOf(cc_status status)
{
    switch (isRp(status.cc1) ? status.cc1 : status.cc2) {
    case cc_state::snk_power_1a5: return rp_value::p_1a5;
    case cc_state::snk_power_3a0: return rp_value::p_3a0;
    default: return rp_value::usb_default;
    }
}

// Delivered to the client on attach, observed on the attached state
struct attach_info {
    plug_orientation orientation;
    rp_value advertisement;
};

namespace state {

// The spec's Disabled state: the port is not operating, terminations
// removed, nothing monitored. start() fires the started event
struct disabled_snk {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::open},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched}
    );
};

// The spec's ErrorRecovery state: both terminations removed for at
// least tErrorRecovery, then connection resolution restarts from the
// table's unattached anchor. Entered on the PD layer's command (e.g.
// nHardResetCount exhausted)
struct error_recovery {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::open},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched}
    );
    static constexpr auto timeout = t_error_recovery;
};

// The sensed line plus the internal-transition handlers keeping it
// current without disturbing a running debounce
struct sink_state {
    explicit sink_state(line_status& line_ref) : line(line_ref) {}

    void handle(event::cc_changed const& event) { line.cc = event.cc; }
    void handle(event::vbus_present const&) { line.vbus_present = true; }
    void handle(event::vbus_removed const&) { line.vbus_present = false; }

    using contexts = mtl::typelist<line_status>;
    line_status& line;
};

// The resting state resets the connection's attachment: the next
// attach resolves afresh
struct unattached_snk : sink_state {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::safe5v}
    );

    unattached_snk(line_status& line_ref, attachment& attached) : sink_state(line_ref)
    {
        attached = {};
    }

    using contexts = mtl::typelist<line_status, attachment>;
};

struct attach_wait_snk : sink_state {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::safe5v}
    );
    static constexpr auto timeout = t_cc_debounce; // CCDebounceTimer

    attach_wait_snk(event::cc_changed const& event, line_status& line_ref) : sink_state(line_ref)
    {
        line.cc = event.cc;
    }
    using sink_state::sink_state;
};

// AttachWait.SNK with a stable single Rp, waiting for VBUS
struct attach_wait_snk_debounced : sink_state {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::safe5v}
    );

    using sink_state::sink_state;
};

struct attached_snk : sink_state {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::sink},
        vbus_level{vbus_level::sink_disconnect},
        attached_role{power_role::sink},
        pd_connection{}
    );

    // the debounce timed out with VBUS already present: a fresh attach
    attached_snk(line_status& line_ref, attachment& attached_ref)
        : sink_state(line_ref), attached(attached_ref)
    {
        attachFresh();
    }
    // entered on the VBUS report: a fresh attach (the debounced wait)
    // resolves plug and data role; VBUS returning after a hard reset
    // resumes the connection unchanged
    attached_snk(event::vbus_present const&, line_status& line_ref, attachment& attached_ref)
        : sink_state(line_ref), attached(attached_ref)
    {
        if (attached.resolved) {
            origin_        = attach_origin::resumed_after_hard_reset;
            advertisement_ = advertisementOf(line.cc);
        } else {
            attachFresh();
        }
    }
    // entered on a CC event (the DRP's TryWait.SNK attach): the payload
    // lands on the line before the attach is resolved from it
    attached_snk(event::cc_changed const& event, line_status& line_ref, attachment& attached_ref)
        : sink_state(line_ref), attached(attached_ref)
    {
        line.cc = event.cc;
        attachFresh();
    }
    // entered from a PD-directed role swap's standby (the DRP layer),
    // completed or aborted: the plug stays put - the attachment is
    // already resolved - and the current draw is governed by the PD
    // contract, not the Rp advertisement
    attached_snk(event::swap_complete const&, line_status& line_ref, attachment& attached_ref)
        : sink_state(line_ref), attached(attached_ref),
          origin_(attach_origin::completed_power_role_swap)
    {
    }
    attached_snk(event::swap_abort const&, line_status& line_ref, attachment& attached_ref)
        : sink_state(line_ref), attached(attached_ref)
    {
    }

    using sink_state::handle;
    // a DR_Swap flips the data role in place (DRP only)
    void handle(event::swap_data_role const&) { attached.data = otherDataRole(attached.data); }

    plug_orientation orientation() const { return attached.orientation; }
    data_role dataRole() const { return attached.data; }
    // the attach result, the CC polarity and the partner, observed as
    // instance values: the hw driver applies the polarity, the clients
    // and loggers consume the attach info, the PD layer above acts on
    // the partner (what brought the port here, data role, the
    // source's Rp)
    auto values() const
    {
        return fsm::annotate(
            attach_info{.orientation = attached.orientation, .advertisement = advertisement_},
            polarity{.orientation = attached.orientation},
            attached_partner{.origin = origin_, .data = attached.data, .cc = line.cc});
    }

    using contexts = mtl::typelist<line_status, attachment>;
    attachment& attached;

private:
    // a fresh attach resolves the plug from the line and takes the
    // sink's default data role
    void attachFresh()
    {
        attached = {.orientation = orientationOf(line.cc), .data = data_role::ufp,
                    .resolved    = true};
        advertisement_ = advertisementOf(line.cc);
    }

    rp_value advertisement_ = rp_value::usb_default;
    attach_origin origin_   = attach_origin::fresh_attach;
};

// The hard-reset window, first phase: the source legitimately drops
// VBUS to vSafe0V - not a detach. Rd stays presented, the sink path
// is off, the PD connection holds (the engine awaits the source's
// capabilities). The policy engine's NoResponseTimer owns the give-up;
// the timeout here only terminates a dead port
struct hard_reset_snk : sink_state {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::safe5v},
        pd_connection{}
    );
    static constexpr auto timeout = t_hard_reset_window;

    using sink_state::sink_state;
};

// ... second phase: VBUS is down, its return resumes Attached.SNK
struct hard_reset_recover_snk : sink_state {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::safe5v},
        pd_connection{}
    );
    static constexpr auto timeout = t_hard_reset_window;

    using sink_state::sink_state;
};

} // namespace state

// Guards on the debounce outcome - the line's latest CC status and
// VBUS report - as primitives the rows combine
struct stable_rp {
    static bool check(state::attach_wait_snk const& state) { return singleRp(state.line.cc); }
};

// VBUS present per the line, in any state (the DRP's Try flows ask too)
struct vbus_present_in_context {
    static bool check(auto const& state) { return state.line.vbus_present; }
};

// The sink attach flow, shared with the DRP layer: UNATTACHED anchors
// it (the sink's resting state, or the DRP's toggling Rd phase) and
// ATTACH is where a successful attach leads (Attached.SNK, or Try.SRC
// for a source-preferring DRP)
template<typename UNATTACHED, typename ATTACH>
using sink_attach_flow = mtl::typelist<
    fsm::transition<fsm::from<UNATTACHED>, fsm::on<event::cc_changed>,
                    fsm::to<state::attach_wait_snk>>,
    fsm::internal_transition<fsm::from<UNATTACHED>, fsm::on<event::vbus_present>>,
    fsm::internal_transition<fsm::from<UNATTACHED>, fsm::on<event::vbus_removed>>,
    // a CC change during the debounce restarts it
    fsm::transition<fsm::from<state::attach_wait_snk>, fsm::on<event::cc_changed>,
                    fsm::to<state::attach_wait_snk>>,
    fsm::internal_transition<fsm::from<state::attach_wait_snk>, fsm::on<event::vbus_present>>,
    fsm::internal_transition<fsm::from<state::attach_wait_snk>, fsm::on<event::vbus_removed>>,
    // debounce complete: attach, keep waiting for VBUS, or detach
    fsm::transition<fsm::from<state::attach_wait_snk>, fsm::on<fsm::timeout>,
                    fsm::to<ATTACH>, fsm::guard<stable_rp, vbus_present_in_context>>,
    fsm::transition<fsm::from<state::attach_wait_snk>, fsm::on<fsm::timeout>,
                    fsm::to<state::attach_wait_snk_debounced>, fsm::guard<stable_rp>>,
    fsm::transition<fsm::from<state::attach_wait_snk>, fsm::on<fsm::timeout>,
                    fsm::to<UNATTACHED>>,
    fsm::transition<fsm::from<state::attach_wait_snk_debounced>, fsm::on<event::vbus_present>,
                    fsm::to<ATTACH>>,
    fsm::internal_transition<fsm::from<state::attach_wait_snk_debounced>,
                             fsm::on<event::vbus_removed>>,
    fsm::transition<fsm::from<state::attach_wait_snk_debounced>, fsm::on<event::cc_changed>,
                    fsm::to<state::attach_wait_snk>>,
    // sink detach detection is VBUS-based
    fsm::transition<fsm::from<state::attached_snk>, fsm::on<event::vbus_removed>,
                    fsm::to<UNATTACHED>>,
    fsm::internal_transition<fsm::from<state::attached_snk>, fsm::on<event::cc_changed>>,
    fsm::internal_transition<fsm::from<state::attached_snk>, fsm::on<event::vbus_present>>>;

// The spec timer range of every timed state of the sink flow; the DRP
// concatenates this map with its own, like the flows themselves
using sink_timer_ranges = mtl::typelist<
    fsm::timed_by<state::attach_wait_snk, spec::t_cc_debounce>>;

// Separate entry: the DRP shares the state, standalone source tables
// do not - the bidirectional map check rejects entries for absent states
using error_recovery_timer_range = mtl::typelist<
    fsm::timed_by<state::error_recovery, spec::t_error_recovery>>;

using hard_reset_timer_ranges = mtl::typelist<
    fsm::timed_by<state::hard_reset_snk, spec::t_hard_reset_window>,
    fsm::timed_by<state::hard_reset_recover_snk, spec::t_hard_reset_window>>;

// The hard-reset window, shared with the DRP: attach held while VBUS
// legitimately cycles through vSafe0V; a window that never completes
// falls back to the table's unattached anchor
template<typename UNATTACHED>
using hard_reset_flow = mtl::typelist<
    fsm::transition<fsm::from<state::attached_snk>, fsm::on<event::hard_reset>,
                    fsm::to<state::hard_reset_snk>>,
    fsm::transition<fsm::from<state::hard_reset_snk>, fsm::on<event::vbus_removed>,
                    fsm::to<state::hard_reset_recover_snk>>,
    fsm::internal_transition<fsm::from<state::hard_reset_snk>, fsm::on<event::vbus_present>>,
    fsm::internal_transition<fsm::from<state::hard_reset_snk>, fsm::on<event::cc_changed>>,
    fsm::transition<fsm::from<state::hard_reset_snk>, fsm::on<fsm::timeout>,
                    fsm::to<UNATTACHED>>,
    fsm::transition<fsm::from<state::hard_reset_recover_snk>, fsm::on<event::vbus_present>,
                    fsm::to<state::attached_snk>>,
    fsm::internal_transition<fsm::from<state::hard_reset_recover_snk>,
                             fsm::on<event::vbus_removed>>,
    fsm::internal_transition<fsm::from<state::hard_reset_recover_snk>,
                             fsm::on<event::cc_changed>>,
    fsm::transition<fsm::from<state::hard_reset_recover_snk>, fsm::on<fsm::timeout>,
                    fsm::to<UNATTACHED>>>;

// ErrorRecovery is anchored per table: open terminations, then back
// to that table's unattached resting state
template<typename UNATTACHED>
using error_recovery_flow = mtl::typelist<
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::error_recovery>,
                    fsm::to<state::error_recovery>>,
    fsm::transition<fsm::from<state::error_recovery>, fsm::on<fsm::timeout>,
                    fsm::to<UNATTACHED>>>;

// A named struct, not an alias: the short name replaces the fully
// spelled table type in every mangled symbol
struct sink_table
    : mtl::rebind_t<
          mtl::linearize_t<mtl::typelist<
              fsm::initial<state::disabled_snk>,
              fsm::transition<fsm::from<state::disabled_snk>, fsm::on<event::started>,
                              fsm::to<state::unattached_snk>>,
              sink_attach_flow<state::unattached_snk, state::attached_snk>,
              error_recovery_flow<state::unattached_snk>,
              hard_reset_flow<state::unattached_snk>>>,
          fsm::transition_table> {};
// timeout bounds and reachability checked in test/compliance.cpp

// Applies each state's hw annotation (suppressed while unchanged) and
// the attached state's plug orientation
template<concepts::tcpc TCPC>
struct hw_driver : fsm::observing<hw_driver<TCPC>> {
    explicit hw_driver(TCPC& tcpc_ref) : tcpc(tcpc_ref) {}

    // A state the dispatch would silently skip is a table bug: the
    // previous state's terminations or power paths would stay applied.
    // Checked per element: every state must carry both
    template<fsm::concepts::transition_table TABLE>
    static constexpr void validate()
    {
        static_assert(fsm::all_states_carry_v<TABLE, cc_termination>,
                      "hw_driver: every state must annotate its CC termination");
        static_assert(fsm::all_states_carry_v<TABLE, vbus_power>,
                      "hw_driver: every state must annotate its VBUS power paths");
    }

    void notifyEntry(cc_termination termination)
    {
        tcpc.setCc(termination.pull, rp_value::usb_default);
    }
    void notifyEntry(vbus_power power) { tcpc.sinkVbus(power.path == vbus_path::sink); }
    void notifyEntry(polarity resolved) { tcpc.setPlugOrientation(resolved.orientation); }

    TCPC& tcpc;
};

} // namespace tc

template<concepts::tcpc TCPC, concepts::vbus VBUS, fsm::concepts::timer TIMER,
         typename... OBSERVERs>
class TypeCSink : public tc::port_frontend<TypeCSink<TCPC, VBUS, TIMER, OBSERVERs...>, TCPC,
                                           VBUS> {
public:
    // Construction rests in Disabled with open terminations; the port
    // goes live on start(). The observers are injected into the
    // machine after the built-in ones (timer, hw driver, vbus watcher);
    // attach results are observed on the attached state's
    // attachedInfo(), and an observer providing onPdAlert(alert_status)
    // receives the alert bits this layer does not consume - the hook
    // for the PD layers above
    TypeCSink(TCPC& tcpc, VBUS& vbus, TIMER& timer, OBSERVERs&... observers)
        : tcpc_(tcpc), hw_(tcpc), vbus_(vbus), timer_(timer), timed_(timer_),
          observers_(observers...), sm_(timed_, hw_, vbus_, observers...)
    {
    }

    // The go-live moment, provided by the shared frontend
    void start() { this->startPort(); }

    // PD-directed Type-C Error Recovery: both terminations removed for
    // tErrorRecovery, then resolution restarts. Call from the stack's
    // serialized context
    bool errorRecovery() { return sm_.process(tc::event::error_recovery{}); }

    // PD-directed hard-reset window: the attach is held while VBUS
    // legitimately cycles through vSafe0V and back
    bool hardResetWindow() { return sm_.process(tc::event::hard_reset{}); }

private:
    friend tc::port_frontend<TypeCSink, TCPC, VBUS>;

    TCPC& tcpc_;
    tc::hw_driver<TCPC> hw_;
    tc::vbus_watcher<VBUS> vbus_;
    fsm::QueuedTimer<TIMER> timer_;
    fsm::timed<fsm::QueuedTimer<TIMER>&> timed_;
    std::tuple<OBSERVERs&...> observers_;
    fsm::QueuedMachine<tc::sink_table, 4, fsm::inline_work, fsm::no_lock,
                       fsm::timed<fsm::QueuedTimer<TIMER>&>, tc::hw_driver<TCPC>,
                       tc::vbus_watcher<VBUS>, OBSERVERs...>
        sm_;
};

} // namespace usbc
