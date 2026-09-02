/*
 * USB Type-C source connection layer: Unattached.SRC, AttachWait.SRC,
 * Attached.SRC and UnattachedWait.SRC per the USB Type-C Cable and
 * Connector Specification. The port presents Rp with the configured
 * advertisement, debounces a detected Rd for tCCDebounce, requires
 * VBUS at vSafe0V before applying power (attach_wait_src_debounced
 * waits for it, like the sink's sub-state waits for vSafe5V), sources
 * VBUS while attached, and discharges to vSafe0V in UnattachedWait.SRC
 * after a detach - skipped when VBUS already sits at vSafe0V, since
 * the vbus driver reports changes only. Detach detection (Rd removed)
 * decides on the CC event payload through a (state, event) guard.
 *
 * Integration mirrors TypeCSink: initialized drivers plus a
 * caller-owned timer policy instance. Construction rests in the spec's
 * Disabled state (open terminations, nothing monitored); start() is
 * the go-live moment - it fires the started event, whose transition
 * presents Rp and arms the vSafe0V monitor, then registers the
 * callbacks, re-arms the monitor for its initial report, and seeds the
 * CC state when a sink is already present. Client callbacks may
 * originate from the timer context on debounce-timeout paths.
 *
 * Not covered yet: DRP, Try.SRC, debug accessories (both-Rd results
 * detach), VCONN and Ra cable handling, and debounced source-side
 * detach (tPDDebounce).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Tcpc.hpp>
#include <usbc/TypeC.hpp>
#include <usbc/Vbus.hpp>

#include <mtl/StateMachine.hpp>

#include <concepts>

namespace usbc {

namespace concepts {

// A source advertises its own Rp: attach reports the orientation only
template<typename T>
concept tc_source_client = requires(T client, plug_orientation orientation) {
    client.onAttached(orientation);
    client.onDetached();
};

} // namespace concepts

namespace tc {

constexpr bool isRd(cc_state state)
{
    return state == cc_state::src_rd;
}

constexpr bool singleRd(cc_status status)
{
    return isRd(status.cc1) != isRd(status.cc2);
}

constexpr plug_orientation srcOrientationOf(cc_status status)
{
    return isRd(status.cc1) ? plug_orientation::cc1 : plug_orientation::cc2;
}

// The CC pull (Rp with the port's configured advertisement, or open
// while Disabled), power path, and discharge a state requires
struct src_hw_config {
    cc_pull pull;
    bool source;
    bool discharge;
    constexpr bool operator==(src_hw_config const&) const = default;
};

struct src_context {
    cc_status cc{cc_state::src_open, cc_state::src_open};
    bool vbus_safe0v = false;
};

namespace event {

struct vbus_reached_safe0v {};
struct vbus_left_safe0v {};

} // namespace event

namespace state {

// The spec's Disabled state: the port is not operating, terminations
// removed, nothing monitored. start() fires the started event
struct disabled_src {
    static constexpr src_hw_config hw{.pull = cc_pull::open, .source = false,
                                      .discharge = false};
};

// Common context plus the internal-transition handlers keeping it
// current without disturbing a running debounce
struct source_state {
    explicit source_state(src_context& ctx) : context(ctx) {}

    void handle(event::cc_changed const& event) { context.cc = event.cc; }
    void handle(event::vbus_reached_safe0v const&) { context.vbus_safe0v = true; }
    void handle(event::vbus_left_safe0v const&) { context.vbus_safe0v = false; }

    src_context& context;
};

struct unattached_src : source_state {
    static constexpr src_hw_config hw{.pull = cc_pull::rp, .source = false, .discharge = false};
    static constexpr vbus_level watch = vbus_level::safe0v;

    // entering on the discharge-complete event records what it means -
    // a transition does not run the internal handlers
    unattached_src(event::vbus_reached_safe0v const&, src_context& ctx) : source_state(ctx)
    {
        context.vbus_safe0v = true;
    }
    using source_state::source_state;
};

struct attach_wait_src : source_state {
    static constexpr src_hw_config hw{.pull = cc_pull::rp, .source = false, .discharge = false};
    static constexpr vbus_level watch = vbus_level::safe0v;
    static constexpr auto timeout     = t_cc_debounce; // CCDebounceTimer

    attach_wait_src(event::cc_changed const& event, src_context& ctx) : source_state(ctx)
    {
        context.cc = event.cc;
    }
    using source_state::source_state;
};

// AttachWait.SRC with a stable single Rd, waiting for VBUS at vSafe0V
struct attach_wait_src_debounced : source_state {
    static constexpr src_hw_config hw{.pull = cc_pull::rp, .source = false, .discharge = false};
    static constexpr vbus_level watch = vbus_level::safe0v;

    using source_state::source_state;
};

struct attached_src : source_state {
    static constexpr src_hw_config hw{.pull = cc_pull::rp, .source = true, .discharge = false};
    static constexpr vbus_level watch = vbus_level::safe0v;

    // entered from the debounced wait on the vSafe0V event
    attached_src(event::vbus_reached_safe0v const&, src_context& ctx) : attached_src(ctx)
    {
        context.vbus_safe0v = true;
    }
    explicit attached_src(src_context& ctx)
        : source_state(ctx), orientation_(srcOrientationOf(ctx.cc))
    {
    }

    plug_orientation orientation() const { return orientation_; }
    plug_orientation attachedInfo() const { return orientation_; }

private:
    plug_orientation orientation_;
};

// Discharges VBUS to vSafe0V before presenting Rp for a new attach
struct unattached_wait_src : source_state {
    static constexpr src_hw_config hw{.pull = cc_pull::rp, .source = false, .discharge = true};
    static constexpr vbus_level watch = vbus_level::safe0v;

    unattached_wait_src(event::cc_changed const& event, src_context& ctx) : source_state(ctx)
    {
        context.cc = event.cc;
    }
    using source_state::source_state;
};

} // namespace state

struct src_attach_conditions_met {
    static bool check(state::attach_wait_src const& state)
    {
        return singleRd(state.context.cc) && state.context.vbus_safe0v;
    }
};

struct src_stable_rd {
    static bool check(state::attach_wait_src const& state)
    {
        return singleRd(state.context.cc);
    }
};

// Decides on the event's CC payload - the state's context still holds
// the pre-event status when the guard runs
struct rd_removed {
    static bool check(state::attached_src const&, event::cc_changed const& event)
    {
        return !singleRd(event.cc);
    }
};

// Detach with VBUS already at vSafe0V: nothing to discharge, so
// UnattachedWait.SRC is skipped (its exit event would never come - the
// vbus driver reports changes only)
struct rd_removed_at_safe0v {
    static bool check(state::attached_src const& state, event::cc_changed const& event)
    {
        return !singleRd(event.cc) && state.context.vbus_safe0v;
    }
};

using source_table = fsm::transition_table<
    fsm::initial<state::disabled_src>,
    fsm::transition<fsm::from<state::disabled_src>, fsm::on<event::started>,
                    fsm::to<state::unattached_src>>,
    fsm::transition<fsm::from<state::unattached_src>, fsm::on<event::cc_changed>,
                    fsm::to<state::attach_wait_src>>,
    fsm::internal_transition<fsm::from<state::unattached_src>,
                             fsm::on<event::vbus_reached_safe0v>>,
    fsm::internal_transition<fsm::from<state::unattached_src>,
                             fsm::on<event::vbus_left_safe0v>>,
    // a CC change during the debounce restarts it
    fsm::transition<fsm::from<state::attach_wait_src>, fsm::on<event::cc_changed>,
                    fsm::to<state::attach_wait_src>>,
    fsm::internal_transition<fsm::from<state::attach_wait_src>,
                             fsm::on<event::vbus_reached_safe0v>>,
    fsm::internal_transition<fsm::from<state::attach_wait_src>,
                             fsm::on<event::vbus_left_safe0v>>,
    // debounce complete: attach, wait for vSafe0V, or back to unattached
    fsm::transition<fsm::from<state::attach_wait_src>, fsm::on<fsm::timeout>,
                    fsm::to<state::attached_src>, fsm::guard<src_attach_conditions_met>>,
    fsm::transition<fsm::from<state::attach_wait_src>, fsm::on<fsm::timeout>,
                    fsm::to<state::attach_wait_src_debounced>, fsm::guard<src_stable_rd>>,
    fsm::transition<fsm::from<state::attach_wait_src>, fsm::on<fsm::timeout>,
                    fsm::to<state::unattached_src>>,
    fsm::transition<fsm::from<state::attach_wait_src_debounced>,
                    fsm::on<event::vbus_reached_safe0v>, fsm::to<state::attached_src>>,
    fsm::internal_transition<fsm::from<state::attach_wait_src_debounced>,
                             fsm::on<event::vbus_left_safe0v>>,
    fsm::transition<fsm::from<state::attach_wait_src_debounced>, fsm::on<event::cc_changed>,
                    fsm::to<state::attach_wait_src>>,
    // source detach detection is CC-based: Rd removed
    fsm::transition<fsm::from<state::attached_src>, fsm::on<event::cc_changed>,
                    fsm::to<state::unattached_src>, fsm::guard<rd_removed_at_safe0v>>,
    fsm::transition<fsm::from<state::attached_src>, fsm::on<event::cc_changed>,
                    fsm::to<state::unattached_wait_src>, fsm::guard<rd_removed>>,
    fsm::internal_transition<fsm::from<state::attached_src>, fsm::on<event::cc_changed>>,
    fsm::internal_transition<fsm::from<state::attached_src>,
                             fsm::on<event::vbus_reached_safe0v>>,
    fsm::internal_transition<fsm::from<state::attached_src>, fsm::on<event::vbus_left_safe0v>>,
    fsm::transition<fsm::from<state::unattached_wait_src>, fsm::on<event::vbus_reached_safe0v>,
                    fsm::to<state::unattached_src>>,
    fsm::internal_transition<fsm::from<state::unattached_wait_src>,
                             fsm::on<event::vbus_left_safe0v>>,
    fsm::internal_transition<fsm::from<state::unattached_wait_src>,
                             fsm::on<event::cc_changed>>>;

// Applies each state's src_hw annotation (suppressed while unchanged)
// with the port's configured Rp advertisement, and the attached
// state's plug orientation. Discharge belongs to the vbus driver
template<concepts::tcpc TCPC, concepts::vbus VBUS>
struct src_hw_driver : fsm::observing<src_hw_driver<TCPC, VBUS>> {
    src_hw_driver(TCPC& tcpc_ref, VBUS& vbus_ref, rp_value advertisement)
        : tcpc(tcpc_ref), vbus(vbus_ref), rp(advertisement)
    {
    }

    template<typename STATE>
    static constexpr auto observe_static() -> decltype(STATE::hw)
    {
        return STATE::hw;
    }
    void notifyEntry(src_hw_config const& config)
    {
        tcpc.setCc(config.pull, rp);
        tcpc.sourceVbus(config.source);
        vbus.discharge(config.discharge);
    }

    static constexpr auto observe_nonstatic(auto const& state) -> decltype((state.orientation()))
    {
        return state.orientation();
    }
    void notifyEntry(plug_orientation orientation) { tcpc.setPlugOrientation(orientation); }

    TCPC& tcpc;
    VBUS& vbus;
    rp_value rp;
};

} // namespace tc

template<concepts::tcpc TCPC, concepts::vbus VBUS, fsm::concepts::timer TIMER,
         concepts::tc_source_client CLIENT>
class TypeCSource {
public:
    // Construction rests in Disabled with open terminations; the port
    // goes live on start(). The advertisement is the Rp the port
    // presents (and must honor)
    TypeCSource(TCPC& tcpc, VBUS& vbus, TIMER& timer, CLIENT& client,
                rp_value advertisement = rp_value::usb_default)
        : tcpc_(tcpc), hw_(tcpc, vbus, advertisement), vbus_(vbus), client_(client), timed_(timer)
    {
    }

    // Leaves Disabled: Rp and monitoring apply through the machine,
    // the callbacks register, and a present sink is seeded from the CC
    // status. A second start() finds no started transition and does
    // nothing
    void start()
    {
        if (!sm_.process(tc::event::started{})) {
            return;
        }
        vbus_.vbus.setCallback(
            [](void* self, bool met) { static_cast<TypeCSource*>(self)->vbusEvent(met); }, this);
        tcpc_.setAlertHandler([](void* self) { static_cast<TypeCSource*>(self)->alert(); }, this);
        vbus_.vbus.monitor(vbus_level::safe0v); // deliver the initial condition
        seedCcState();
    }

private:
    // Drains the TCPC's pending alerts; a client providing
    // onPdAlert(alert_status) receives the bits this layer does not
    // consume - the hook for the PD layers above
    void alert()
    {
        if (auto const alerts = tcpc_.readAlert()) {
            if (any(*alerts & alert_status::cc_status_changed)) {
                ccAlert();
            }
            if constexpr (requires { client_.onPdAlert(*alerts); }) {
                auto const residual = *alerts & ~alert_status::cc_status_changed;
                if (any(residual)) {
                    client_.onPdAlert(residual);
                }
            }
        }
    }

    void ccAlert()
    {
        if (auto const cc = tcpc_.readCcStatus()) {
            sm_.process(tc::event::cc_changed{*cc});
        }
    }

    // The vbus driver reported the vSafe0V condition the layer monitors
    void vbusEvent(bool met)
    {
        if (met) {
            sm_.process(tc::event::vbus_reached_safe0v{});
        } else {
            sm_.process(tc::event::vbus_left_safe0v{});
        }
    }

    // A sink plugged in before construction has no alert to announce it
    void seedCcState()
    {
        auto const cc = tcpc_.readCcStatus();
        if (cc && (tc::isRd(cc->cc1) || tc::isRd(cc->cc2))) {
            sm_.process(tc::event::cc_changed{*cc});
        }
    }

    // Tells the client about attach results, from the live attached state
    struct attach_reporter : fsm::observing<attach_reporter> {
        explicit attach_reporter(CLIENT& client_ref) : client(client_ref) {}

        static constexpr auto observe_nonstatic(auto const& state)
            -> decltype((state.attachedInfo()))
        {
            return state.attachedInfo();
        }
        void notifyEntry(plug_orientation orientation) { client.onAttached(orientation); }
        void notifyExit(plug_orientation) { client.onDetached(); }

        CLIENT& client;
    };

    TCPC& tcpc_;
    tc::src_hw_driver<TCPC, VBUS> hw_;
    tc::vbus_watcher<VBUS> vbus_;
    CLIENT& client_;
    attach_reporter reporter_{client_};
    fsm::timed<TIMER&> timed_;
    fsm::state_machine<tc::source_table, fsm::timed<TIMER&>, tc::src_hw_driver<TCPC, VBUS>,
                       tc::vbus_watcher<VBUS>, attach_reporter>
        sm_{timed_, hw_, vbus_, reporter_};
};

} // namespace usbc
