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

constexpr bool isRp(cc_state state)
{
    return state == cc_state::snk_default || state == cc_state::snk_power_1a5 ||
           state == cc_state::snk_power_3a0;
}

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

// The CC terminations and power path a state requires, applied by the
// hw_driver observer with change suppression
struct hw_config {
    cc_pull pull;
    bool sink;
    constexpr bool operator==(hw_config const&) const = default;
};

// Delivered to the client on attach, observed on the attached state
struct attach_info {
    plug_orientation orientation;
    rp_value advertisement;
};

struct port_context {
    cc_status cc{cc_state::snk_open, cc_state::snk_open};
    bool vbus_present = false;
};

namespace event {

struct vbus_present {};
struct vbus_removed {};

} // namespace event

namespace state {

// The spec's Disabled state: the port is not operating, terminations
// removed, nothing monitored. start() fires the started event
struct disabled_snk {
    static constexpr hw_config hw{cc_pull::open, false};
};

// Common context plus the internal-transition handlers keeping it
// current without disturbing a running debounce
struct sink_state {
    explicit sink_state(port_context& ctx) : context(ctx) {}

    void handle(event::cc_changed const& event) { context.cc = event.cc; }
    void handle(event::vbus_present const&) { context.vbus_present = true; }
    void handle(event::vbus_removed const&) { context.vbus_present = false; }

    port_context& context;
};

struct unattached_snk : sink_state {
    static constexpr hw_config hw{cc_pull::rd, false};
    static constexpr vbus_level watch = vbus_level::safe5v;

    using sink_state::sink_state;
};

struct attach_wait_snk : sink_state {
    static constexpr hw_config hw{cc_pull::rd, false};
    static constexpr vbus_level watch = vbus_level::safe5v;
    static constexpr auto timeout     = t_cc_debounce; // CCDebounceTimer

    attach_wait_snk(event::cc_changed const& event, port_context& ctx) : sink_state(ctx)
    {
        context.cc = event.cc;
    }
    using sink_state::sink_state;
};

// AttachWait.SNK with a stable single Rp, waiting for VBUS
struct attach_wait_snk_debounced : sink_state {
    static constexpr hw_config hw{cc_pull::rd, false};
    static constexpr vbus_level watch = vbus_level::safe5v;

    using sink_state::sink_state;
};

struct attached_snk : sink_state {
    static constexpr hw_config hw{cc_pull::rd, true};
    static constexpr vbus_level watch = vbus_level::sink_disconnect;

    explicit attached_snk(port_context& ctx)
        : sink_state(ctx),
          orientation_(orientationOf(ctx.cc)),
          advertisement_(advertisementOf(ctx.cc))
    {
    }
    // entered on a CC event (the DRP's TryWait.SNK attach): the payload
    // must land in the context before the orientation is derived
    attached_snk(event::cc_changed const& event, port_context& ctx)
        : attached_snk((ctx.cc = event.cc, ctx))
    {
    }

    plug_orientation orientation() const { return orientation_; }
    attach_info attachedInfo() const { return {orientation_, advertisement_}; }

private:
    plug_orientation orientation_;
    rp_value advertisement_;
};

} // namespace state

struct attach_conditions_met {
    static bool check(state::attach_wait_snk const& state)
    {
        return singleRp(state.context.cc) && state.context.vbus_present;
    }
};

struct stable_rp {
    static bool check(state::attach_wait_snk const& state) { return singleRp(state.context.cc); }
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
                    fsm::to<ATTACH>, fsm::guard<attach_conditions_met>>,
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

using sink_table = mtl::rebind_t<
    mtl::linearize_t<mtl::typelist<
        fsm::initial<state::disabled_snk>,
        fsm::transition<fsm::from<state::disabled_snk>, fsm::on<event::started>,
                        fsm::to<state::unattached_snk>>,
        sink_attach_flow<state::unattached_snk, state::attached_snk>>>,
    fsm::transition_table>;
static_assert(fsm::timeouts_within_bounds_v<sink_table, sink_timer_ranges>);
static_assert(fsm::all_states_reachable_v<sink_table>);

// Applies each state's hw annotation (suppressed while unchanged) and
// the attached state's plug orientation
template<concepts::tcpc TCPC>
struct hw_driver : fsm::observing<hw_driver<TCPC>> {
    explicit hw_driver(TCPC& tcpc_ref) : tcpc(tcpc_ref) {}

    // A state the dispatch would silently skip is a table bug: the
    // previous state's terminations would stay applied
    template<fsm::concepts::transition_table TABLE>
    static constexpr void validate()
    {
        static_assert(fsm::all_states_notified_v<hw_driver, TABLE>,
                      "hw_driver: every state must annotate an hw config");
    }

    template<typename STATE>
    static constexpr auto observe_static() -> decltype(STATE::hw)
    {
        return STATE::hw;
    }
    void notifyEntry(hw_config const& config)
    {
        tcpc.setCc(config.pull, rp_value::usb_default);
        tcpc.sinkVbus(config.sink);
    }

    static constexpr auto observe_nonstatic(auto const& state) -> decltype((state.orientation()))
    {
        return state.orientation();
    }
    void notifyEntry(plug_orientation orientation) { tcpc.setPlugOrientation(orientation); }

    TCPC& tcpc;
};

} // namespace tc

template<concepts::tcpc TCPC, concepts::vbus VBUS, fsm::concepts::timer TIMER,
         typename... OBSERVERs>
class TypeCSink {
public:
    // Construction rests in Disabled with open terminations; the port
    // goes live on start(). The observers are injected into the
    // machine after the built-in ones (timer, hw driver, vbus watcher);
    // attach results are observed on the attached state's
    // attachedInfo(), and an observer providing onPdAlert(alert_status)
    // receives the alert bits this layer does not consume - the hook
    // for the PD layers above
    TypeCSink(TCPC& tcpc, VBUS& vbus, TIMER& timer, OBSERVERs&... observers)
        : tcpc_(tcpc), hw_(tcpc), vbus_(vbus), timed_(timer), observers_(observers...),
          sm_(timed_, hw_, vbus_, observers...)
    {
    }

    // Leaves Disabled: the terminations and monitoring apply through
    // the machine, the callbacks register, and a present source is
    // seeded from the CC status. A second start() finds no started
    // transition and does nothing
    void start()
    {
        if (!sm_.process(tc::event::started{})) {
            return;
        }
        vbus_.vbus.setCallback(
            [](void* self, bool met) { static_cast<TypeCSink*>(self)->vbusEvent(met); }, this);
        tcpc_.setAlertHandler([](void* self) { static_cast<TypeCSink*>(self)->alert(); }, this);
        vbus_.vbus.monitor(vbus_.monitored); // deliver the initial condition
        seedCcState();
    }

private:
    // Drains the TCPC's pending alerts and dispatches them; invoked
    // through the alert handler registered at construction. The bits
    // this layer does not consume go to the observers providing
    // onPdAlert(alert_status)
    void alert()
    {
        if (auto const alerts = tcpc_.readAlert()) {
            if (any(*alerts & alert_status::cc_status_changed)) {
                ccAlert();
            }
            auto const residual = *alerts & ~alert_status::cc_status_changed;
            if (any(residual)) {
                std::apply([&](auto&... observer) { (forwardPdAlert(observer, residual), ...); },
                           observers_);
            }
        }
    }

    static void forwardPdAlert(auto& observer, alert_status alerts)
    {
        if constexpr (requires { observer.onPdAlert(alerts); }) {
            observer.onPdAlert(alerts);
        }
    }

    // The TCPC reported a CC status change
    void ccAlert()
    {
        if (auto const cc = tcpc_.readCcStatus()) {
            sm_.process(tc::event::cc_changed{*cc});
        }
    }

    // The vbus driver reported its monitored condition
    void vbusEvent(bool met)
    {
        bool const present = vbus_.monitored == vbus_level::sink_disconnect ? !met : met;
        if (present) {
            sm_.process(tc::event::vbus_present{});
        } else {
            sm_.process(tc::event::vbus_removed{});
        }
    }

    // A source plugged in before construction has no alert to announce it
    void seedCcState()
    {
        auto const cc = tcpc_.readCcStatus();
        if (cc && (tc::isRp(cc->cc1) || tc::isRp(cc->cc2))) {
            sm_.process(tc::event::cc_changed{*cc});
        }
    }

    TCPC& tcpc_;
    tc::hw_driver<TCPC> hw_;
    tc::vbus_watcher<VBUS> vbus_;
    fsm::timed<TIMER&> timed_;
    std::tuple<OBSERVERs&...> observers_;
    fsm::state_machine<tc::sink_table, fsm::timed<TIMER&>, tc::hw_driver<TCPC>,
                       tc::vbus_watcher<VBUS>, OBSERVERs...>
        sm_;
};

} // namespace usbc
