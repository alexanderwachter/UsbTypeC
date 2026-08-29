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
 * Integration: initialize the tcpc and vbus drivers and register the
 * vbus callback before constructing the machine (construction applies
 * the initial state's terminations and starts monitoring). Feed
 * cc_status_changed alerts through cc_alert() and vbus callback
 * reports through vbus_event(); both run in the stack's context, and
 * the TIMER policy is serialized with them by the integrator (mtl
 * timer contract). Client callbacks (on_attached/on_detached) may
 * originate from the timer context on debounce-timeout paths.
 *
 * Not covered yet: source and DRP roles, Try.SNK, debug and audio
 * accessories (both-Rp results detach), Rp change notification while
 * attached, and VCONN (a sink without cable communication needs none).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Tcpc.hpp>
#include <usbc/Vbus.hpp>

#include <mtl/StateMachine.hpp>

#include <chrono>
#include <concepts>

namespace usbc {

namespace concepts {

template<typename T>
concept tc_sink_client = requires(T client, plug_orientation orientation, rp_value advertisement) {
    client.on_attached(orientation, advertisement);
    client.on_detached();
};

} // namespace concepts

namespace tc {

inline constexpr auto t_cc_debounce = std::chrono::milliseconds{150}; // 100 ms - 200 ms

constexpr bool is_rp(cc_state state)
{
    return state == cc_state::snk_default || state == cc_state::snk_power_1a5 ||
           state == cc_state::snk_power_3a0;
}

constexpr bool single_rp(cc_status status)
{
    return is_rp(status.cc1) != is_rp(status.cc2);
}

constexpr plug_orientation orientation_of(cc_status status)
{
    return is_rp(status.cc1) ? plug_orientation::cc1 : plug_orientation::cc2;
}

constexpr rp_value advertisement_of(cc_status status)
{
    switch (is_rp(status.cc1) ? status.cc1 : status.cc2) {
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

struct cc_changed {
    cc_status cc;
};
struct vbus_present {};
struct vbus_removed {};

} // namespace event

namespace state {

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
          orientation_(orientation_of(ctx.cc)),
          advertisement_(advertisement_of(ctx.cc))
    {
    }

    plug_orientation orientation() const { return orientation_; }
    attach_info attached_info() const { return {orientation_, advertisement_}; }

private:
    plug_orientation orientation_;
    rp_value advertisement_;
};

} // namespace state

struct attach_conditions_met {
    static bool check(state::attach_wait_snk const& state)
    {
        return single_rp(state.context.cc) && state.context.vbus_present;
    }
};

struct stable_rp {
    static bool check(state::attach_wait_snk const& state) { return single_rp(state.context.cc); }
};

using sink_table = fsm::transition_table<
    fsm::initial<state::unattached_snk>,
    fsm::transition<fsm::from<state::unattached_snk>, fsm::on<event::cc_changed>,
                    fsm::to<state::attach_wait_snk>>,
    fsm::internal_transition<fsm::from<state::unattached_snk>, fsm::on<event::vbus_present>>,
    fsm::internal_transition<fsm::from<state::unattached_snk>, fsm::on<event::vbus_removed>>,
    // a CC change during the debounce restarts it
    fsm::transition<fsm::from<state::attach_wait_snk>, fsm::on<event::cc_changed>,
                    fsm::to<state::attach_wait_snk>>,
    fsm::internal_transition<fsm::from<state::attach_wait_snk>, fsm::on<event::vbus_present>>,
    fsm::internal_transition<fsm::from<state::attach_wait_snk>, fsm::on<event::vbus_removed>>,
    // debounce complete: attach, keep waiting for VBUS, or detach
    fsm::transition<fsm::from<state::attach_wait_snk>, fsm::on<fsm::timeout>,
                    fsm::to<state::attached_snk>, fsm::guard<attach_conditions_met>>,
    fsm::transition<fsm::from<state::attach_wait_snk>, fsm::on<fsm::timeout>,
                    fsm::to<state::attach_wait_snk_debounced>, fsm::guard<stable_rp>>,
    fsm::transition<fsm::from<state::attach_wait_snk>, fsm::on<fsm::timeout>,
                    fsm::to<state::unattached_snk>>,
    fsm::transition<fsm::from<state::attach_wait_snk_debounced>, fsm::on<event::vbus_present>,
                    fsm::to<state::attached_snk>>,
    fsm::internal_transition<fsm::from<state::attach_wait_snk_debounced>,
                             fsm::on<event::vbus_removed>>,
    fsm::transition<fsm::from<state::attach_wait_snk_debounced>, fsm::on<event::cc_changed>,
                    fsm::to<state::attach_wait_snk>>,
    // sink detach detection is VBUS-based
    fsm::transition<fsm::from<state::attached_snk>, fsm::on<event::vbus_removed>,
                    fsm::to<state::unattached_snk>>,
    fsm::internal_transition<fsm::from<state::attached_snk>, fsm::on<event::cc_changed>>,
    fsm::internal_transition<fsm::from<state::attached_snk>, fsm::on<event::vbus_present>>>;

// Applies each state's hw annotation (suppressed while unchanged) and
// the attached state's plug orientation
template<concepts::tcpc TCPC>
struct hw_driver : fsm::observing<hw_driver<TCPC>> {
    explicit hw_driver(TCPC& tcpc_ref) : tcpc(tcpc_ref) {}

    template<typename STATE>
    static constexpr auto observe_static() -> decltype(STATE::hw)
    {
        return STATE::hw;
    }
    void notify_entry(hw_config const& config)
    {
        tcpc.set_cc(config.pull, rp_value::usb_default);
        tcpc.sink_vbus(config.sink);
    }

    static constexpr auto observe_nonstatic(auto const& state) -> decltype((state.orientation()))
    {
        return state.orientation();
    }
    void notify_entry(plug_orientation orientation) { tcpc.set_plug_orientation(orientation); }

    TCPC& tcpc;
};

// Arms the vbus driver with each state's watched level; the class maps
// the callback's meaning through the level it armed last
template<concepts::vbus VBUS>
struct vbus_watcher : fsm::observing<vbus_watcher<VBUS>> {
    explicit vbus_watcher(VBUS& vbus_ref) : vbus(vbus_ref) {}

    template<typename STATE>
    static constexpr auto observe_static() -> decltype(STATE::watch)
    {
        return STATE::watch;
    }
    void notify_entry(vbus_level level)
    {
        monitored = level;
        vbus.monitor(level);
    }

    VBUS& vbus;
    vbus_level monitored = vbus_level::safe5v;
};

} // namespace tc

template<concepts::tcpc TCPC, concepts::vbus VBUS, fsm::concepts::timer TIMER,
         concepts::tc_sink_client CLIENT>
class type_c_sink {
public:
    type_c_sink(TCPC& tcpc, VBUS& vbus, CLIENT& client)
        : tcpc_(tcpc), hw_(tcpc), vbus_(vbus), client_(client)
    {
    }

    // The TCPC reported a CC status change
    void cc_alert()
    {
        if (auto const cc = tcpc_.read_cc_status()) {
            sm_.process(tc::event::cc_changed{*cc});
        }
    }

    // The vbus driver reported its monitored condition
    void vbus_event(bool met)
    {
        bool const present = vbus_.monitored == vbus_level::sink_disconnect ? !met : met;
        if (present) {
            sm_.process(tc::event::vbus_present{});
        } else {
            sm_.process(tc::event::vbus_removed{});
        }
    }

    // The timer policy instance, for platform integration (e.g. polling)
    TIMER& timer() { return timed_.timer; }

private:
    // Tells the client about attach results, from the live attached state
    struct attach_reporter : fsm::observing<attach_reporter> {
        explicit attach_reporter(CLIENT& client_ref) : client(client_ref) {}

        static constexpr auto observe_nonstatic(auto const& state)
            -> decltype((state.attached_info()))
        {
            return state.attached_info();
        }
        void notify_entry(tc::attach_info info)
        {
            client.on_attached(info.orientation, info.advertisement);
        }
        void notify_exit(tc::attach_info) { client.on_detached(); }

        CLIENT& client;
    };

    TCPC& tcpc_;
    tc::hw_driver<TCPC> hw_;
    tc::vbus_watcher<VBUS> vbus_;
    CLIENT& client_;
    attach_reporter reporter_{client_};
    fsm::timed<TIMER> timed_{};
    fsm::state_machine<tc::sink_table, fsm::timed<TIMER>, tc::hw_driver<TCPC>,
                       tc::vbus_watcher<VBUS>, attach_reporter>
        sm_{timed_, hw_, vbus_, reporter_};
};

} // namespace usbc
