/*
 * Common ground of the USB Type-C connection layers: the CC debounce
 * time, the CC change event, and the vbus_watcher observer arming the
 * vbus driver with each state's watched level. The sink and source
 * layers live in TypeCSink.hpp and TypeCSource.hpp.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Spec.hpp>
#include <usbc/Tcpc.hpp>
#include <usbc/Vbus.hpp>

#include <mtl/StateMachine.hpp>

#include <chrono>
#include <tuple>

namespace usbc {

namespace tc {

inline constexpr auto t_cc_debounce   = std::chrono::milliseconds{150}; // tCCDebounce
inline constexpr auto t_error_recovery = std::chrono::milliseconds{50}; // tErrorRecovery
inline constexpr auto t_hard_reset_window = std::chrono::milliseconds{6000}; // > tNoResponse

// What one CC line's voltage says about the partner's termination:
// an Rp seen while presenting Rd, an Rd seen while presenting Rp
constexpr bool isRp(cc_state state)
{
    return state == cc_state::snk_default || state == cc_state::snk_power_1a5 ||
           state == cc_state::snk_power_3a0;
}

constexpr bool isRd(cc_state state)
{
    return state == cc_state::src_rd;
}

namespace event {

struct cc_changed {
    cc_status cc;
};
struct started {}; // start(): leave Disabled, apply the terminations

// The vbus driver's report, mapped through the level the watcher
// armed: presence for the sink-role levels (vSafe5V, vSinkDisconnect),
// the discharge condition for vSafe0V
struct vbus_present {};
struct vbus_removed {};
struct vbus_reached_safe0v {};
struct vbus_left_safe0v {};

// Role swaps directed by the layer above (USB PD PR_Swap/DR_Swap),
// DRP only: the pair stays attached and the plug orientation carries
// over in the shared context. A power swap runs in two phases - the
// swap_to_* events enter a standby with the power paths off and
// detach detection suspended (VBUS is legitimately absent while the
// roles change hands), swap_complete lands in the new attached state
// on the partner's PS_RDY, and swap_abort restores the departing
// role. A data role swap changes no terminations at all - it only
// flips the context's data role
struct swap_to_source {};
struct swap_to_sink {};
struct swap_complete {};
struct swap_abort {};
struct swap_data_role {};
struct error_recovery {}; // PD-directed: remove both terminations
struct hard_reset {};     // PD-directed: open the hard-reset window

} // namespace event

// Machine-owned context shared by every connection-layer state, across
// both roles: the latest CC status (interpreted through the presented
// pull), the vbus conditions, and the resolved plug orientation and
// data role - kept here so they survive a role swap (a power swap
// leaves the data role alone, per the PD spec)
struct port_context {
    cc_status cc{cc_state::snk_open, cc_state::snk_open};
    bool vbus_present = false;
    bool vbus_safe0v  = false;
    plug_orientation orientation = plug_orientation::cc1;
    data_role data               = data_role::ufp;
    // set while the hard-reset window holds the attach: re-entering
    // Attached.SNK resumes the connection (plug and data role kept)
    // instead of resolving a fresh attach
    bool resuming = false;
};

// The one place tying a VBUS level to the events its reports become:
// the sink-role levels report presence, vSafe0V reports the discharge
// condition - and on the below-threshold levels a met report means the
// bus is gone. The ports' report mapping and the verification below
// both consume this
enum class vbus_family : std::uint8_t { presence, discharge };

constexpr vbus_family familyOf(vbus_level level)
{
    return level == vbus_level::safe0v ? vbus_family::discharge : vbus_family::presence;
}

constexpr bool metMeansPresent(vbus_level level)
{
    return level == vbus_level::safe5v;
}

// A watching state must consume the event family its armed level
// makes the driver deliver - a missing transition would silently drop
// a report (a lost detach at worst)
template<typename TABLE>
struct watch_family_handled {
    template<typename STATE, bool WATCHING = requires { STATE::watch; }>
    struct pred
        : std::bool_constant<
              familyOf(STATE::watch) == vbus_family::discharge
                  ? (fsm::handles_event_v<TABLE, STATE, event::vbus_reached_safe0v> &&
                     fsm::handles_event_v<TABLE, STATE, event::vbus_left_safe0v>)
                  : (fsm::handles_event_v<TABLE, STATE, event::vbus_present> &&
                     fsm::handles_event_v<TABLE, STATE, event::vbus_removed>)> {};

    // an unwatched state gets no reports and implies nothing
    template<typename STATE>
    struct pred<STATE, false> : std::true_type {};
};

template<typename TABLE>
struct watch_events_consistent
    : std::bool_constant<mtl::all_of_v<typename TABLE::states,
                                       watch_family_handled<TABLE>::template pred>> {};

template<typename TABLE>
inline constexpr bool watch_events_consistent_v = watch_events_consistent<TABLE>::value;

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
    void notifyEntry(vbus_level level)
    {
        monitored = level;
        vbus.monitor(level);
    }

    // Only a state presenting no terminations (Disabled, ErrorRecovery)
    // may go unwatched: nothing can attach while the pull is open
    template<typename STATE, bool HAS_HW = requires { STATE::hw; }>
    struct idle_without_watch : std::bool_constant<STATE::hw.pull == cc_pull::open> {};

    template<typename STATE>
    struct idle_without_watch<STATE, false> : std::false_type {};

    template<typename STATE>
    struct watched_or_idle : std::bool_constant<fsm::is_notified_of_v<vbus_watcher, STATE> ||
                                                idle_without_watch<STATE>::value> {};

    template<fsm::concepts::transition_table TABLE>
    static constexpr void validate()
    {
        static_assert(mtl::all_of_v<typename TABLE::states, watched_or_idle>,
                      "vbus_watcher: a state presenting terminations must watch a VBUS level");
        static_assert(watch_events_consistent_v<TABLE>,
                      "vbus_watcher: a watching state must handle its level's event family");
    }

    VBUS& vbus;
    vbus_level monitored = vbus_level::safe5v;
};

// The driver-facing frontend every connection layer shares (CRTP):
// start() wiring, the TCPC alert pump with residual forwarding to the
// observers providing onPdAlert, the vbus report mapped through the
// armed level's family, and the seeding of a partner already present.
// The derived port provides tcpc_, vbus_ (the watcher), observers_,
// and sm_, and befriends this base
template<typename DERIVED, concepts::tcpc TCPC, concepts::vbus VBUS>
class port_frontend {
protected:
    // Leaves Disabled through the started event; when it fires, the
    // callbacks register, the monitor re-arms for its initial report,
    // and a present partner is seeded from the CC status. A second
    // start() finds no started transition and does nothing
    void startPort()
    {
        auto& self = derived();
        if (!self.sm_.process(event::started{})) {
            return;
        }
        self.vbus_.vbus.setCallback(
            [](void* frontend, bool met) {
                static_cast<port_frontend*>(frontend)->vbusEvent(met);
            },
            this);
        self.tcpc_.setAlertHandler(
            [](void* frontend) { static_cast<port_frontend*>(frontend)->alert(); }, this);
        self.vbus_.vbus.monitor(self.vbus_.monitored); // deliver the initial condition
        seedCcState();
    }

private:
    DERIVED& derived() { return static_cast<DERIVED&>(*this); }

    // Drains the TCPC's pending alerts; the bits this layer does not
    // consume go to the observers providing onPdAlert(alert_status)
    void alert()
    {
        auto& self = derived();
        if (auto const alerts = self.tcpc_.readAlert()) {
            if (any(*alerts & alert_status::cc_status_changed)) {
                if (auto const cc = self.tcpc_.readCcStatus()) {
                    self.sm_.process(event::cc_changed{*cc});
                    std::apply([&](auto&... observer)
                               { (forwardCcStatus(observer, *cc), ...); },
                               self.observers_);
                }
            }
            auto const residual = *alerts & ~alert_status::cc_status_changed;
            if (any(residual)) {
                std::apply([&](auto&... observer) { (forwardPdAlert(observer, residual), ...); },
                           self.observers_);
            }
        }
    }

    static void forwardPdAlert(auto& observer, alert_status alerts)
    {
        if constexpr (requires { observer.onPdAlert(alerts); }) {
            observer.onPdAlert(alerts);
        }
    }

    // The fresh CC status after the machine processed it - the PD
    // layer reads the source's Rp as SinkTxOk/SinkTxNG (PD3 collision
    // avoidance)
    static void forwardCcStatus(auto& observer, cc_status cc)
    {
        if constexpr (requires { observer.onCcStatus(cc); }) {
            observer.onCcStatus(cc);
        }
    }

    // The vbus driver reports the condition the watcher armed; the
    // event follows the level's family - events a table does not
    // handle are dropped by the machine
    void vbusEvent(bool met)
    {
        auto& self = derived();
        if (familyOf(self.vbus_.monitored) == vbus_family::discharge) {
            if (met) {
                self.sm_.process(event::vbus_reached_safe0v{});
            } else {
                self.sm_.process(event::vbus_left_safe0v{});
            }
        } else if (met == metMeansPresent(self.vbus_.monitored)) {
            self.sm_.process(event::vbus_present{});
        } else {
            self.sm_.process(event::vbus_removed{});
        }
    }

    // A partner plugged in before construction has no alert to
    // announce it
    void seedCcState()
    {
        auto& self    = derived();
        auto const cc = self.tcpc_.readCcStatus();
        if (cc && (isRp(cc->cc1) || isRp(cc->cc2) || isRd(cc->cc1) || isRd(cc->cc2))) {
            self.sm_.process(event::cc_changed{*cc});
        }
    }
};

} // namespace tc

} // namespace usbc
