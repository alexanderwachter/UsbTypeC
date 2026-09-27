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

// PD3 collision avoidance, seen from the sink: the source's Rp reads
// as SinkTxOk (3.0 A advertisement) or SinkTxNG
constexpr bool sinkTxOk(cc_status status)
{
    return status.cc1 == cc_state::snk_power_3a0 || status.cc2 == cc_state::snk_power_3a0;
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

// The resolved CC polarity of an attach, applied to the TCPC by the
// hw drivers - a strong type so it cannot collide with the raw
// plug_orientation the source-role attach reports to clients
struct polarity {
    plug_orientation orientation;
    constexpr bool operator==(polarity const&) const = default;
};

// The presented CC termination; the hw driver applies it (a source
// role adds its configured Rp advertisement)
struct cc_termination {
    cc_pull pull;
    constexpr bool operator==(cc_termination const&) const = default;
};

// The attached pair's power role, carried by the Attached states only:
// the facades' powerRole() reads it off the machine (a swap standby or
// a hard-reset window carries none - not attached in either role)
struct attached_role {
    power_role role;
    constexpr bool operator==(attached_role const&) const = default;
};

// The switch positions of the VBUS power circuitry, mutually
// exclusive by construction and named for the specification's VBUS
// conditions where one is driven
enum class vbus_path : std::uint8_t {
    open,   // both paths open, the line floats
    sink,   // the sink path draws from VBUS
    safe5v, // the source path applies vSafe5V
    safe0v, // paths open, actively discharging to vSafe0V
};

// The VBUS power annotation - the element that switches power. Every
// connection state carries one, so a role change collapses the old
// role's path, suppressed while unchanged
struct vbus_power {
    vbus_path path = vbus_path::open;
    constexpr bool operator==(vbus_power const&) const = default;
};

// Compile-time probe consuming exactly one annotation type: the
// drivers' validate() checks completeness per element with it - one
// all_states_notified over several accepted types would pass a state
// that carries only one of them
template<typename T>
struct annotation_probe : fsm::observing<annotation_probe<T>> {
    void notifyEntry(T const&) {}
};

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
    // a Try phase deadline expired while a debounce was running: the
    // debounce may still attach, but its failure exits the phase
    // instead of re-arming it (DRP Try flows only)
    bool try_expired = false;
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
    template<typename STATE, bool WATCHING = requires { STATE::vbus_watch; }>
    struct pred
        : std::bool_constant<
              STATE::vbus_watch == vbus_level::unwatched // no reports, nothing implied
                  ? true
                  : familyOf(STATE::vbus_watch) == vbus_family::discharge
                        ? (fsm::handles_event_v<TABLE, STATE, event::vbus_reached_safe0v> &&
                           fsm::handles_event_v<TABLE, STATE, event::vbus_left_safe0v>)
                        : (fsm::handles_event_v<TABLE, STATE, event::vbus_present> &&
                           fsm::handles_event_v<TABLE, STATE, event::vbus_removed>)> {};

    // a state without the member cannot arm anything and implies nothing
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

    // the states carry their watch level in their annotation sets
    void notifyEntry(vbus_level level)
    {
        monitored = level;
        vbus.monitor(level);
    }

    // Every state declares its level - vbus_level::unwatched is the
    // deliberate "monitoring off", never an omission
    template<fsm::concepts::transition_table TABLE>
    static constexpr void validate()
    {
        static_assert(fsm::all_states_notified_v<annotation_probe<vbus_level>, TABLE>,
                      "vbus_watcher: every state must annotate its vbus_watch level "
                      "(vbus_level::unwatched switches monitoring off)");
        static_assert(watch_events_consistent_v<TABLE>,
                      "vbus_watcher: a watching state must handle its level's event family");
    }

    VBUS& vbus;
    vbus_level monitored = vbus_level::unwatched;
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
    // start() does nothing (the queued process() reports acceptance,
    // not whether the transition fired - the latch keeps this
    // idempotent)
    void startPort()
    {
        if (started_) {
            return;
        }
        started_   = true;
        auto& self = derived();
        self.sm_.process(event::started{});
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
    // handle are dropped by the machine. Reports may arrive
    // synchronously from within a state entry's monitor re-arm: sm_
    // is an fsm::QueuedMachine, which delivers them in order after
    // the running transition completes
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

    bool started_ = false;
};

} // namespace tc

} // namespace usbc
