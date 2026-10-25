/*
 * Common ground of the USB Type-C connection layers: the spec's
 * debounce times, the CcWatcher observer debouncing the CC driver's
 * reports by each state's rule and reporting the status that held as
 * event::cc_detected, the check that every state declares what the
 * hardware blocks observe, and the frontend wiring the drivers'
 * callbacks into the machine. The hardware itself - the CC block
 * (Cc.hpp) and the VBUS block (Vbus.hpp) - observes the states'
 * annotations directly. The sink and source layers live in
 * TypeCSink.hpp and TypeCSource.hpp.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Cc.hpp>
#include <usbc/Message.hpp>
#include <usbc/Spec.hpp>
#include <usbc/Vbus.hpp>

#include <mtl/StateMachine.hpp>

#include <chrono>
#include <optional>
#include <tuple>
#include <type_traits>

namespace usbc {

namespace tc {

// One debounce time of a state's CC rule: how long a status has to
// hold before it counts as detected - a spec time (Table 4-33) as its
// count of milliseconds, or at_once. An enum: a byte per time on every
// monitoring state, and the DOT graphs show the name
enum class debounce_time : std::uint8_t {
    at_once = 0,
    t_pd_debounce = 15,
    t_try_cc_debounce = 15,
    t_src_disconnect = 15,
    t_cc_debounce = 150,
};

constexpr std::chrono::milliseconds durationOf(debounce_time time)
{
    return std::chrono::milliseconds{static_cast<std::underlying_type_t<debounce_time>>(time)};
}

inline constexpr auto t_cc_debounce = durationOf(debounce_time::t_cc_debounce);
inline constexpr auto t_pd_debounce = durationOf(debounce_time::t_pd_debounce);
inline constexpr auto t_try_cc_debounce = durationOf(debounce_time::t_try_cc_debounce);
inline constexpr auto t_src_disconnect = durationOf(debounce_time::t_src_disconnect);
inline constexpr auto t_error_recovery = std::chrono::milliseconds{50};      // tErrorRecovery
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

constexpr bool isRa(cc_state state)
{
    return state == cc_state::src_ra;
}

// A source connection as a sink detects it: Rp on at least one pin
constexpr bool sourcePresent(cc_status status)
{
    return isRp(status.cc1) || isRp(status.cc2);
}

// A sink or accessory connection as a source detects it: Rd on either
// pin, or Ra on both
constexpr bool sinkPresent(cc_status status)
{
    return isRd(status.cc1) || isRd(status.cc2) || (isRa(status.cc1) && isRa(status.cc2));
}

// PD3 collision avoidance, seen from the sink: the source's Rp reads
// as SinkTxOk (3.0 A advertisement) or SinkTxNG
constexpr bool sinkTxOk(cc_status status)
{
    return status.cc1 == cc_state::snk_power_3a0 || status.cc2 == cc_state::snk_power_3a0;
}

namespace event {

// The CC status that held for the state's debounce (CcWatcher's
// report; the spec's "detected"), the only form in which the CC pins
// reach the connection machines
struct cc_detected {
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

// The attached pair's power role, carried by the Attached states only:
// the facades' powerRole() reads it off the machine (a swap standby or
// a hard-reset window carries none - not attached in either role)
struct attached_role {
    power_role role;
    constexpr bool operator==(attached_role const&) const = default;
};

// The connection to the partner, carried by every state in which the
// PD layer's negotiation persists: the Attached states and the windows
// holding an attach on the PD layer's word (the sink's hard-reset
// window while VBUS legitimately cycles, the DRP's power-swap
// standbys). A port's engines live exactly as long as the tag: its
// entry is the connection's start, its exit the one real detach - the
// moves inside the group are change-suppressed
struct pd_connection {
    constexpr bool operator==(pd_connection const&) const = default;
};

// What brought the port into an Attached state - the state knows from
// its constructor: a fresh attach, VBUS returning after the sink's
// hard-reset window (plug and roles kept), or the completion of a
// PD-directed power role swap (the engine already live mid-swap)
enum class attach_origin : std::uint8_t {
    fresh_attach,
    resumed_after_hard_reset,
    completed_power_role_swap,
};

// The partner as the attach resolved it, an instance value of the
// Attached states delivered on every entry: what brought the port
// here, the data role this port holds and the CC status the attach
// rests on - a sink reads the source's Rp from it (SinkTxOk/SinkTxNG,
// PD3 collision avoidance), a source whether the cable presents Ra
struct attached_partner {
    attach_origin origin;
    data_role data;
    cc_status cc;
};

// The one machine-owned context of the connection layer: one
// connection's attachment. The detected CC status the attach rests on
// (the states waiting for a VBUS report after a detection keep it here
// for the Attached state to resolve plug orientation and the Rp
// advertisement from, and attached_partner reports it), the plug
// orientation and data role a fresh attach resolves, kept across a
// hard-reset window and a power role swap (a power swap leaves the
// data role alone, per the PD spec). The Unattached anchors reset it;
// `resolved` tells Attached.SNK whether VBUS returning resumes the
// connection or attaches afresh. The CC status itself is no context:
// the decision and the status arrive together in event::cc_detected
struct attachment {
    cc_status detected = {.cc1 = cc_state::snk_open, .cc2 = cc_state::snk_open};
    plug_orientation orientation = plug_orientation::cc1;
    data_role data = data_role::ufp;
    bool resolved = false;
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

// The report the driver delivers when the armed condition is met: the
// event a state arms the level for
template<vbus_level LEVEL>
using met_event_t = std::conditional_t<
    familyOf(LEVEL) == vbus_family::discharge,
    fsm::events<event::vbus_reached_safe0v>,
    std::conditional_t<
        metMeansPresent(LEVEL),
        fsm::events<event::vbus_present>,
        fsm::events<event::vbus_removed>>>;

// The event a state owes for the VBUS level it watches: the met
// report of its level - a state arming a condition without a
// transition on its report waits for nothing (a lost detach at worst).
// A state without the annotation, or with monitoring off, owes
// nothing; fsm::all_states_handle is the table-wide proof
template<fsm::concepts::state STATE>
struct required_vbus_events : std::type_identity<fsm::events<>> {};

template<fsm::concepts::state STATE>
    requires requires { STATE::annotations.template get<vbus_level>(); } &&
             (STATE::annotations.template get<vbus_level>() != vbus_level::unwatched)
struct required_vbus_events<STATE>
    : std::type_identity<met_event_t<STATE::annotations.template get<vbus_level>()>> {};

template<fsm::concepts::transition_table TABLE>
inline constexpr bool watch_events_consistent_v =
    fsm::all_states_handle_v<TABLE, required_vbus_events>;

// What the hardware blocks observe, declared by every connection state:
// the CC termination and the VBUS switch position (a state the
// dispatch would silently skip would leave the previous state's
// terminations or power paths applied) and the watched VBUS level
// (vbus_level::unwatched is the deliberate "monitoring off", never an
// omission), with a transition on the level's met report. The ports
// assert it on their tables
template<fsm::concepts::transition_table TABLE>
inline constexpr bool hardware_annotations_complete_v =
    fsm::all_states_carry_v<TABLE, cc_termination> && fsm::all_states_carry_v<TABLE, vbus_power> &&
    fsm::all_states_carry_v<TABLE, vbus_level> && watch_events_consistent_v<TABLE>;

// --- CC debounce -------------------------------------------------------------

// Annotation of a connection state monitoring the CC pins - its rule:
// the debounce time of a termination (a source's Rp seen through Rd, a
// sink's Rd or Ra seen through Rp) and of the open line. CcWatcher
// applies it and reports the status that held as event::cc_detected; a
// state without the annotation does not monitor the pins
struct cc_debounce {
    debounce_time termination;
    debounce_time open;
    constexpr bool operator==(cc_debounce const&) const = default;
};

// Guard question answered by CcWatcher: a CC status is under debounce
// right now. The DRP's Try walls apply only while the wanted
// termination "has not yet been detected"
struct cc_state_pending {};

using cc_detected_callback = void (*)(void* context, cc_status cc);

// The debounce times applied, within the spec's ranges
static_assert(spec::t_pd_debounce.contains(t_pd_debounce));
static_assert(spec::t_try_cc_debounce.contains(t_try_cc_debounce));
static_assert(spec::t_src_disconnect.contains(t_src_disconnect));
static_assert(spec::t_cc_debounce.contains(t_cc_debounce));

// Debounces the CC pins by the connection state's rule, on one timer.
// Injected into the connection machine, it observes the state's rule
// and the pull it presents; the CC driver's reports come in through
// report(), the status that held goes out through the callback (the
// port frontend turns it into event::cc_detected). A rule change
// evaluates the last report under the new rule - the spec counts a
// debounce from the state's entry - except after a pull change, when
// the last report means nothing until the driver has measured the line
// afresh; a state without a rule stops the watcher. A report equal to
// the last one is no change and leaves a running debounce alone; a
// report that differs restarts it, so at the timeout the last report is
// the status that held. The timer's callback arrives in the stack's
// serialized context (mtl timer contract), like the driver's reports
template<fsm::concepts::timer TIMER>
struct CcWatcher : fsm::observing<CcWatcher<TIMER>> {
    explicit CcWatcher(TIMER& timer_ref) : timer_(timer_ref) {}

    void notifyEntry(cc_termination) { reported_ = false; }

    void notifyEntry(cc_debounce rule)
    {
        rule_ = rule;
        evaluate();
    }

    void notifyExit(cc_debounce)
    {
        rule_.reset();
        timer_.stop();
        pending_ = false;
    }

    bool check(cc_state_pending) const { return pending_; }

    // the CC driver's report
    void report(cc_status cc)
    {
        if (reported_ && cc == status_) {
            return;
        }
        status_ = cc;
        reported_ = true;
        evaluate();
    }

    void setCallback(cc_detected_callback callback, void* context)
    {
        callback_ = callback;
        context_ = context;
    }

private:
    void evaluate()
    {
        if (!reported_ || !rule_) {
            return;
        }
        auto const time = terminationPresent() ? rule_->termination : rule_->open;
        if (time == debounce_time::at_once) {
            timer_.stop();
            pending_ = false;
            detected();
        } else {
            debounceFor(durationOf(time)); // a running timer is rescheduled
        }
    }

    void debounceFor(std::chrono::milliseconds time)
    {
        pending_ = true;
        timer_.start(
            time,
            [](void* watcher) { static_cast<CcWatcher*>(watcher)->debounced(); },
            this
        );
    }

    void debounced()
    {
        pending_ = false;
        detected();
    }

    void detected()
    {
        if (callback_ != nullptr) {
            callback_(context_, status_);
        }
    }

    // the driver reports in the presented role's encoding: a sink sees
    // snk_* states, a source src_* states
    bool terminationPresent() const { return sourcePresent(status_) || sinkPresent(status_); }

    TIMER& timer_;
    cc_status status_{};
    std::optional<cc_debounce> rule_{};
    bool reported_ = false;
    bool pending_ = false;
    cc_detected_callback callback_ = nullptr;
    void* context_ = nullptr;
};

// The driver-facing frontend every connection layer shares (CRTP):
// start() wiring, the CC driver's reports handed to the CcWatcher and
// its detections turned into events, the vbus driver's report mapped
// through the family of the level it reports on. The derived port
// provides cc_driver_, vbus_driver_, cc_ (the watcher), observers_,
// and sm_, and befriends this base
template<typename DERIVED, concepts::cc CC, concepts::vbus VBUS>
class port_frontend {
protected:
    // Leaves Disabled through the started event with the callbacks in
    // place, so the drivers' first reports - the VBUS condition the
    // Unattached state arms - arrive; the Rp advertisement is set
    // before the terminations are applied, and the line as it is
    // reaches the watcher (a partner plugged in before changes nothing
    // the driver would report). A second start() does nothing (the
    // queued process() reports acceptance, not whether the transition
    // fired - the latch keeps this idempotent)
    void startPort(rp_value advertisement)
    {
        if (started_) {
            return;
        }
        started_ = true;
        auto& self = derived();
        self.cc_.setCallback(
            [](void* frontend, cc_status cc) {
                static_cast<port_frontend*>(frontend)->ccDetected(cc);
            },
            this
        );
        self.cc_driver_.setCallback(
            [](void* frontend, cc_status cc) {
                static_cast<port_frontend*>(frontend)->ccChanged(cc);
            },
            this
        );
        self.vbus_driver_.setCallback(
            [](void* frontend, vbus_level level, bool met) {
                static_cast<port_frontend*>(frontend)->vbusEvent(level, met);
            },
            this
        );
        self.cc_driver_.setRp(advertisement);
        self.sm_.process(event::started{});
        if (auto const cc = self.cc_driver_.readCcStatus()) {
            self.cc_.report(*cc);
        }
    }

private:
    DERIVED& derived() { return static_cast<DERIVED&>(*this); }

    // The driver's report: the watcher debounces it, and the PD layer
    // reads the raw status for the source's Rp as SinkTxOk/SinkTxNG
    // (PD3 collision avoidance)
    void ccChanged(cc_status cc)
    {
        auto& self = derived();
        self.cc_.report(cc);
        std::apply(
            [&](auto&... observer) { (forwardCcStatus(observer, cc), ...); },
            self.observers_
        );
    }

    static void forwardCcStatus(auto& observer, cc_status cc)
    {
        if constexpr (requires { observer.onCcStatus(cc); }) {
            observer.onCcStatus(cc);
        }
    }

    // The status that held for the state's debounce: the watcher's
    // report may arrive from within a state entry's rule change; sm_
    // is an fsm::QueuedMachine, which delivers it after the running
    // transition completes
    void ccDetected(cc_status cc) { derived().sm_.process(event::cc_detected{cc}); }

    // The vbus driver reports on the condition the state armed; the
    // event follows the level's family - events a table does not
    // handle are dropped by the machine. Reports may arrive
    // synchronously from within a state entry's arming: sm_ is an
    // fsm::QueuedMachine, which delivers them in order after the
    // running transition completes
    void vbusEvent(vbus_level level, bool met)
    {
        auto& self = derived();
        if (familyOf(level) == vbus_family::discharge) {
            if (met) {
                self.sm_.process(event::vbus_reached_safe0v{});
            } else {
                self.sm_.process(event::vbus_left_safe0v{});
            }
        } else if (met == metMeansPresent(level)) {
            self.sm_.process(event::vbus_present{});
        } else {
            self.sm_.process(event::vbus_removed{});
        }
    }

    bool started_ = false;
};

} // namespace tc

} // namespace usbc
