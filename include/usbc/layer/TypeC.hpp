/*
 * What every USB Type-C connection layer runs around its table: the
 * CcWatcher observer debouncing the CC driver's reports by each
 * state's rule (state/TypeC.hpp) and reporting the status that held
 * as event::cc_detected, and the frontend wiring the drivers'
 * callbacks into the machine. The ports TypeCSink, TypeCSource and
 * TypeCDrp compose both.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/interface/Cc.hpp>
#include <usbc/interface/Vbus.hpp>
#include <usbc/state/TypeC.hpp>

#include <mtl/StateMachine.hpp>

#include <chrono>
#include <optional>
#include <tuple>

namespace usbc {

namespace tc {

using cc_detected_callback = void (*)(void* context, cc_status cc);

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
