/*
 * USB Type-C sink port: the sink connection layer (state/TypeCSink.hpp)
 * behind one class, driving a CC and a VBUS driver. No PD involved -
 * the port detects attach/detach, resolves plug orientation and the Rp
 * current advertisement, and switches the sink power path.
 *
 * Integration: hand over initialized drivers - the CC block
 * (interface/Cc.hpp) and the VBUS block (interface/Vbus.hpp), both
 * observers of this machine applying the states' terminations, power
 * paths and watched levels themselves - and two caller-owned timer
 * policy instances, the connection states' and the CC debounce's.
 * Construction rests in the spec's Disabled state (open terminations,
 * nothing monitored); start() is the go-live moment - it registers the
 * drivers' callbacks, fires the started event, whose transition
 * applies the Unattached.SNK terminations, and hands the watcher the
 * line as it is. The drivers must invoke their callbacks from the
 * stack's serialized context (the Zephyr adapters deliver on a
 * workqueue), and the timers are serialized with them by the
 * integrator (mtl timer contract). Attach results reach injected
 * observers watching the attached state's tc::attach_info; their
 * notifications may originate from the timer context on debounce
 * paths.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/interface/Cc.hpp>
#include <usbc/interface/Vbus.hpp>
#include <usbc/layer/TypeC.hpp>
#include <usbc/state/TypeC.hpp>
#include <usbc/state/TypeCSink.hpp>

#include <mtl/StateMachine.hpp>

#include <tuple>

namespace usbc {

template<concepts::cc CC, concepts::vbus VBUS, fsm::concepts::timer TIMER, typename... OBSERVERs>
class TypeCSink : public tc::port_frontend<TypeCSink<CC, VBUS, TIMER, OBSERVERs...>, CC, VBUS> {
    static_assert(
        tc::hardware_annotations_complete_v<tc::tc_sink_table>,
        "every state must annotate its CC termination, VBUS power path and watched VBUS level"
    );

public:
    // Construction rests in Disabled with open terminations; the port
    // goes live on start(). timer runs the connection states' timeouts,
    // cc_timer the CC debounce. The drivers observe the machine, the
    // observers are injected after the built-in ones (timer, the two
    // drivers, cc watcher); attach results are observed on the attached
    // state's attach_info
    TypeCSink(CC& cc, VBUS& vbus, TIMER& timer, TIMER& cc_timer, OBSERVERs&... observers)
        : cc_driver_(cc), vbus_driver_(vbus), cc_(cc_timer), timer_(timer), timed_(timer_),
          observers_(observers...), sm_(timed_, cc, vbus, cc_, observers...)
    {
    }

    // The go-live moment, provided by the shared frontend; a sink
    // presents no Rp
    void start() { this->startPort(rp_value::usb_default); }

    // PD-directed Type-C Error Recovery: both terminations removed for
    // tErrorRecovery, then resolution restarts. Call from the stack's
    // serialized context
    bool errorRecovery() { return sm_.process(tc::event::error_recovery{}); }

    // PD-directed hard-reset window: the attach is held while VBUS
    // legitimately cycles through vSafe0V and back
    bool hardResetWindow() { return sm_.process(tc::event::hard_reset{}); }

private:
    friend tc::port_frontend<TypeCSink, CC, VBUS>;

    CC& cc_driver_;
    VBUS& vbus_driver_;
    tc::CcWatcher<TIMER> cc_;
    fsm::QueuedTimer<TIMER> timer_;
    fsm::timed<fsm::QueuedTimer<TIMER>&> timed_;
    std::tuple<OBSERVERs&...> observers_;
    fsm::QueuedMachine<
        tc::tc_sink_table,
        4,
        fsm::inline_work,
        fsm::no_lock,
        fsm::timed<fsm::QueuedTimer<TIMER>&>,
        CC,
        VBUS,
        tc::CcWatcher<TIMER>,
        OBSERVERs...>
        sm_;
};

} // namespace usbc
