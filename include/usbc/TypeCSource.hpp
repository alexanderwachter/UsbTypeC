/*
 * USB Type-C source port: the source connection layer
 * (state/TypeCSource.hpp) behind one class. The port presents Rp with
 * the configured advertisement, detects attach/detach, resolves the
 * plug orientation and switches the source power path.
 *
 * Integration mirrors TypeCSink: initialized drivers - the CC block
 * and the VBUS block, observers of this machine - plus two
 * caller-owned timer policy instances, the connection states' and the
 * CC debounce's. Construction rests in the spec's Disabled state (open
 * terminations, nothing monitored); start() is the go-live moment - it
 * registers the callbacks, sets the Rp advertisement, fires the
 * started event, whose transition presents Rp, and hands the watcher
 * the line as it is. Attach results reach injected observers watching
 * the attached state's plug_orientation; their notifications may
 * originate from the timer context on debounce paths.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/interface/Cc.hpp>
#include <usbc/interface/Vbus.hpp>
#include <usbc/layer/TypeC.hpp>
#include <usbc/state/TypeC.hpp>
#include <usbc/state/TypeCSource.hpp>

#include <mtl/StateMachine.hpp>

#include <tuple>

namespace usbc {

template<concepts::cc CC, concepts::vbus VBUS, fsm::concepts::timer TIMER, typename... OBSERVERs>
class TypeCSource : public tc::port_frontend<TypeCSource<CC, VBUS, TIMER, OBSERVERs...>, CC, VBUS> {
    static_assert(
        tc::hardware_annotations_complete_v<tc::tc_source_table>,
        "every state must annotate its CC termination, VBUS power path and watched VBUS level"
    );

public:
    // Construction rests in Disabled with open terminations; the port
    // goes live on start(). timer runs the connection states' timeouts,
    // cc_timer the CC debounce. The advertisement is the Rp the port
    // presents (and must honor). The drivers observe the machine - the
    // vbus driver also answers the tables' questions about VBUS now -
    // and the observers are injected after the built-in ones (timer,
    // the two drivers, cc watcher); attach results are observed on the
    // attached state's plug_orientation (a source reports no more)
    TypeCSource(
        CC& cc,
        VBUS& vbus,
        TIMER& timer,
        TIMER& cc_timer,
        rp_value advertisement,
        OBSERVERs&... observers
    )
        : cc_driver_(cc), vbus_driver_(vbus), cc_(cc_timer), timer_(timer), timed_(timer_),
          advertisement_(advertisement), observers_(observers...),
          sm_(timed_, cc, vbus, cc_, observers...)
    {
    }
    // Default-Rp convenience: a trailing pack cannot follow a defaulted
    // advertisement
    TypeCSource(CC& cc, VBUS& vbus, TIMER& timer, TIMER& cc_timer, OBSERVERs&... observers)
        : TypeCSource(cc, vbus, timer, cc_timer, rp_value::usb_default, observers...)
    {
    }

    // The go-live moment, provided by the shared frontend
    void start() { this->startPort(advertisement_); }

    // PD-directed Type-C Error Recovery: both terminations removed for
    // tErrorRecovery, then resolution restarts. Call from the stack's
    // serialized context
    bool errorRecovery() { return sm_.process(tc::event::error_recovery{}); }

private:
    friend tc::port_frontend<TypeCSource, CC, VBUS>;

    CC& cc_driver_;
    VBUS& vbus_driver_;
    tc::CcWatcher<TIMER> cc_;
    fsm::QueuedTimer<TIMER> timer_;
    fsm::timed<fsm::QueuedTimer<TIMER>&> timed_;
    rp_value advertisement_;
    std::tuple<OBSERVERs&...> observers_;
    fsm::QueuedMachine<
        tc::tc_source_table,
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
