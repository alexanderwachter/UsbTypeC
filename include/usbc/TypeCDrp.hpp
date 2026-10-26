/*
 * USB Type-C dual-role port (DRP): the DRP connection layer
 * (state/TypeCDrp.hpp) behind one class. The port toggles between
 * Unattached.SNK and Unattached.SRC and follows the sink or source
 * attach flow of whichever role it was advertising when a partner
 * appeared; the toggle timing (drp_timing) and the role preference
 * (drp_preference) are the table's compile-time policies.
 *
 * Integration matches TypeCSink/TypeCSource: initialized drivers -
 * the CC and VBUS blocks, observers of this machine - plus two
 * caller-owned timer policy instances; construction rests in the
 * spec's Disabled state and start() goes live in Unattached.SNK. The
 * callback's meaning is mapped through the level it reports on.
 * Injected observers watching the attached states' instance values
 * learn which role attached through the info type: tc::attach_info
 * (orientation and advertisement) for Attached.SNK only; both roles
 * deliver the plug_orientation. An observer answering
 * check(usbc::sourcing_allowed) is the role lock (sink-only while it
 * says no). The PD-directed role swaps are the PdDrp facade's calls.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/annotation/TypeC.hpp>
#include <usbc/interface/Cc.hpp>
#include <usbc/interface/Questions.hpp>
#include <usbc/interface/Vbus.hpp>
#include <usbc/layer/TypeC.hpp>
#include <usbc/spec/Message.hpp>
#include <usbc/state/TypeC.hpp>
#include <usbc/state/TypeCDrp.hpp>

#include <mtl/StateMachine.hpp>

#include <optional>
#include <tuple>
#include <type_traits>

namespace usbc {

template<
    concepts::cc CC,
    concepts::vbus VBUS,
    fsm::concepts::timer TIMER,
    drp_timing const& TIMING = default_drp_timing,
    drp_preference PREFERENCE = drp_preference::none,
    typename... OBSERVERs>
class TypeCDrp
    : public tc::
          port_frontend<TypeCDrp<CC, VBUS, TIMER, TIMING, PREFERENCE, OBSERVERs...>, CC, VBUS> {
    static_assert(
        tc::hardware_annotations_complete_v<tc::drp::table_for_t<TIMING, PREFERENCE>>,
        "every state must annotate its CC termination, VBUS power path and watched VBUS level"
    );

public:
    // Construction rests in Disabled with open terminations; the port
    // goes live on start(). timer runs the connection states' timeouts
    // (the toggle, the Try walls, the standbys), cc_timer the CC
    // debounce. The advertisement is the Rp presented whenever the
    // port advertises the source role. The drivers observe the machine
    // (one VBUS switch position for both roles: entering the other
    // role's states collapses this one's path by value), the observers
    // are injected after the built-in ones (timer, the two drivers, cc
    // watcher); attach results are observed on the attached states'
    // instance values with the role encoded in the info type, and an
    // observer answering check(usbc::sourcing_allowed) is the role lock
    // (sink-only while it says no)
    TypeCDrp(
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
    TypeCDrp(CC& cc, VBUS& vbus, TIMER& timer, TIMER& cc_timer, OBSERVERs&... observers)
        : TypeCDrp(cc, vbus, timer, cc_timer, rp_value::usb_default, observers...)
    {
    }

    // Power role swap directed by the layer above (a USB PD PR_Swap
    // both sides agreed to - the arbitration happened there), in
    // phases: begin enters the swap standby - power paths off, Rd
    // presented, detach detection suspended while VBUS is legitimately
    // absent - completeSwap() (the partner's PS_RDY) lands in the new
    // attached state, abortSwap() restores the departing role. A
    // standby left to its spec timeout (tPSSourceOff/tPSSourceOn)
    // falls back to the departing role on its own. False when not
    // attached in the departing role. Call from the stack's serialized
    // context
    bool beginSwapToSource()
    {
        if (powerRole() != power_role::sink) {
            return false;
        }
        return sm_.process(tc::event::swap_to_source{});
    }

    bool beginSwapToSink()
    {
        if (powerRole() != power_role::source) {
            return false;
        }
        return sm_.process(tc::event::swap_to_sink{});
    }

    // Both only mean something in a swap standby - the standbys'
    // swap_standby annotation answers that, where the old process()
    // return cannot (the queued machine reports acceptance, not whether
    // a transition fired)
    bool completeSwap() { return inSwapStandby() && sm_.process(tc::event::swap_complete{}); }

    bool abortSwap() { return inSwapStandby() && sm_.process(tc::event::swap_abort{}); }

    // Data role swap directed by the layer above (an agreed USB PD
    // DR_Swap): no termination changes, only the context's data role
    // flips, by an internal transition - observers hear the flip from
    // the engine's pe::data_role_changed, the role at attach from
    // tc::attached_data_role. False while not attached
    bool swapDataRole()
    {
        if (!dataRole()) {
            return false;
        }
        return applyDataRoleSwap();
    }

    // The flip without the arbitration: for a PD layer applying a
    // DR_Swap it already negotiated (the verdict was asked when the
    // message exchange began). The attach check replaces the old
    // process() return: the queued machine reports acceptance, not
    // whether the internal transition fired
    bool applyDataRoleSwap()
    {
        if (!dataRole()) {
            return false;
        }
        sm_.process(tc::event::swap_data_role{});
        return true;
    }

    // The role lock's answer (usbc::sourcing_allowed): the injected
    // object answering it decides, the question's static default (yes)
    // stands without one - the same resolution the machine applies
    bool sourcingAllowed()
    {
        return std::apply(
            [](auto&... observer) { return (allowsSourcing(observer) && ...); },
            observers_
        );
    }

    // The attached pair's power role, as the Attached states annotate
    // it; nullopt while not attached (a swap standby included)
    std::optional<power_role> powerRole() const
    {
        if (auto const attached = sm_.template annotation<tc::attached_role>()) {
            return attached->role;
        }
        return std::nullopt;
    }

    // The attached pair's data role, from the connection's attachment;
    // nullopt while not attached
    std::optional<data_role> dataRole() const
    {
        if (powerRole()) {
            return sm_.template context<tc::attachment>().data;
        }
        return std::nullopt;
    }

    // The go-live moment, provided by the shared frontend
    void start() { this->startPort(advertisement_); }

    // PD-directed Type-C Error Recovery: both terminations removed for
    // tErrorRecovery, then resolution restarts from the preference's
    // Unattached state. Call from the stack's serialized context
    bool errorRecovery() { return sm_.process(tc::event::error_recovery{}); }

    // PD-directed hard-reset window: the attach is held while VBUS
    // legitimately cycles through vSafe0V and back
    bool hardResetWindow() { return sm_.process(tc::event::hard_reset{}); }

private:
    friend tc::port_frontend<TypeCDrp, CC, VBUS>;

    bool inSwapStandby() const { return sm_.template annotation<tc::swap_standby>().has_value(); }

    static bool allowsSourcing(auto& observer)
    {
        if constexpr (
            fsm::concepts::
                answers_stateless_guard<std::remove_cvref_t<decltype(observer)>, sourcing_allowed>
        ) {
            return observer.check(sourcing_allowed{});
        } else {
            return true;
        }
    }

    using table = tc::drp::table_for_t<TIMING, PREFERENCE>;
    using queued_timer = fsm::QueuedTimer<TIMER>;

    CC& cc_driver_;
    VBUS& vbus_driver_;
    tc::CcWatcher<TIMER> cc_;
    queued_timer timer_;
    fsm::timed<queued_timer&> timed_;
    rp_value advertisement_;
    std::tuple<OBSERVERs&...> observers_;
    fsm::QueuedMachine<
        table,
        4,
        fsm::inline_work,
        fsm::no_lock,
        fsm::timed<queued_timer&>,
        CC,
        VBUS,
        tc::CcWatcher<TIMER>,
        OBSERVERs...>
        sm_;
};

} // namespace usbc
