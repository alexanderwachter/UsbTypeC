/*
 * The VCONN machine running the vconn table (state/Vconn.hpp), one
 * per dual-role port.
 *
 * Integration: the machine observes the connection machine (a fresh
 * source-role attach with Ra turns VCONN on, the PD connection's exit
 * turns it off) and both engines' machines (the VCONN_Swap exchange's
 * progress: agreed, the partner's PS_RDY, our PS_RDY on the wire; a
 * hard reset issued by an engine acknowledges the failed hand-off),
 * and answers the engines' question whether this port holds the VCONN
 * Source role. Everything runs in the stack's serialized context; the
 * timer fires from its own context (mtl timer contract).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/annotation/PolicyEngine.hpp>
#include <usbc/annotation/ProtocolLayer.hpp>
#include <usbc/annotation/TypeC.hpp>
#include <usbc/annotation/Vconn.hpp>
#include <usbc/interface/Questions.hpp>
#include <usbc/state/Vconn.hpp>

#include <mtl/StateMachine.hpp>

namespace usbc {

// The observers carry the vconn_port that enables the feature and
// drives the switch; without one the machine runs vconn_off alone
template<fsm::concepts::timer TIMER, typename... OBSERVERs>
class VconnMachine : public fsm::observing<VconnMachine<TIMER, OBSERVERs...>> {
public:
    VconnMachine(TIMER& timer, OBSERVERs&... observers)
        : timer_(timer), timed_(timer_), sm_(timed_, observers...)
    {
    }

    // The connection machine's facts: only a fresh source-role attach
    // with Ra starts sourcing (a swap's completion preserves whatever
    // the role is), the one real detach ends it. Observed ahead of the
    // source engine: switching VCONN reconfigures the CC lines, which
    // aborts a transmission in flight
    void notifyEntry(tc::attached_as_source const& attach)
    {
        if (attach.origin == tc::attach_origin::fresh_attach) {
            attachedSource(tc::raPresent(attach.cc));
        }
    }
    void notifyExit(tc::pd_connection) { detached(); }

    // The engines' VCONN_Swap progress, observed in both engines'
    // machines; a hard reset issued by an engine is the escalation of
    // a failed hand-off, which leaves VCONN here
    void notifyEntry(pe::vconn_swap_agreed) { swapAgreed(); }
    void notifyEntry(pe::vconn_partner_on) { partnerPsRdy(); }
    void notifyEntry(pe::vconn_ps_rdy_sent) { psRdySent(); }
    void notifyEntry(prl::hard_reset_action) { swapFailureHandled(); }

    // The engines' question on the partner's VCONN_Swap: a holder of
    // the role accepts, relinquishing needs no capability
    bool check(pe::vconn_source_role_held) const { return isVconnSource(); }

    // The same, as calls
    void attachedSource(bool ra) { sm_.process(vconn::event::attached_source{ra}); }
    void detached() { sm_.process(vconn::event::detached{}); }
    void swapAgreed() { sm_.process(vconn::event::swap_agreed{}); }
    void partnerPsRdy() { sm_.process(vconn::event::partner_ps_rdy{}); }
    void psRdySent() { sm_.process(vconn::event::ps_rdy_sent{}); }
    void swapFailureHandled() { sm_.process(vconn::event::swap_failed_handled{}); }

    // Whether this port holds the VCONN Source role, as the states
    // annotate it (an accepted swap away from giving it up still
    // counts - VCONN is on until the hand-off)
    bool isVconnSource() const { return sm_.template annotation<vconn::source_role>().has_value(); }

private:
    fsm::QueuedTimer<TIMER> timer_;
    fsm::timed<fsm::QueuedTimer<TIMER>&> timed_;
    // Queued: the tVCONNSourceTimeout expiry's hard-reset request is
    // acted on from the observing hook, which feeds back into this
    // machine (swapFailureHandled) - ordered delivery, no facade pump
    fsm::QueuedMachine<
        vconn::vconn_table,
        2,
        fsm::inline_work,
        fsm::no_lock,
        fsm::timed<fsm::QueuedTimer<TIMER>&>,
        OBSERVERs...>
        sm_;
};

} // namespace usbc
