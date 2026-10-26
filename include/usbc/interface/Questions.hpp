/*
 * The tables' questions: fsm guard tags asked by the state machines
 * and answered by an object injected into the port (every machine of
 * the port sees it). The application answers the role lock and the
 * swap arbitrations - and answering a swap question is what enables
 * the feature; the VCONN source question is the vconn machine's own.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/annotation/Vconn.hpp>

#include <concepts>

namespace usbc {

// The role lock: may this port act as a source? Asked by the DRP's
// toggle before advertising Rp, by the attach flow before Try.SRC
// (the spec allows a source-preferring port's sink flow to attach),
// and by the sink engine before accepting the partner's PR_Swap. Its
// static default says yes; an application object injected into the
// port answering bool check(sourcing_allowed) overrides it, and while
// it says no the DRP behaves as a sink: it rests at Rd re-asking
// every slice, attaches as sink, refuses swaps to source. A phase
// already running completes (a lock engaged mid-debounce may still
// resolve to Attached.SRC once), a port already sourcing keeps its
// contract - the application requests the swap. Exactly one answering
// object per port (mtl's rule)
struct sourcing_allowed {
    static constexpr bool check() { return true; }
};

namespace pe {

// The engines' arbitration questions - fsm::guard tags without a
// static check, answered by the policy injected into the engine's
// machine: bool check(pe::pr_swap_allowed) - may this port take the
// other power role? - and likewise the other data role and the VCONN
// source role. The request event carries the role to take, for an
// answer wanting it
struct pr_swap_allowed {};
struct dr_swap_allowed {};
struct vconn_swap_allowed {};
// Does this port hold the VCONN Source role? Answered by the port
// from its vconn machine: a holder accepts the partner's VCONN_Swap,
// giving the role up needs no capability
struct vconn_source_role_held {};

// The optional features, as tags - the fsm library's feature mechanism:
// a state declares the feature it belongs to (`using feature =
// pe::..._feature;`), and the tag names the question that enables it:
// a policy answering it is exactly what brings the feature's states
// in, and the machine leaves a disabled feature's states - and every
// entry touching them - out at compile time. The answering policy must
// satisfy the feature's contract (checked where it is detected)
struct pr_swap_feature { // contract: allowSwap(power_role)
    using enabled_by = pr_swap_allowed;
};
struct dr_swap_feature { // contract: allowSwap(data_role)
    using enabled_by = dr_swap_allowed;
};
struct vconn_feature { // contract: concepts::vconn_port (the answerer observes the switch)
    using enabled_by = vconn_swap_allowed;
};

} // namespace pe

namespace concepts {

// The one injected object that both connects the VCONN switch
// hardware - observing the states' switch annotation - and answers
// the engines' question whether this port may take the role; the
// answer is what enables the whole feature
template<typename T>
concept vconn_port = requires(T port, vconn::vconn_switch vconn, pe::vconn_swap_allowed question) {
    port.notifyEntry(vconn);
    { port.check(question) } -> std::convertible_to<bool>;
};

} // namespace concepts

} // namespace usbc
