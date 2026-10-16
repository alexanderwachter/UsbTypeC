/*
 * Port-control questions an application answers at runtime: fsm
 * guard tags asked by the tables, answered by an object injected
 * into the port (every machine of the port sees it).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

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

} // namespace usbc
