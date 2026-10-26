/*
 * The facts the USB Type-C connection machines publish, as the
 * observers hear them: the attach and its role, the PD connection's
 * span, the swap standby, and the directions the PD layer gives the
 * connection layer (annotated on the policy engines' states, executed
 * by the port). The predicates read the CC status a fact carries. No
 * state, no machine: an application observer, the protocol layer and
 * the policy engines include this and nothing of the connection layer.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/interface/Cc.hpp>
#include <usbc/spec/Message.hpp>

#include <cstdint>

namespace usbc {

namespace tc {

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

// The cable asks to be powered: Ra on the pin the plug does not use
constexpr bool raPresent(cc_status status)
{
    return status.cc1 == cc_state::src_ra || status.cc2 == cc_state::src_ra;
}

// The attach result of a sink, observed on the attached state
struct attach_info {
    plug_orientation orientation;
    rp_value advertisement;
};

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

// The data role this port holds in the connection, an instance value
// of the Attached states delivered on every entry (a fresh attach
// resolves it, a resume and a completed power role swap keep it). An
// agreed DR_Swap flips it in place, so the flip itself is heard from
// the engine's pe::data_role_changed, not from here
struct attached_data_role {
    data_role role;
    constexpr bool operator==(attached_data_role const&) const = default;
};

// The attach as the Attached state of each role resolved it, an
// instance value delivered on every entry: what brought the port here
// and the CC status the attach rests on. The type names the role, so
// the sink engine observes the one, the source engine and the vconn
// machine the other - a sink reads the source's Rp from the status
// (SinkTxOk/SinkTxNG, PD3 collision avoidance), a source whether the
// cable presents Ra
struct attached_as_sink {
    attach_origin origin;
    cc_status cc;
};
struct attached_as_source {
    attach_origin origin;
    cc_status cc;
};

// The window of a PR_Swap where the roles change hands (DRP): power
// paths off, Rd presented, VBUS not watched and the CC pins not
// monitored - VBUS is legitimately absent and the partner's
// terminations flap, neither is a detach. The annotation marks that
// window and the role the swap heads for: completeSwap()/abortSwap()
// only mean something inside it (the facade asks the machine), and the
// engines hand the connection over on it. The PD connection holds
// throughout - the standbys carry its tag
struct swap_standby {
    power_role to;
    constexpr bool operator==(swap_standby const&) const = default;
};

// What the PD layer directs the connection layer to do: a policy
// engine state carries one of these in its annotation set, and the
// port executes it from the engine's machine. All are compile-time
// facts of their states, comparable for the sets' change suppression
struct request_error_recovery { // nHardResetCount exhausted
    constexpr bool operator==(request_error_recovery const&) const = default;
};
struct hard_reset_window { // hold the attach while VBUS cycles
    constexpr bool operator==(hard_reset_window const&) const = default;
};
struct enter_swap_standby { // agreed PR_Swap: hold the connection in the
    power_role role;        // swap standby toward this role
    constexpr bool operator==(enter_swap_standby const&) const = default;
};
struct assert_new_role { // a PR_Swap reached the termination change
    power_role role;
    constexpr bool operator==(assert_new_role const&) const = default;
};
struct swap_completed { // the new source's PS_RDY: the swap is done
    constexpr bool operator==(swap_completed const&) const = default;
};

} // namespace tc

} // namespace usbc
