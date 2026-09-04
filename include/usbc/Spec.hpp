/*
 * Constants of the USB Type-C Cable and Connector Specification and
 * the USB Power Delivery Specification, under their spec names so code
 * can be reviewed against the standard. Ranged timing parameters are
 * fsm::timeout_range values; each state machine pairs its table with a
 * timer-range map from its states to these ranges, checked table-wide
 * by fsm::timeouts_within_bounds. Fixed values and counters are
 * referenced directly.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Units.hpp>

#include <mtl/StateMachine.hpp>

#include <chrono>
#include <cstdint>

namespace usbc {

namespace spec {

using fsm::timeout_range;

constexpr bool within(auto value, auto min, auto max)
{
    return min <= value && value <= max;
}

using std::chrono::microseconds;
using std::chrono::milliseconds;

// --- USB Type-C Cable and Connector Specification ----------------------------

// CC timing
inline constexpr timeout_range t_cc_debounce{milliseconds{100}, milliseconds{200}}; // tCCDebounce
inline constexpr timeout_range t_pd_debounce{milliseconds{10}, milliseconds{20}};   // tPDDebounce

// DRP timing
inline constexpr timeout_range t_drp{milliseconds{50}, milliseconds{100}}; // tDRP toggle period
inline constexpr unsigned dc_src_drp_min = 30; // dcSRC.DRP, percent of tDRP at Rp
inline constexpr unsigned dc_src_drp_max = 70;
// one role's slice of the toggle: dcSRC.DRP min * tDRP min to max * max
inline constexpr timeout_range t_drp_pw{milliseconds{15}, milliseconds{70}};
inline constexpr timeout_range t_drp_try{milliseconds{75}, milliseconds{150}};        // tDRPTry
inline constexpr timeout_range t_drp_try_wait{milliseconds{400}, milliseconds{800}};  // tDRPTryWait
inline constexpr timeout_range t_try_cc_debounce{milliseconds{10}, milliseconds{20}}; // tTryCCDebounce
inline constexpr timeout_range t_try_timeout{milliseconds{550}, milliseconds{1100}};  // tTryTimeout

// VBUS thresholds
inline constexpr millivolt v_safe_0v_max = 800;  // vSafe0V
inline constexpr millivolt v_safe_5v_min = 4750; // vSafe5V
inline constexpr millivolt v_safe_5v_nom = 5000;
inline constexpr millivolt v_safe_5v_max = 5500;
inline constexpr millivolt v_sink_disconnect_max = 3670; // vSinkDisconnect

// Rp current advertisements; usb_default is the USB 2.0 default load
inline constexpr milliamp i_usb_default = 500;
inline constexpr milliamp i_rp_1a5     = 1500;
inline constexpr milliamp i_rp_3a0     = 3000;

// --- USB Power Delivery Specification ----------------------------------------

// Protocol layer timing
inline constexpr timeout_range t_receive{microseconds{900}, microseconds{1100}}; // tReceive
inline constexpr timeout_range t_hard_reset_complete{milliseconds{0}, milliseconds{5}};

// Policy engine timing
inline constexpr timeout_range t_sender_response{milliseconds{24}, milliseconds{30}};
inline constexpr timeout_range t_chunking_not_supported{milliseconds{40}, milliseconds{50}};
inline constexpr timeout_range t_typec_send_source_cap{milliseconds{100}, milliseconds{200}};
inline constexpr timeout_range t_src_transition{milliseconds{25}, milliseconds{35}};
inline constexpr timeout_range t_src_recover{milliseconds{660}, milliseconds{1000}};
inline constexpr timeout_range t_sink_wait_cap{milliseconds{310}, milliseconds{620}};
inline constexpr timeout_range t_ps_transition{milliseconds{450}, milliseconds{550}};

// PR_Swap sequencing: how long the new source waits for the old
// source's PS_RDY, and the old source for the new source's
inline constexpr timeout_range t_ps_source_off{milliseconds{750}, milliseconds{920}};
inline constexpr timeout_range t_ps_source_on{milliseconds{390}, milliseconds{480}};

// Counters
inline constexpr std::uint8_t n_retry_count = 2;  // nRetryCount, PD rev 3.x
inline constexpr std::uint8_t n_caps_count  = 50; // nCapsCount

// Sink standby draw during a transition, at any voltage
inline constexpr milliamp i_snk_stdby = 500; // iSnkStdby

} // namespace spec

} // namespace usbc
