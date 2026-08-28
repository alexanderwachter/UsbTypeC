/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace usbc {

enum class power_role : std::uint8_t { sink, source };

enum class data_role : std::uint8_t { ufp, dfp };

// USB PD Specification Revision as encoded in the message header
enum class pd_revision : std::uint8_t { rev_1_0, rev_2_0, rev_3_x };

enum class sop_type : std::uint8_t {
    sop,
    sop_prime,
    sop_double_prime,
    sop_prime_debug,
    sop_double_prime_debug,
};

// One USB PD message as it crosses the port controller interface: the
// 16-bit message header plus the raw payload bytes (data objects for
// standard messages, extended header + data for chunked extended ones).
// CRC and ordered sets stay inside the TCPC.
struct pd_message {
    static constexpr std::size_t max_payload_size = 28; // 7 data objects

    sop_type sop{sop_type::sop};
    std::uint16_t header{};
    std::uint8_t payload_size{}; // bytes used in payload
    std::array<std::uint8_t, max_payload_size> payload{};
};

} // namespace usbc
