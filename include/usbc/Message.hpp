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

// The numeric values of these three enums are the pd_header field
// encodings
enum class power_role : std::uint8_t { sink = 0, source = 1 };

enum class data_role : std::uint8_t { ufp = 0, dfp = 1 };

enum class pd_revision : std::uint8_t { rev_1_0 = 0, rev_2_0 = 1, rev_3_x = 2 };

// Contiguous zero-based values; used as per-SOP* array index
enum class sop_type : std::uint8_t {
    sop                    = 0,
    sop_prime              = 1,
    sop_double_prime       = 2,
    sop_prime_debug        = 3,
    sop_double_prime_debug = 4,
};

// The 16-bit PD message header. The two role fields carry the port's
// roles in SOP messages; in SOP'/SOP'' messages bit 8 is the cable
// plug indicator and bit 5 is reserved - a cable-facing layer
// reinterprets them.
struct pd_header {
    std::uint8_t message_type{};              // 5 bits; control message if
                                              // num_data_objects == 0 and not extended
    data_role port_data_role{data_role::ufp}; // bit 5
    pd_revision revision{pd_revision::rev_3_x};
    power_role port_power_role{power_role::sink};
    std::uint8_t message_id{};       // 3 bits
    std::uint8_t num_data_objects{}; // 3 bits
    bool extended{};

    static constexpr pd_header decode(std::uint16_t raw)
    {
        return {
            .message_type     = static_cast<std::uint8_t>(raw & 0x1fu),
            .port_data_role   = static_cast<data_role>((raw >> 5u) & 0x1u),
            .revision         = static_cast<pd_revision>((raw >> 6u) & 0x3u),
            .port_power_role  = static_cast<power_role>((raw >> 8u) & 0x1u),
            .message_id       = static_cast<std::uint8_t>((raw >> 9u) & 0x7u),
            .num_data_objects = static_cast<std::uint8_t>((raw >> 12u) & 0x7u),
            .extended         = ((raw >> 15u) & 0x1u) != 0u,
        };
    }

    constexpr std::uint16_t encode() const
    {
        return static_cast<std::uint16_t>(
            (message_type & 0x1fu) | (static_cast<std::uint16_t>(port_data_role) << 5u) |
            (static_cast<std::uint16_t>(revision) << 6u) |
            (static_cast<std::uint16_t>(port_power_role) << 8u) |
            ((message_id & 0x7u) << 9u) | ((num_data_objects & 0x7u) << 12u) |
            (static_cast<std::uint16_t>(extended) << 15u));
    }

    constexpr bool operator==(pd_header const&) const = default;
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
