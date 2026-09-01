/*
 * PD message types and the data-object codec the policy engine needs:
 * control/data message type encodings, fixed supply PDO decoding, sink
 * PDO encoding, and fixed Request Data Object (RDO) encoding. Only
 * fixed supplies are handled so far; other PDO kinds are recognized by
 * type and left to the policies to skip.
 *
 * The RDO object position is encoded in bits 30..28 with bit 31 zero,
 * which is valid for PD revision 2.0 and 3.x alike for the positions
 * 1..7 a 7-object capabilities message can carry.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Message.hpp>
#include <usbc/Units.hpp>

#include <cstdint>

namespace usbc {

// The numeric values of these two enums are the header message type
// encodings (num_data_objects == 0: control, > 0: data)
enum class control_message_type : std::uint8_t {
    good_crc      = 0x01,
    goto_min      = 0x02,
    accept        = 0x03,
    reject        = 0x04,
    ping          = 0x05,
    ps_rdy        = 0x06,
    get_source_cap = 0x07,
    get_sink_cap  = 0x08,
    dr_swap       = 0x09,
    pr_swap       = 0x0a,
    vconn_swap    = 0x0b,
    wait          = 0x0c,
    soft_reset    = 0x0d,
    not_supported = 0x10,
};

enum class data_message_type : std::uint8_t {
    source_capabilities = 0x01,
    request             = 0x02,
    bist                = 0x03,
    sink_capabilities   = 0x04,
    vendor_defined      = 0x0f,
};

namespace pdo {

enum class kind : std::uint8_t {
    fixed_supply       = 0b00, // bits 31..30
    battery            = 0b01,
    variable_supply    = 0b10,
    augmented          = 0b11,
};

constexpr kind kindOf(std::uint32_t pdo)
{
    return static_cast<kind>((pdo >> 30u) & 0x3u);
}

// Fixed supply source PDO fields
constexpr millivolt fixedVoltage(std::uint32_t pdo)
{
    return static_cast<millivolt>(((pdo >> 10u) & 0x3ffu) * 50u);
}

constexpr milliamp fixedMaxCurrent(std::uint32_t pdo)
{
    return static_cast<milliamp>((pdo & 0x3ffu) * 10u);
}

// Fixed sink PDO, for the Sink_Capabilities message
constexpr std::uint32_t makeFixedSink(millivolt voltage, milliamp operational_current)
{
    return (static_cast<std::uint32_t>(voltage / 50) << 10u) |
           static_cast<std::uint32_t>(operational_current / 10);
}

// Fixed supply source PDO, for the Source_Capabilities message; only
// the voltage/current fields - a plain always-on fixed supply
constexpr std::uint32_t makeFixedSource(millivolt voltage, milliamp maximum_current)
{
    return (static_cast<std::uint32_t>(voltage / 50) << 10u) |
           static_cast<std::uint32_t>(maximum_current / 10);
}

// Fixed supply RDO. no_usb_suspend is set: a pure power sink does not
// participate in USB suspend power rules
constexpr std::uint32_t makeFixedRequest(std::uint8_t object_position, milliamp operating_current,
                                         milliamp maximum_current, bool capability_mismatch)
{
    constexpr std::uint32_t no_usb_suspend = 1u << 24u;
    return (static_cast<std::uint32_t>(object_position & 0x7u) << 28u) |
           (capability_mismatch ? 1u << 26u : 0u) | no_usb_suspend |
           (static_cast<std::uint32_t>(operating_current / 10) << 10u) |
           static_cast<std::uint32_t>(maximum_current / 10);
}

// Fixed supply RDO fields, for the source evaluating a Request
constexpr std::uint8_t requestPosition(std::uint32_t rdo)
{
    return static_cast<std::uint8_t>((rdo >> 28u) & 0x7u);
}

constexpr milliamp requestOperatingCurrent(std::uint32_t rdo)
{
    return static_cast<milliamp>(((rdo >> 10u) & 0x3ffu) * 10u);
}

constexpr milliamp requestMaximumCurrent(std::uint32_t rdo)
{
    return static_cast<milliamp>((rdo & 0x3ffu) * 10u);
}

constexpr bool requestMismatch(std::uint32_t rdo)
{
    return ((rdo >> 26u) & 0x1u) != 0u;
}

} // namespace pdo

// The 16-bit extended message header, first two payload bytes of an
// extended message
struct extended_header {
    bool chunked               = false;
    std::uint8_t chunk_number  = 0;
    bool request_chunk         = false;
    std::uint16_t data_size    = 0; // bytes of the full extended message

    static constexpr extended_header decode(std::uint16_t raw)
    {
        return {
            .chunked       = ((raw >> 15u) & 0x1u) != 0u,
            .chunk_number  = static_cast<std::uint8_t>((raw >> 11u) & 0xfu),
            .request_chunk = ((raw >> 10u) & 0x1u) != 0u,
            .data_size     = static_cast<std::uint16_t>(raw & 0x1ffu),
        };
    }

    constexpr std::uint16_t encode() const
    {
        return static_cast<std::uint16_t>((chunked ? 1u << 15u : 0u) |
                                          ((chunk_number & 0xfu) << 11u) |
                                          (request_chunk ? 1u << 10u : 0u) | (data_size & 0x1ffu));
    }

    constexpr bool operator==(extended_header const&) const = default;
};

// Message classification and construction on top of pd_header
constexpr bool isControl(pd_header header, control_message_type type)
{
    return header.num_data_objects == 0 && !header.extended &&
           header.message_type == static_cast<std::uint8_t>(type);
}

constexpr bool isData(pd_header header, data_message_type type)
{
    return header.num_data_objects > 0 && !header.extended &&
           header.message_type == static_cast<std::uint8_t>(type);
}

} // namespace usbc
