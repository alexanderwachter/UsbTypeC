/*
 * The policies a port's application provides: the sink's contract
 * selection over the source's capabilities (concepts::sink_policy) and
 * the source's evaluation of a Request (concepts::source_policy), with
 * the capability and contract types they exchange with the engines.
 * PowerPolicy and RequestPolicy are the defaults.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/interface/Power.hpp>
#include <usbc/spec/Pdo.hpp>
#include <usbc/spec/Units.hpp>

#include <algorithm>
#include <concepts>
#include <cstdint>
#include <optional>
#include <span>

namespace usbc {

// One contract the sink side can accept, and the Sink_Capabilities
// content (also answered by a DRP asked while sourcing)
struct sink_capability {
    millivolt voltage;
    milliamp current;
};

// A policy's answer: which source PDO to request and at what current
struct contract_request {
    std::uint8_t position = 0; // 1-based object position in the source capabilities
    millivolt voltage = 5000;
    milliamp operating_current = 0;
    milliamp maximum_current = 0;
    bool mismatch = false;
};

namespace concepts {

template<typename T>
concept sink_policy = requires(
    T policy,
    std::span<std::uint32_t const> source_capabilities,
    std::span<sink_capability const> capabilities
) {
    {
        policy.select(source_capabilities, capabilities)
    } -> std::same_as<std::optional<contract_request>>;
};

template<typename T>
concept source_policy =
    requires(T policy, std::uint32_t rdo, std::span<std::uint32_t const> capabilities) {
        { policy.evaluate(rdo, capabilities) } -> std::same_as<std::optional<supply_target>>;
    };

} // namespace concepts

// Default selection policy: only source PDOs whose voltage the sink
// lists are considered, with the current capped by the sink capability
// and the power budget. A contract must deliver min_power; the lowest
// voltage delivering max_power wins. When no PDO reaches max_power the
// one with the most power wins, equal power prefers the lower voltage.
class PowerPolicy {
public:
    constexpr PowerPolicy(milliwatt min_power, milliwatt max_power)
        : min_power_(min_power), max_power_(max_power)
    {
    }

    constexpr std::optional<contract_request> select(
        std::span<std::uint32_t const> source_capabilities,
        std::span<sink_capability const> capabilities
    ) const
    {
        std::optional<contract_request> best;
        milliwatt best_power = 0;
        for (std::uint8_t index = 0; index < source_capabilities.size(); ++index) {
            auto const object = source_capabilities[index];
            if (pdo::kindOf(object) != pdo::kind::fixed_supply) {
                continue;
            }
            auto const voltage = pdo::fixedVoltage(object);
            auto const accepted = std::find_if(
                capabilities.begin(),
                capabilities.end(),
                [voltage](sink_capability capability) { return capability.voltage == voltage; }
            );
            if (accepted == capabilities.end()) {
                continue; // the sink cannot take this voltage
            }
            auto const wanted =
                static_cast<milliamp>((static_cast<std::int64_t>(max_power_) * 1000) / voltage);
            auto const current =
                std::min({pdo::fixedMaxCurrent(object), accepted->current, wanted});
            auto const power =
                static_cast<milliwatt>((static_cast<std::int64_t>(voltage) * current) / 1000);
            if (power < min_power_) {
                continue;
            }
            if (!best || power > best_power || (power == best_power && voltage < best->voltage)) {
                best_power = power;
                best = contract_request{
                    .position = static_cast<std::uint8_t>(index + 1),
                    .voltage = voltage,
                    .operating_current = current,
                    .maximum_current = current,
                    .mismatch = false
                };
            }
        }
        return best;
    }

private:
    milliwatt min_power_;
    milliwatt max_power_;
};
static_assert(concepts::sink_policy<PowerPolicy>);

// Default evaluation policy: a Request is granted when it names a
// fixed-supply PDO of the capabilities and its operating current stays
// within that PDO's maximum
class RequestPolicy {
public:
    constexpr std::optional<supply_target>
    evaluate(std::uint32_t rdo, std::span<std::uint32_t const> capabilities) const
    {
        auto const position = pdo::requestPosition(rdo);
        if (position == 0 || position > capabilities.size()) {
            return std::nullopt;
        }
        auto const object = capabilities[position - 1u];
        if (pdo::kindOf(object) != pdo::kind::fixed_supply) {
            return std::nullopt;
        }
        auto const current = pdo::requestOperatingCurrent(rdo);
        if (current > pdo::fixedMaxCurrent(object)) {
            return std::nullopt;
        }
        return supply_target{.voltage = pdo::fixedVoltage(object), .current = current};
    }
};
static_assert(concepts::source_policy<RequestPolicy>);

} // namespace usbc
