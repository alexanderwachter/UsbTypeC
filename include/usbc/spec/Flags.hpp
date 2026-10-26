/*
 * Bitwise operators for enum classes used as flag sets. An enum opts
 * in by specializing is_flags; its enumerators must be single bits and
 * the empty set must be value 0.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <type_traits>

namespace usbc {

template<typename>
struct is_flags : std::false_type {};

template<typename T>
inline constexpr bool is_flags_v = is_flags<T>::value;

namespace concepts {

template<typename T>
concept flags = std::is_enum_v<T> && is_flags_v<T>;

} // namespace concepts

template<concepts::flags FLAGS>
constexpr FLAGS operator|(FLAGS lhs, FLAGS rhs)
{
    using underlying = std::underlying_type_t<FLAGS>;
    return static_cast<FLAGS>(static_cast<underlying>(lhs) | static_cast<underlying>(rhs));
}

template<concepts::flags FLAGS>
constexpr FLAGS operator&(FLAGS lhs, FLAGS rhs)
{
    using underlying = std::underlying_type_t<FLAGS>;
    return static_cast<FLAGS>(static_cast<underlying>(lhs) & static_cast<underlying>(rhs));
}

template<concepts::flags FLAGS>
constexpr FLAGS operator~(FLAGS flags)
{
    using underlying = std::underlying_type_t<FLAGS>;
    return static_cast<FLAGS>(static_cast<underlying>(~static_cast<underlying>(flags)));
}

template<concepts::flags FLAGS>
constexpr FLAGS& operator|=(FLAGS& lhs, FLAGS rhs)
{
    return lhs = lhs | rhs;
}

template<concepts::flags FLAGS>
constexpr FLAGS& operator&=(FLAGS& lhs, FLAGS rhs)
{
    return lhs = lhs & rhs;
}

template<concepts::flags FLAGS>
constexpr bool any(FLAGS flags)
{
    return flags != FLAGS{};
}

} // namespace usbc
