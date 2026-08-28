/*
 * Plain integer unit aliases, to be replaced by mtl unit types once
 * mtl/Units.hpp matures.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <cstdint>

namespace usbc {

using millivolt = std::int32_t;
using milliamp  = std::int32_t;

} // namespace usbc
