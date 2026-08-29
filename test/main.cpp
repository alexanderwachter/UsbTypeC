/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <print>

int tcpc_tests();
int protocol_layer_tests();
int type_c_tests();

int main(int argc, const char* argv[]) {
    int const failures = tcpc_tests() + protocol_layer_tests() + type_c_tests();
    if (failures != 0) {
        std::print("{} check(s) FAILED\n", failures);
        return 1;
    }
    std::print("all checks passed\n");
    return 0;
}
