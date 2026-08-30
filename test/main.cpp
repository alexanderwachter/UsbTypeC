/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <print>

int tcpcTests();
int protocolLayerTests();
int typeCTests();

int main(int argc, const char* argv[]) {
    int const failures = tcpcTests() + protocolLayerTests() + typeCTests();
    if (failures != 0) {
        std::print("{} check(s) FAILED\n", failures);
        return 1;
    }
    std::print("all checks passed\n");
    return 0;
}
