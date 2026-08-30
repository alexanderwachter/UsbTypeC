/*
 * Writes a Graphviz DOT file for every state machine of the stack.
 * Usage: usbc-dotgen [output directory]
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mtl/StateMachineDot.hpp>
#include <usbc/ProtocolLayer.hpp>
#include <usbc/TypeC.hpp>

#include <filesystem>
#include <fstream>
#include <print>
#include <string>
#include <string_view>

namespace {

template<typename TABLE>
void write(std::filesystem::path const& directory, std::string_view name)
{
    auto const path = directory / (std::string{name} + ".dot");
    std::ofstream out{path};
    fsm::writeDot<TABLE>(out, name);
    std::print("{}\n", path.string());
}

} // namespace

int main(int argc, char* argv[])
{
    std::filesystem::path const directory = argc > 1 ? argv[1] : ".";
    std::filesystem::create_directories(directory);

    write<usbc::prl::tx_table>(directory, "prl_tx");
    write<usbc::tc::sink_table>(directory, "tc_sink");

    return 0;
}
