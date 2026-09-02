/*
 * Writes a Graphviz DOT file for every state machine of the stack.
 * Usage: usbc-dotgen [output directory]
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mtl/StateMachineDot.hpp>
#include <usbc/SinkPolicyEngine.hpp>
#include <usbc/SourcePolicyEngine.hpp>
#include <usbc/ProtocolLayer.hpp>
#include <usbc/TypeCDrp.hpp>
#include <usbc/TypeCSink.hpp>
#include <usbc/TypeCSource.hpp>

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
    write<usbc::tc::source_table>(directory, "tc_source");
    using drp_timing = usbc::default_drp_timing;
    write<usbc::tc::drp::table_for_t<drp_timing, usbc::drp_preference::none>>(directory,
                                                                              "tc_drp");
    write<usbc::tc::drp::table_for_t<drp_timing, usbc::drp_preference::source>>(
        directory, "tc_drp_try_src");
    write<usbc::tc::drp::table_for_t<drp_timing, usbc::drp_preference::sink>>(
        directory, "tc_drp_try_snk");
    write<usbc::pe::sink_table>(directory, "pe_sink");
    write<usbc::pe::source_table>(directory, "pe_source");

    return 0;
}
