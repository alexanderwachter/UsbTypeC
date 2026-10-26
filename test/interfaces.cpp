/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mocks.hpp"

#include <usbc/interface/Cc.hpp>
#include <usbc/interface/PdPhy.hpp>
#include <usbc/interface/Power.hpp>
#include <usbc/interface/Vbus.hpp>
#include <usbc/interface/Virtual.hpp>

#include <mtl/StateMachine.hpp>

#include <cstdint>
#include <optional>
#include <print>
#include <source_location>

namespace {

// the hardware block doubles live in mocks.hpp, shared with the layer tests

// --- mock source power supply -----------------------------------------------
struct mock_supply : usbc::SourceSupply<mock_supply> {
    usbc::supply_ready_callback callback = nullptr;
    void* context = nullptr;
    usbc::millivolt target_mv = 5000;
    usbc::milliamp limit_ma = 0;

    void setReadyCallback(usbc::supply_ready_callback cb, void* ctx)
    {
        callback = cb;
        context = ctx;
    }
    bool setOutput(usbc::millivolt voltage, usbc::milliamp current_limit)
    {
        target_mv = voltage;
        limit_ma = current_limit;
        return true;
    }

    // test helper: the regulator reaches (or loses) the target
    void settle(bool at_target)
    {
        if (callback != nullptr) {
            callback(context, at_target);
        }
    }
};

// --- mock sink power client -------------------------------------------------
struct mock_sink_load {
    usbc::millivolt expected_mv = 5000;
    usbc::milliamp limit_ma = 0;

    bool setLimit(usbc::millivolt expected_voltage, usbc::milliamp max_current)
    {
        expected_mv = expected_voltage;
        limit_ma = max_current;
        return true;
    }
    void onContract(usbc::millivolt, usbc::milliamp) {}
    void onContractLost() {}
};

// --- compile-time checks ----------------------------------------------------
namespace compile_time {

static_assert(usbc::concepts::cc<mock_cc>);
static_assert(usbc::concepts::pd_phy<mock_phy>);
static_assert(usbc::concepts::vbus<mock_vbus>);
static_assert(!usbc::concepts::cc<mock_phy>);
static_assert(!usbc::concepts::pd_phy<mock_cc>);
static_assert(!usbc::concepts::vbus<mock_cc>);

// the flags operators are opt-in: enums without the trait stay plain
// (probed through templates - a non-dependent invalid expression in a
// requires-expression is a hard error, not false)
template<typename T>
concept has_flags_ops = requires(T v) {
    v | v;
    v |= v;
    any(v);
};
static_assert(has_flags_ops<usbc::receive_detect>);
static_assert(!has_flags_ops<usbc::cc_pull>);
static_assert(!has_flags_ops<usbc::sop_type>);
static_assert(!has_flags_ops<usbc::transmit_outcome>);

// hiding both transmit overloads must break the PHY concept
struct no_transmit : mock_phy {
    bool transmit(usbc::pd_message const&) = delete;
};
static_assert(!usbc::concepts::pd_phy<no_transmit>);

// a wrong return type must break the concept, not just a missing member
struct void_set_rp : fsm::observing<void_set_rp> {
    void notifyEntry(usbc::cc_termination) {}
    void setRp(usbc::rp_value) {}
    void setCallback(usbc::cc_callback, void*) {}
    std::optional<usbc::cc_status> readCcStatus() { return std::nullopt; }
};
static_assert(!usbc::concepts::cc<void_set_rp>);

// read functions must report failure; a bare value return is rejected
struct bare_cc_status : fsm::observing<bare_cc_status> {
    void notifyEntry(usbc::cc_termination) {}
    bool setRp(usbc::rp_value) { return true; }
    void setCallback(usbc::cc_callback, void*) {}
    usbc::cc_status readCcStatus() { return {}; }
};
static_assert(!usbc::concepts::cc<bare_cc_status>);

// the hooks without the fsm::observing base: the machine would never
// call them, so the concept refuses the class
struct hooks_without_base {
    void notifyEntry(usbc::cc_termination) {}
    bool setRp(usbc::rp_value) { return true; }
    void setCallback(usbc::cc_callback, void*) {}
    std::optional<usbc::cc_status> readCcStatus() { return std::nullopt; }
};
static_assert(!usbc::concepts::cc<hooks_without_base>);

struct vbus_without_base {
    void notifyEntry(usbc::vbus_power) {}
    void notifyEntry(usbc::vbus_level) {}
    bool check(usbc::vbus_at_safe0v) { return false; }
    bool check(usbc::vbus_at_safe5v) { return false; }
    void setCallback(usbc::vbus_callback, void*) {}
};
static_assert(!usbc::concepts::vbus<vbus_without_base>);

// a question answered with nothing is no answer
struct silent_check : fsm::observing<silent_check> {
    void notifyEntry(usbc::vbus_power) {}
    void notifyEntry(usbc::vbus_level) {}
    void check(usbc::vbus_at_safe0v) {}
    bool check(usbc::vbus_at_safe5v) { return false; }
    void setCallback(usbc::vbus_callback, void*) {}
};
static_assert(!usbc::concepts::vbus<silent_check>);

static_assert(usbc::concepts::source_supply<mock_supply>);
static_assert(usbc::concepts::sink_power_client<mock_sink_load>);

// a current-only limit is not enough: the load must also know the
// input voltage to expect
struct current_only : mock_sink_load {
    bool setLimit(usbc::milliamp) { return true; }
};
static_assert(!usbc::concepts::sink_power_client<current_only>);

// a voltage-only setter is not enough: the contract's current limit
// must be programmable too
struct voltage_only : mock_supply {
    bool setOutput(usbc::millivolt) { return true; }
};
static_assert(!usbc::concepts::source_supply_client<voltage_only>);

// a driver written against the inheritance facade: the interface types
// satisfy the concepts (interface/Virtual.hpp asserts it), the driver
// fills in the virtuals
struct virtual_driver : usbc::CcInterface, usbc::PdPhyInterface {
    int transmitted = 0;
    usbc::rp_value rp = usbc::rp_value::usb_default;

    void notifyEntry(usbc::cc_termination) override {}
    bool setRp(usbc::rp_value value) override
    {
        rp = value;
        return true;
    }
    void setCallback(usbc::cc_callback, void*) override {}
    std::optional<usbc::cc_status> readCcStatus() override { return std::nullopt; }
    void setCallbacks(usbc::pd_phy_callbacks const&, void*) override {}
    bool setMessageHeaderInfo(usbc::message_header_info) override { return true; }
    bool setReceiveDetect(usbc::receive_detect) override { return true; }
    bool selectCcLine(usbc::plug_orientation) override { return true; }
    bool transmit(usbc::pd_message const&) override
    {
        ++transmitted;
        return true;
    }
    bool transmit(usbc::transmit_signal) override
    {
        ++transmitted;
        return true;
    }
};

} // namespace compile_time

// --- runtime checks ---------------------------------------------------------
int failures = 0;

void check(bool condition, std::source_location location = std::source_location::current())
{
    if (!condition) {
        std::print("check failed at {}:{}\n", location.file_name(), location.line());
        ++failures;
    }
}

// The stack-side usage pattern, written against the concepts only
template<usbc::concepts::cc CC, usbc::concepts::pd_phy PHY>
bool startSink(CC& cc, PHY& phy)
{
    cc.notifyEntry(usbc::cc_termination{usbc::cc_pull::rd});
    return cc.setRp(usbc::rp_value::usb_default) &&
           phy.setMessageHeaderInfo(
               {usbc::power_role::sink, usbc::data_role::ufp, usbc::pd_revision::rev_3_x}
           ) &&
           phy.setReceiveDetect(usbc::receive_detect::sop | usbc::receive_detect::hard_reset);
}

template<usbc::concepts::source_supply_client SUPPLY>
bool requestOutput(SUPPLY& supply, usbc::millivolt voltage, usbc::milliamp current_limit)
{
    return supply.setOutput(voltage, current_limit);
}

template<usbc::concepts::sink_power_client LOAD>
bool applyLimit(LOAD& load, usbc::millivolt expected_voltage, usbc::milliamp max_current)
{
    return load.setLimit(expected_voltage, max_current);
}

// What a protocol layer hears from the PHY it holds
struct phy_events {
    int transmitted = 0;
    usbc::transmit_outcome outcome = usbc::transmit_outcome::failed;
    int received = 0;
    std::uint16_t header = 0;
    int hard_resets = 0;
};

constexpr usbc::pd_phy_callbacks phy_listener{
    .transmitted =
        [](void* ctx, usbc::transmit_outcome outcome) {
            auto& events = *static_cast<phy_events*>(ctx);
            ++events.transmitted;
            events.outcome = outcome;
        },
    .received =
        [](void* ctx, usbc::pd_message const& message) {
            auto& events = *static_cast<phy_events*>(ctx);
            ++events.received;
            events.header = message.header;
        },
    .hard_reset_received = [](void* ctx) { ++static_cast<phy_events*>(ctx)->hard_resets; },
};

} // namespace

int interfaceTests()
{
    mock_cc cc;
    mock_phy phy;
    mock_vbus vbus;

    check(startSink(cc, phy));
    check(cc.pull == usbc::cc_pull::rd);
    check(phy.header_info.power == usbc::power_role::sink);
    check(phy.header_info.data == usbc::data_role::ufp);
    check(
        any(phy.detect & usbc::receive_detect::sop) &&
        any(phy.detect & usbc::receive_detect::hard_reset) &&
        !any(phy.detect & usbc::receive_detect::sop_prime)
    );

    // CC flow: the line's changes come through the callback, with the
    // status
    struct cc_events {
        int count = 0;
        usbc::cc_status status{};
    } lines;
    cc.setCallback(
        [](void* ctx, usbc::cc_status status) {
            auto& ev = *static_cast<cc_events*>(ctx);
            ++ev.count;
            ev.status = status;
        },
        &lines
    );
    cc.report({usbc::cc_state::snk_power_1a5, usbc::cc_state::snk_open});
    check(lines.count == 1 && lines.status.cc1 == usbc::cc_state::snk_power_1a5);
    check(cc.readCcStatus() == lines.status); // one measurement, the same line

    // event-driven vbus flow: the state's level arms the condition, the
    // report names the level; the switch position is the state's too
    struct vbus_events {
        int count = 0;
        usbc::vbus_level level = usbc::vbus_level::unwatched;
        bool met = false;
    } events;
    vbus.setCallback(
        [](void* ctx, usbc::vbus_level level, bool met) {
            auto& ev = *static_cast<vbus_events*>(ctx);
            ++ev.count;
            ev.level = level;
            ev.met = met;
        },
        &events
    );

    vbus.notifyEntry(usbc::vbus_level::safe5v);
    check(events.count == 1 && !events.met); // initial state reported unasked
    check(events.level == usbc::vbus_level::safe5v);
    vbus.setVoltage(5000); // source attached
    check(events.count == 2 && events.met);
    vbus.setVoltage(5100); // still in range: no event
    check(events.count == 2);
    check(vbus.check(usbc::vbus_at_safe5v{}) && !vbus.check(usbc::vbus_at_safe0v{}));

    vbus.notifyEntry(usbc::vbus_level::sink_disconnect); // re-arm for detach
    check(events.count == 3 && !events.met && events.level == usbc::vbus_level::sink_disconnect);
    vbus.setVoltage(3000); // source removed
    check(events.count == 4 && events.met);

    vbus.notifyEntry(usbc::vbus_power{usbc::vbus_path::sink});
    check(vbus.sinking() && !vbus.sourcing() && !vbus.discharging());
    vbus.notifyEntry(usbc::vbus_power{usbc::vbus_path::safe0v});
    check(vbus.discharging() && !vbus.sinking());

    // PHY flow: the holder hears what arrives and the outcomes; nobody
    // holding it hears nothing
    phy_events heard;
    usbc::pd_message incoming{
        .sop = usbc::sop_type::sop,
        .header = 0x1161,
        .payload_size = 4,
        .payload = {1, 2, 3, 4}
    };
    phy.injectMessage(incoming);
    check(heard.received == 0 && !phy.claimed());
    phy.setCallbacks(phy_listener, &heard);
    check(phy.claimed());
    phy.injectMessage(incoming);
    check(heard.received == 1 && heard.header == 0x1161);
    phy.transmitted(usbc::transmit_outcome::discarded);
    check(heard.transmitted == 1 && heard.outcome == usbc::transmit_outcome::discarded);
    phy.hardResetReceived();
    check(heard.hard_resets == 1);
    phy.setCallbacks({}, nullptr);
    phy.injectMessage(incoming);
    check(heard.received == 1 && !phy.claimed());

    // transmit both forms through the concept-constrained interface
    check(phy.transmit(incoming) && phy.transmit_count == 1);
    check(
        phy.transmit(usbc::transmit_signal::hard_reset) &&
        phy.last_signal == usbc::transmit_signal::hard_reset
    );
    check(phy.selectCcLine(usbc::plug_orientation::cc2) && phy.line == usbc::plug_orientation::cc2);

    // source supply: program the contract, PS_RDY trigger on settle
    mock_supply supply;
    bool at_target = false;
    supply.setReadyCallback([](void* ctx, bool at) { *static_cast<bool*>(ctx) = at; }, &at_target);
    check(requestOutput(supply, 9000, 3000));
    check(supply.target_mv == 9000 && supply.limit_ma == 3000);
    check(!at_target); // still transitioning
    supply.settle(true);
    check(at_target);
    supply.settle(false); // lost regulation must be observable
    check(!at_target);

    // the inheritance facade dispatches virtually through the base types
    compile_time::virtual_driver driver;
    usbc::CcInterface& port = driver;
    usbc::PdPhyInterface& pd = driver;
    check(port.setRp(usbc::rp_value::p_3a0) && driver.rp == usbc::rp_value::p_3a0);
    check(pd.transmit(usbc::transmit_signal::hard_reset) && driver.transmitted == 1);

    // sink load: implicit contract, standby during transition, contract limit
    mock_sink_load load;
    check(applyLimit(load, 5000, 1500)); // Rp advertised 1.5 A
    check(load.expected_mv == 5000 && load.limit_ma == 1500);
    check(applyLimit(load, 9000, 500));                           // iSnkStdby while transitioning
    check(applyLimit(load, 9000, 3000) && load.limit_ma == 3000); // after PS_RDY

    return failures;
}
