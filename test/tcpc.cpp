/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mocks.hpp"

#include <usbc/Interfaces.hpp>
#include <usbc/SinkLoad.hpp>
#include <usbc/SourceSupply.hpp>
#include <usbc/Tcpc.hpp>
#include <usbc/Vbus.hpp>

#include <cstdint>
#include <optional>
#include <print>
#include <source_location>

namespace {

// mock_vbus moved to mocks.hpp, shared with the Type-C layer tests

// --- mock source power supply -----------------------------------------------
struct mock_supply {
    usbc::supply_callback callback = nullptr;
    void* context                  = nullptr;
    usbc::millivolt target_mv      = 5000;
    usbc::milliamp limit_ma        = 0;

    void setCallback(usbc::supply_callback cb, void* ctx)
    {
        callback = cb;
        context  = ctx;
    }
    bool setOutput(usbc::millivolt voltage, usbc::milliamp current_limit)
    {
        target_mv = voltage;
        limit_ma  = current_limit;
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

// --- mock sink load ---------------------------------------------------------
struct mock_sink_load {
    usbc::millivolt expected_mv = 5000;
    usbc::milliamp limit_ma     = 0;

    bool setLimit(usbc::millivolt expected_voltage, usbc::milliamp max_current)
    {
        expected_mv = expected_voltage;
        limit_ma    = max_current;
        return true;
    }
};

// --- compile-time checks ----------------------------------------------------
namespace compile_time {

static_assert(usbc::concepts::tcpc<mock_tcpc>);
static_assert(usbc::concepts::vconn_switch<mock_tcpc>);
static_assert(usbc::concepts::pd_transport<mock_tcpc>);

// the capability concepts are independent: a Type-C-only driver
// without VCONN and PD messaging still is a tcpc
struct no_vconn : mock_tcpc {
    bool setVconn(bool) = delete;
};
static_assert(usbc::concepts::tcpc<no_vconn> && !usbc::concepts::vconn_switch<no_vconn>);

// the flags operators are opt-in: enums without the trait stay plain
// (probed through templates - a non-dependent invalid expression in a
// requires-expression is a hard error, not false)
template<typename T>
concept has_flags_ops = requires(T v) {
    v | v;
    v |= v;
    any(v);
};
static_assert(has_flags_ops<usbc::alert_status>);
static_assert(has_flags_ops<usbc::receive_detect>);
static_assert(!has_flags_ops<usbc::cc_pull>);
static_assert(!has_flags_ops<usbc::sop_type>);
static_assert(usbc::concepts::vbus<mock_vbus>);
static_assert(!usbc::concepts::vbus<mock_tcpc>);
static_assert(!usbc::concepts::tcpc<mock_vbus>);

// hiding both transmit overloads must break the transport concept,
// while the core tcpc stays satisfied
struct no_transmit : mock_tcpc {
    bool transmit(usbc::pd_message const&) = delete;
};
static_assert(!usbc::concepts::pd_transport<no_transmit>);
static_assert(usbc::concepts::tcpc<no_transmit>);

// a wrong return type must break the concept, not just a missing member
struct void_set_cc : mock_tcpc {
    void setCc(usbc::cc_pull, usbc::rp_value) {}
};
static_assert(!usbc::concepts::tcpc<void_set_cc>);

// read functions must report failure; a bare value return is rejected
struct bare_cc_status : mock_tcpc {
    usbc::cc_status readCcStatus() { return line_state; }
};
static_assert(!usbc::concepts::tcpc<bare_cc_status>);

struct void_monitor : mock_vbus {
    void monitor(usbc::vbus_level) {}
};
static_assert(!usbc::concepts::vbus<void_monitor>);

static_assert(usbc::concepts::source_supply<mock_supply>);
static_assert(usbc::concepts::sink_load<mock_sink_load>);

// a current-only limit is not enough: the load must also know the
// input voltage to expect
struct current_only : mock_sink_load {
    bool setLimit(usbc::milliamp) { return true; }
};
static_assert(!usbc::concepts::sink_load<current_only>);

// a voltage-only setter is not enough: the contract's current limit
// must be programmable too
struct voltage_only : mock_supply {
    bool setOutput(usbc::millivolt) { return true; }
};
static_assert(!usbc::concepts::source_supply<voltage_only>);

// a driver written against the inheritance facade satisfies the
// concepts by construction, through the derived and the base type
struct virtual_driver : usbc::TcpcInterface, usbc::VconnInterface, usbc::PdTransportInterface {
    int transmitted = 0;

    void setAlertHandler(usbc::alert_callback, void*) override {}
    std::optional<usbc::alert_status> readAlert() override { return usbc::alert_status::none; }
    bool setCc(usbc::cc_pull, usbc::rp_value) override { return true; }
    std::optional<usbc::cc_status> readCcStatus() override { return std::nullopt; }
    bool setPlugOrientation(usbc::plug_orientation) override { return true; }
    bool sourceVbus(bool) override { return true; }
    bool sinkVbus(bool) override { return true; }
    bool setVconn(bool) override { return true; }
    bool setMessageHeaderInfo(usbc::message_header_info) override { return true; }
    bool setReceiveDetect(usbc::receive_detect) override { return true; }
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
    bool receive(usbc::pd_message&) override { return false; }
};
static_assert(usbc::concepts::tcpc<virtual_driver>);
static_assert(usbc::concepts::vconn_switch<virtual_driver>);
static_assert(usbc::concepts::pd_transport<virtual_driver>);

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
template<usbc::concepts::tcpc TCPC, usbc::concepts::vbus VBUS>
bool startSink(TCPC& tcpc, VBUS& vbus)
{
    return tcpc.setCc(usbc::cc_pull::rd, usbc::rp_value::usb_default) &&
           vbus.enable(true) &&
           tcpc.setMessageHeaderInfo({usbc::power_role::sink, usbc::data_role::ufp,
                                         usbc::pd_revision::rev_3_x}) &&
           tcpc.setReceiveDetect(usbc::receive_detect::sop | usbc::receive_detect::hard_reset);
}

template<usbc::concepts::source_supply SUPPLY>
bool requestOutput(SUPPLY& supply, usbc::millivolt voltage, usbc::milliamp current_limit)
{
    return supply.setOutput(voltage, current_limit);
}

template<usbc::concepts::sink_load LOAD>
bool applyLimit(LOAD& load, usbc::millivolt expected_voltage, usbc::milliamp max_current)
{
    return load.setLimit(expected_voltage, max_current);
}

template<typename TCPC>
    requires(usbc::concepts::tcpc<TCPC> && usbc::concepts::pd_transport<TCPC>)
std::optional<usbc::pd_message> fetchMessage(TCPC& tcpc)
{
    auto const alerts = tcpc.readAlert();
    if (!alerts || !any(*alerts & usbc::alert_status::message_received)) {
        return std::nullopt;
    }
    usbc::pd_message message;
    if (!tcpc.receive(message)) {
        return std::nullopt;
    }
    return message;
}

} // namespace

int tcpcTests()
{
    mock_tcpc tcpc;
    mock_vbus vbus;

    check(startSink(tcpc, vbus));
    check(tcpc.pull == usbc::cc_pull::rd);
    check(vbus.enabled);
    check(tcpc.header_info.power == usbc::power_role::sink);
    check(tcpc.header_info.data == usbc::data_role::ufp);
    check(any(tcpc.detect & usbc::receive_detect::sop) &&
          any(tcpc.detect & usbc::receive_detect::hard_reset) &&
          !any(tcpc.detect & usbc::receive_detect::sop_prime));

    // event-driven vbus flow: monitor a level, get notified on crossings
    struct vbus_events {
        int count = 0;
        bool met  = false;
    } events;
    vbus.setCallback(
        [](void* ctx, bool met) {
            auto& ev = *static_cast<vbus_events*>(ctx);
            ++ev.count;
            ev.met = met;
        },
        &events);

    check(vbus.monitor(usbc::vbus_level::safe5v));
    check(events.count == 1 && !events.met); // initial state reported unasked
    vbus.setVoltage(5000);                  // source attached
    check(events.count == 2 && events.met);
    vbus.setVoltage(5100);                  // still in range: no event
    check(events.count == 2);

    check(vbus.monitor(usbc::vbus_level::sink_disconnect)); // re-arm for detach
    check(events.count == 3 && !events.met);
    vbus.setVoltage(3000); // source removed
    check(events.count == 4 && events.met);

    // alert flow: driver callback fires, stack fetches the message
    bool alerted = false;
    tcpc.setAlertHandler([](void* ctx) { *static_cast<bool*>(ctx) = true; }, &alerted);
    usbc::pd_message incoming{
        .sop = usbc::sop_type::sop, .header = 0x1161, .payload_size = 4, .payload = {1, 2, 3, 4}};
    tcpc.injectMessage(incoming);
    check(alerted);

    auto const fetched = fetchMessage(tcpc);
    check(fetched.has_value());
    check(fetched && fetched->header == 0x1161 && fetched->payload_size == 4);
    check(!tcpc.pending_rx);                 // consumed
    check(!fetchMessage(tcpc).has_value()); // readAlert() cleared the pending alert

    // transmit both forms through the concept-constrained interface
    check(tcpc.transmit(incoming) && tcpc.transmit_count == 1);
    check(tcpc.transmit(usbc::transmit_signal::hard_reset) &&
          tcpc.last_signal == usbc::transmit_signal::hard_reset);

    // source supply: program the contract, PS_RDY trigger on settle
    mock_supply supply;
    bool at_target = false;
    supply.setCallback([](void* ctx, bool at) { *static_cast<bool*>(ctx) = at; }, &at_target);
    check(requestOutput(supply, 9000, 3000));
    check(supply.target_mv == 9000 && supply.limit_ma == 3000);
    check(!at_target); // still transitioning
    supply.settle(true);
    check(at_target);
    supply.settle(false); // lost regulation must be observable
    check(!at_target);

    // the inheritance facade dispatches virtually through the base types
    compile_time::virtual_driver driver;
    usbc::TcpcInterface& port = driver;
    usbc::PdTransportInterface& pd = driver;
    check(port.setCc(usbc::cc_pull::rd, usbc::rp_value::usb_default));
    check(pd.transmit(usbc::transmit_signal::hard_reset) && driver.transmitted == 1);

    // sink load: implicit contract, standby during transition, contract limit
    mock_sink_load load;
    check(applyLimit(load, 5000, 1500)); // Rp advertised 1.5 A
    check(load.expected_mv == 5000 && load.limit_ma == 1500);
    check(applyLimit(load, 9000, 500)); // iSnkStdby while transitioning
    check(applyLimit(load, 9000, 3000) && load.limit_ma == 3000); // after PS_RDY

    return failures;
}
