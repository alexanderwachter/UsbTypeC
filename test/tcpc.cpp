/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/SinkLoad.hpp>
#include <usbc/SourceSupply.hpp>
#include <usbc/Tcpc.hpp>
#include <usbc/Vbus.hpp>

#include <cstdint>
#include <optional>
#include <print>
#include <source_location>

namespace {

// --- mock TCPC driver (host/test implementation) ----------------------------
struct mock_tcpc {
    bool initialized              = false;
    usbc::alert_callback callback = nullptr;
    void* context                 = nullptr;
    usbc::alert_status alerts = usbc::alert_status::none;
    usbc::cc_pull pull            = usbc::cc_pull::open;
    usbc::rp_value rp             = usbc::rp_value::usb_default;
    usbc::cc_status line_state{usbc::cc_state::src_open, usbc::cc_state::src_open};
    usbc::plug_orientation orientation = usbc::plug_orientation::cc1;
    bool sourcing                 = false;
    bool sinking                  = false;
    bool vconn                    = false;
    usbc::receive_detect detect = usbc::receive_detect::none;
    usbc::message_header_info header_info{};
    usbc::pd_message last_transmitted{};
    std::uint8_t last_retry_count = 0;
    std::optional<usbc::transmit_signal> last_signal{};
    std::optional<usbc::pd_message> pending_rx{};

    bool init()
    {
        initialized = true;
        return true;
    }
    void set_alert_handler(usbc::alert_callback cb, void* ctx)
    {
        callback = cb;
        context  = ctx;
    }
    std::optional<usbc::alert_status> read_alert()
    {
        auto const pending = alerts;
        alerts             = usbc::alert_status::none;
        return pending;
    }
    bool set_cc(usbc::cc_pull p, usbc::rp_value current)
    {
        pull = p;
        rp   = current;
        return true;
    }
    std::optional<usbc::cc_status> read_cc_status() { return line_state; }
    bool set_plug_orientation(usbc::plug_orientation o)
    {
        orientation = o;
        return true;
    }
    bool source_vbus(bool enable)
    {
        sourcing = enable;
        return true;
    }
    bool sink_vbus(bool enable)
    {
        sinking = enable;
        return true;
    }
    bool set_vconn(bool enable)
    {
        vconn = enable;
        return true;
    }
    bool set_message_header_info(usbc::message_header_info info)
    {
        header_info = info;
        return true;
    }
    bool set_receive_detect(usbc::receive_detect d)
    {
        detect = d;
        return true;
    }
    bool transmit(usbc::pd_message const& message, std::uint8_t retry_count)
    {
        last_transmitted = message;
        last_retry_count = retry_count;
        return true;
    }
    bool transmit(usbc::transmit_signal signal)
    {
        last_signal = signal;
        return true;
    }
    bool receive(usbc::pd_message& out)
    {
        if (!pending_rx) {
            return false;
        }
        out = *pending_rx;
        pending_rx.reset();
        return true;
    }

    // test helper: a message arrives and the driver raises its alert
    void inject_message(usbc::pd_message const& message)
    {
        pending_rx = message;
        alerts |= usbc::alert_status::message_received;
        if (callback != nullptr) {
            callback(context);
        }
    }
};

// --- mock VBUS driver -------------------------------------------------------
struct mock_vbus {
    bool enabled                 = false;
    usbc::vbus_callback callback = nullptr;
    void* context                = nullptr;
    std::optional<usbc::vbus_level> monitored{};
    std::int32_t voltage_mv = 0;
    bool reported_met       = false;

    bool enable(bool e)
    {
        enabled = e;
        return true;
    }
    void set_callback(usbc::vbus_callback cb, void* ctx)
    {
        callback = cb;
        context  = ctx;
    }
    bool monitor(usbc::vbus_level level)
    {
        monitored = level;
        report(); // contract: current condition state as soon as known
        return true;
    }
    bool discharge(bool enable)
    {
        if (enable) {
            set_voltage(0);
        }
        return true;
    }

    // test helpers: simulate the comparator
    bool met() const
    {
        switch (*monitored) {
        case usbc::vbus_level::safe0v: return voltage_mv <= 800;
        case usbc::vbus_level::safe5v: return voltage_mv >= 4750 && voltage_mv <= 5500;
        case usbc::vbus_level::sink_disconnect: return voltage_mv < 3670;
        case usbc::vbus_level::sink_disconnect_pd: return voltage_mv < 4000;
        }
        return false;
    }
    void report()
    {
        reported_met = met();
        if (callback != nullptr) {
            callback(context, reported_met);
        }
    }
    void set_voltage(std::int32_t mv)
    {
        voltage_mv = mv;
        if (monitored && met() != reported_met) {
            report();
        }
    }
};

// --- mock source power supply -----------------------------------------------
struct mock_supply {
    usbc::supply_callback callback = nullptr;
    void* context                  = nullptr;
    usbc::millivolt target_mv      = 5000;
    usbc::milliamp limit_ma        = 0;

    void set_callback(usbc::supply_callback cb, void* ctx)
    {
        callback = cb;
        context  = ctx;
    }
    bool set_output(usbc::millivolt voltage, usbc::milliamp current_limit)
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

    bool set_limit(usbc::millivolt expected_voltage, usbc::milliamp max_current)
    {
        expected_mv = expected_voltage;
        limit_ma    = max_current;
        return true;
    }
};

// --- compile-time checks ----------------------------------------------------
namespace compile_time {

static_assert(usbc::concepts::tcpc<mock_tcpc>);

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

// hiding both transmit overloads must break the concept
struct no_transmit : mock_tcpc {
    bool transmit(usbc::pd_message const&, std::uint8_t) = delete;
};
static_assert(!usbc::concepts::tcpc<no_transmit>);

// a wrong return type must break the concept, not just a missing member
struct void_init : mock_tcpc {
    void init() {}
};
static_assert(!usbc::concepts::tcpc<void_init>);

// read functions must report failure; a bare value return is rejected
struct bare_cc_status : mock_tcpc {
    usbc::cc_status read_cc_status() { return line_state; }
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
    bool set_limit(usbc::milliamp) { return true; }
};
static_assert(!usbc::concepts::sink_load<current_only>);

// a voltage-only setter is not enough: the contract's current limit
// must be programmable too
struct voltage_only : mock_supply {
    bool set_output(usbc::millivolt) { return true; }
};
static_assert(!usbc::concepts::source_supply<voltage_only>);

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
bool start_sink(TCPC& tcpc, VBUS& vbus)
{
    return tcpc.init() && tcpc.set_cc(usbc::cc_pull::rd, usbc::rp_value::usb_default) &&
           vbus.enable(true) &&
           tcpc.set_message_header_info({usbc::power_role::sink, usbc::data_role::ufp,
                                         usbc::pd_revision::rev_3_x}) &&
           tcpc.set_receive_detect(usbc::receive_detect::sop | usbc::receive_detect::hard_reset);
}

template<usbc::concepts::source_supply SUPPLY>
bool request_output(SUPPLY& supply, usbc::millivolt voltage, usbc::milliamp current_limit)
{
    return supply.set_output(voltage, current_limit);
}

template<usbc::concepts::sink_load LOAD>
bool apply_limit(LOAD& load, usbc::millivolt expected_voltage, usbc::milliamp max_current)
{
    return load.set_limit(expected_voltage, max_current);
}

template<usbc::concepts::tcpc TCPC>
std::optional<usbc::pd_message> fetch_message(TCPC& tcpc)
{
    auto const alerts = tcpc.read_alert();
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

int tcpc_tests()
{
    mock_tcpc tcpc;
    mock_vbus vbus;

    check(start_sink(tcpc, vbus));
    check(tcpc.initialized);
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
    vbus.set_callback(
        [](void* ctx, bool met) {
            auto& ev = *static_cast<vbus_events*>(ctx);
            ++ev.count;
            ev.met = met;
        },
        &events);

    check(vbus.monitor(usbc::vbus_level::safe5v));
    check(events.count == 1 && !events.met); // initial state reported unasked
    vbus.set_voltage(5000);                  // source attached
    check(events.count == 2 && events.met);
    vbus.set_voltage(5100);                  // still in range: no event
    check(events.count == 2);

    check(vbus.monitor(usbc::vbus_level::sink_disconnect)); // re-arm for detach
    check(events.count == 3 && !events.met);
    vbus.set_voltage(3000); // source removed
    check(events.count == 4 && events.met);

    // alert flow: driver callback fires, stack fetches the message
    bool alerted = false;
    tcpc.set_alert_handler([](void* ctx) { *static_cast<bool*>(ctx) = true; }, &alerted);
    usbc::pd_message incoming{
        .sop = usbc::sop_type::sop, .header = 0x1161, .payload_size = 4, .payload = {1, 2, 3, 4}};
    tcpc.inject_message(incoming);
    check(alerted);

    auto const fetched = fetch_message(tcpc);
    check(fetched.has_value());
    check(fetched && fetched->header == 0x1161 && fetched->payload_size == 4);
    check(!tcpc.pending_rx);                 // consumed
    check(!fetch_message(tcpc).has_value()); // read_alert() cleared the pending alert

    // transmit both forms through the concept-constrained interface
    check(tcpc.transmit(incoming, 2) && tcpc.last_retry_count == 2);
    check(tcpc.transmit(usbc::transmit_signal::hard_reset) &&
          tcpc.last_signal == usbc::transmit_signal::hard_reset);

    // source supply: program the contract, PS_RDY trigger on settle
    mock_supply supply;
    bool at_target = false;
    supply.set_callback([](void* ctx, bool at) { *static_cast<bool*>(ctx) = at; }, &at_target);
    check(request_output(supply, 9000, 3000));
    check(supply.target_mv == 9000 && supply.limit_ma == 3000);
    check(!at_target); // still transitioning
    supply.settle(true);
    check(at_target);
    supply.settle(false); // lost regulation must be observable
    check(!at_target);

    // sink load: implicit contract, standby during transition, contract limit
    mock_sink_load load;
    check(apply_limit(load, 5000, 1500)); // Rp advertised 1.5 A
    check(load.expected_mv == 5000 && load.limit_ma == 1500);
    check(apply_limit(load, 9000, 500)); // iSnkStdby while transitioning
    check(apply_limit(load, 9000, 3000) && load.limit_ma == 3000); // after PS_RDY

    return failures;
}
