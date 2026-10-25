/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <usbc/Cc.hpp>
#include <usbc/PdPhy.hpp>
#include <usbc/Spec.hpp>
#include <usbc/Vbus.hpp>

#include <mtl/StateMachine.hpp>

#include <cstdint>
#include <optional>

// CC block test double: presents the pull the connection states
// annotate and reports the line a test sets
struct mock_cc : fsm::observing<mock_cc> {
    usbc::cc_pull pull = usbc::cc_pull::open;
    usbc::rp_value rp = usbc::rp_value::usb_default;
    usbc::cc_status line_state{usbc::cc_state::src_open, usbc::cc_state::src_open};
    usbc::cc_callback callback = nullptr;
    void* context = nullptr;

    void notifyEntry(usbc::cc_termination termination) { pull = termination.pull; }
    bool setRp(usbc::rp_value value)
    {
        rp = value;
        return true;
    }
    void setCallback(usbc::cc_callback cb, void* ctx)
    {
        callback = cb;
        context = ctx;
    }
    std::optional<usbc::cc_status> readCcStatus() { return line_state; }

    // test helpers: the driver reports the line as it is / as it became
    void report()
    {
        if (callback != nullptr) {
            callback(context, line_state);
        }
    }
    void report(usbc::cc_status status)
    {
        line_state = status;
        report();
    }
};

// PD PHY test double: records what the protocol layer configures and
// transmits, reports to whoever holds it
struct mock_phy {
    usbc::pd_phy_callbacks callbacks{};
    void* context = nullptr;
    usbc::message_header_info header_info{};
    usbc::receive_detect detect = usbc::receive_detect::none;
    std::optional<usbc::plug_orientation> line{};
    usbc::pd_message last_transmitted{};
    int transmit_count = 0;
    bool accept_transmit = true;
    std::optional<usbc::transmit_signal> last_signal{};

    void setCallbacks(usbc::pd_phy_callbacks const& cbs, void* ctx)
    {
        callbacks = cbs;
        context = ctx;
    }
    bool setMessageHeaderInfo(usbc::message_header_info info)
    {
        header_info = info;
        return true;
    }
    bool setReceiveDetect(usbc::receive_detect d)
    {
        detect = d;
        return true;
    }
    bool selectCcLine(usbc::plug_orientation l)
    {
        line = l;
        return true;
    }
    bool transmit(usbc::pd_message const& message)
    {
        last_transmitted = message;
        ++transmit_count;
        return accept_transmit;
    }
    bool transmit(usbc::transmit_signal signal)
    {
        last_signal = signal;
        return true;
    }

    // test helpers: the PHY's reports; nobody holding it hears nothing
    bool claimed() const { return callbacks.received != nullptr; }
    void injectMessage(usbc::pd_message const& message)
    {
        if (callbacks.received != nullptr) {
            callbacks.received(context, message);
        }
    }
    void transmitted(usbc::transmit_outcome outcome)
    {
        if (callbacks.transmitted != nullptr) {
            callbacks.transmitted(context, outcome);
        }
    }
    void hardResetReceived()
    {
        if (callbacks.hard_reset_received != nullptr) {
            callbacks.hard_reset_received(context);
        }
    }
};

// VBUS block test double simulating a comparator behind the switch
// position the connection states annotate
struct mock_vbus : fsm::observing<mock_vbus> {
    usbc::vbus_path path = usbc::vbus_path::open;
    usbc::vbus_callback callback = nullptr;
    void* context = nullptr;
    std::optional<usbc::vbus_level> monitored{};
    std::int32_t voltage_mv = 0;
    bool reported_met = false;

    void notifyEntry(usbc::vbus_power power) { path = power.path; }
    void notifyEntry(usbc::vbus_level level)
    {
        monitored = level;
        if (level != usbc::vbus_level::unwatched) {
            report(); // contract: current condition state as soon as known
        }
    }
    bool check(usbc::vbus_at_safe0v) const { return meets(usbc::vbus_level::safe0v); }
    bool check(usbc::vbus_at_safe5v) const { return meets(usbc::vbus_level::safe5v); }
    void setCallback(usbc::vbus_callback cb, void* ctx)
    {
        callback = cb;
        context = ctx;
    }

    // the switch position, by name
    bool sinking() const { return path == usbc::vbus_path::sink; }
    bool sourcing() const { return path == usbc::vbus_path::safe5v; }
    // Discharging takes time and completes asynchronously: the test
    // drives the voltage explicitly
    bool discharging() const { return path == usbc::vbus_path::safe0v; }

    // the comparator, asked for one level
    bool meets(usbc::vbus_level level) const
    {
        switch (level) {
        case usbc::vbus_level::unwatched: return false; // nothing to meet
        case usbc::vbus_level::safe0v: return voltage_mv <= usbc::spec::v_safe_0v_max;
        case usbc::vbus_level::safe5v:
            return voltage_mv >= usbc::spec::v_safe_5v_min &&
                   voltage_mv <= usbc::spec::v_safe_5v_max;
        case usbc::vbus_level::sink_disconnect:
            return voltage_mv < usbc::spec::v_sink_disconnect_max;
        case usbc::vbus_level::sink_disconnect_pd: return voltage_mv < 4000;
        }
        return false;
    }
    // test helpers: the armed condition
    bool met() const { return meets(*monitored); }
    void report()
    {
        reported_met = met();
        if (callback != nullptr) {
            callback(context, *monitored, reported_met);
        }
    }
    void setVoltage(std::int32_t mv)
    {
        voltage_mv = mv;
        if (monitored && *monitored != usbc::vbus_level::unwatched && met() != reported_met) {
            report();
        }
    }
};
