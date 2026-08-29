/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <usbc/Tcpc.hpp>
#include <usbc/Vbus.hpp>

#include <cstdint>
#include <optional>

// TCPC driver test double, shared by the interface and protocol layer tests
struct mock_tcpc {
    bool initialized              = false;
    usbc::alert_callback callback = nullptr;
    void* context                 = nullptr;
    usbc::alert_status alerts     = usbc::alert_status::none;
    usbc::cc_pull pull            = usbc::cc_pull::open;
    usbc::rp_value rp             = usbc::rp_value::usb_default;
    usbc::cc_status line_state{usbc::cc_state::src_open, usbc::cc_state::src_open};
    usbc::plug_orientation orientation = usbc::plug_orientation::cc1;
    bool sourcing                 = false;
    bool sinking                  = false;
    bool vconn                    = false;
    usbc::receive_detect detect   = usbc::receive_detect::none;
    usbc::message_header_info header_info{};
    usbc::pd_message last_transmitted{};
    int transmit_count   = 0;
    bool accept_transmit = true;
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

// VBUS driver test double simulating a comparator
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
