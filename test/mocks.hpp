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

    void setAlertHandler(usbc::alert_callback cb, void* ctx)
    {
        callback = cb;
        context  = ctx;
    }
    std::optional<usbc::alert_status> readAlert()
    {
        auto const pending = alerts;
        alerts             = usbc::alert_status::none;
        return pending;
    }
    bool setCc(usbc::cc_pull p, usbc::rp_value current)
    {
        pull = p;
        rp   = current;
        return true;
    }
    std::optional<usbc::cc_status> readCcStatus() { return line_state; }
    bool setPlugOrientation(usbc::plug_orientation o)
    {
        orientation = o;
        return true;
    }
    bool sourceVbus(bool enable)
    {
        sourcing = enable;
        return true;
    }
    bool sinkVbus(bool enable)
    {
        sinking = enable;
        return true;
    }
    bool setVconn(bool enable)
    {
        vconn = enable;
        return true;
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
    void injectMessage(usbc::pd_message const& message)
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
    void setCallback(usbc::vbus_callback cb, void* ctx)
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
    // Discharging takes time and completes asynchronously: the test
    // drives the voltage explicitly (a synchronous report here would
    // re-enter process() from within an observer hook)
    bool discharge(bool enable)
    {
        discharging = enable;
        return true;
    }
    bool discharging = false;

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
    void setVoltage(std::int32_t mv)
    {
        voltage_mv = mv;
        if (monitored && met() != reported_met) {
            report();
        }
    }
};
