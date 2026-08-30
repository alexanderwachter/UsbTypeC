/*
 * Optional classic inheritance facade over the integration concepts:
 * abstract interfaces whose virtuals mirror the duck-typed contracts,
 * for users who prefer implementing against a base class. Deriving is
 * never required - the stack constrains on the concepts only, and the
 * static_asserts below keep each facade in sync with its concept, so
 * every implementation of an interface satisfies the concept by
 * construction.
 *
 * A driver implements the interfaces its port needs via multiple
 * inheritance (e.g. TcpcInterface + VconnInterface +
 * PdTransportInterface for a full PD port). To dispatch virtually
 * through the stack, instantiate it with the interface type itself,
 * e.g. ProtocolLayer<PdTransportInterface, ...>.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Tcpc.hpp>
#include <usbc/Vbus.hpp>

#include <optional>

namespace usbc {

// concepts::tcpc as an abstract interface
class TcpcInterface {
public:
    virtual ~TcpcInterface() = default;

    virtual void setAlertHandler(alert_callback callback, void* context)   = 0;
    virtual std::optional<alert_status> readAlert()                        = 0;
    virtual bool setCc(cc_pull pull, rp_value rp)                          = 0;
    virtual std::optional<cc_status> readCcStatus()                        = 0;
    virtual bool setPlugOrientation(plug_orientation orientation)          = 0;
    virtual bool sourceVbus(bool enable)                                   = 0;
    virtual bool sinkVbus(bool enable)                                     = 0;
};
static_assert(concepts::tcpc<TcpcInterface>);

// concepts::vconn_switch as an abstract interface
class VconnInterface {
public:
    virtual ~VconnInterface() = default;

    virtual bool setVconn(bool enable) = 0;
};
static_assert(concepts::vconn_switch<VconnInterface>);

// concepts::pd_transport as an abstract interface
class PdTransportInterface {
public:
    virtual ~PdTransportInterface() = default;

    virtual bool setMessageHeaderInfo(message_header_info info) = 0;
    virtual bool setReceiveDetect(receive_detect detect)        = 0;
    virtual bool transmit(pd_message const& message)            = 0;
    virtual bool transmit(transmit_signal signal)               = 0;
    virtual bool receive(pd_message& message)                   = 0;
};
static_assert(concepts::pd_transport<PdTransportInterface>);

// concepts::vbus as an abstract interface
class VbusInterface {
public:
    virtual ~VbusInterface() = default;

    virtual bool enable(bool on)                                    = 0;
    virtual void setCallback(vbus_callback callback, void* context) = 0;
    virtual bool monitor(vbus_level level)                          = 0;
    virtual bool discharge(bool on)                                 = 0;
};
static_assert(concepts::vbus<VbusInterface>);

} // namespace usbc
