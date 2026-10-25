/*
 * Optional classic inheritance facade over the integration concepts:
 * abstract interfaces whose virtuals mirror the duck-typed contracts,
 * for users who prefer implementing against a base class. Deriving is
 * never required - the stack constrains on the concepts only, and the
 * static_asserts below keep each facade in sync with its concept.
 *
 * A driver implements the interfaces its port needs via multiple
 * inheritance (e.g. CcInterface + PdPhyInterface for a port
 * controller). To dispatch virtually through the stack, instantiate
 * it with the interface type itself, e.g. ProtocolLayer<PdPhyInterface,
 * ...>: the observer interfaces derive from fsm::observing of the
 * interface type, so it is the interface that satisfies the concept,
 * not the class deriving from it.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Cc.hpp>
#include <usbc/PdPhy.hpp>
#include <usbc/Vbus.hpp>

#include <mtl/StateMachine.hpp>

#include <optional>

namespace usbc {

// concepts::cc as an abstract interface
class CcInterface : public fsm::observing<CcInterface> {
public:
    virtual ~CcInterface() = default;

    virtual void notifyEntry(cc_termination termination) = 0;
    virtual bool setRp(rp_value rp) = 0;
    virtual void setCallback(cc_callback callback, void* context) = 0;
    virtual std::optional<cc_status> readCcStatus() = 0;
};
static_assert(concepts::cc<CcInterface>);

// concepts::pd_phy as an abstract interface
class PdPhyInterface {
public:
    virtual ~PdPhyInterface() = default;

    virtual void setCallbacks(pd_phy_callbacks const& callbacks, void* context) = 0;
    virtual bool setMessageHeaderInfo(message_header_info info) = 0;
    virtual bool setReceiveDetect(receive_detect detect) = 0;
    virtual bool selectCcLine(plug_orientation line) = 0;
    virtual bool transmit(pd_message const& message) = 0;
    virtual bool transmit(transmit_signal signal) = 0;
};
static_assert(concepts::pd_phy<PdPhyInterface>);

// concepts::vbus as an abstract interface
class VbusInterface : public fsm::observing<VbusInterface> {
public:
    virtual ~VbusInterface() = default;

    virtual void notifyEntry(vbus_power power) = 0;
    virtual void notifyEntry(vbus_level level) = 0;
    virtual bool check(vbus_at_safe0v) = 0;
    virtual bool check(vbus_at_safe5v) = 0;
    virtual void setCallback(vbus_callback callback, void* context) = 0;
};
static_assert(concepts::vbus<VbusInterface>);

} // namespace usbc
