/*
 * The source's VBUS power supply as an integration interface. Programs
 * the voltage/current the port sources; the vbus block merely switches
 * this supply onto VBUS (its vbus_path::safe5v position). Only ports
 * that source power need to provide it.
 *
 * Derive the driver from SourceSupply (CRTP) and implement
 * (concepts::source_supply_client):
 *
 *   bool setOutput(millivolt, milliamp); // start the transition
 *   void setReadyCallback(supply_ready_callback, void*);
 *
 * The base is the observer of the source engine's states: it applies
 * the operating point they annotate - a negotiated target, 0 V across
 * a swap, vSafe5V on a restore - through setOutput(). The interface
 * is event-driven like the others: setOutput() starts the transition
 * to a new operating point and returns once the driver accepted it;
 * the ready callback (typically interrupt context) reports with
 * at_target = true when the output has settled within tolerance at
 * the target - the policy engine's trigger to send PS_RDY. A later
 * at_target = false reports lost regulation, which the policy engine
 * answers with error recovery. A new setOutput() replaces the
 * previous target.
 *
 * On a Request the sequence is: send Accept, wait tSrcTransition,
 * setOutput(), await the settled callback, send PS_RDY.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/PolicyEngine.hpp>
#include <usbc/Units.hpp>

#include <mtl/StateMachine.hpp>

#include <concepts>

namespace usbc {

using supply_ready_callback = void (*)(void* context, bool at_target);

// A policy's answer: the operating point the supply must deliver
struct supply_target {
    millivolt voltage = pe::v_safe_5v;
    milliamp current = pe::i_default_current;
    constexpr bool operator==(supply_target const&) const = default;
};

namespace concepts {

// The driver a SourceSupply-derived class provides
template<typename T>
concept source_supply_client = requires(T s, millivolt voltage, milliamp current_limit) {
    { s.setOutput(voltage, current_limit) } -> std::same_as<bool>;
};

// The supply as the source engine takes it: the settled report's
// callback, and the observer of the states' operating points (what
// SourceSupply provides)
template<typename T>
concept source_supply = requires(
    T s,
    supply_ready_callback callback,
    void* context,
    supply_target target,
    pe::restore_default_action restore
) {
    s.setReadyCallback(callback, context);
    s.notifyEntry(target);
    s.notifyEntry(restore);
};

} // namespace concepts

template<typename DERIVED>
class SourceSupply : public fsm::observing<SourceSupply<DERIVED>> {
public:
    // the derived class's interface is checked once the machine is
    // built - the CRTP class is complete by then
    template<typename TABLE>
    static constexpr void validate()
    {
        static_assert(
            concepts::source_supply_client<DERIVED>,
            "SourceSupply: the derived class must provide setOutput(millivolt, milliamp) -> bool"
        );
    }

    void notifyEntry(supply_target target) { derived().setOutput(target.voltage, target.current); }

    void notifyEntry(pe::restore_default_action)
    {
        derived().setOutput(pe::v_safe_5v, pe::i_default_current);
    }

private:
    DERIVED& derived() { return static_cast<DERIVED&>(*this); }
};

} // namespace usbc
