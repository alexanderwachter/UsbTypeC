/*
 * The power side of a port as integration interfaces, each a CRTP base
 * the application derives its own class from and injects into the
 * port as one observer. The spec-compliant sequencing lives in the
 * bases, delivered from the policy engines' annotations by overload;
 * the derived class provides the effects.
 *
 * SinkPower (concepts::sink_power_client) - the sink's power consumer,
 * typically the input limit of a charger IC or load switch:
 *
 *   bool setLimit(millivolt, milliamp);   // limit the sink load
 *   void onContract(millivolt, milliamp); // explicit contract in place
 *   void onContractLost();                // back to default power
 *
 * iSnkStdby during the sink transition (the standby_limit element),
 * the contract applied when the diagram's Power column changes to
 * Explicit Contract - every root state carries the level, so a bounce
 * between Ready and its service states is suppressed as unchanged; the
 * level edge arrives before the entry's contract terms (an instance
 * value) and marks the application pending for them - and vSafe5V
 * defaults restored on the states carrying the restore action, with
 * onContractLost() fired only when a contract was actually in place.
 * Unlike the supply there is no completion event: the standard bounds
 * the reduction by time (tSnkStdby), not by handshake.
 *
 * SourcePower (concepts::source_power_client) - the source's contract
 * notifications; the supply itself is the engine's business:
 *
 *   void onContract(millivolt, milliamp); // explicit contract in place
 *   void onContractLost();                // back to vSafe5V defaults
 *
 * SourceSupply (concepts::source_supply_client) - the source's VBUS
 * power supply, programming the voltage/current the port sources; the
 * vbus block merely switches this supply onto VBUS (its
 * vbus_path::safe5v position):
 *
 *   bool setOutput(millivolt, milliamp); // start the transition
 *   void setReadyCallback(supply_ready_callback, void*);
 *
 * The base applies the operating point the source engine's states
 * annotate - a negotiated target, 0 V across a swap, vSafe5V on a
 * restore - through setOutput(), which starts the transition and
 * returns once the driver accepted it; the ready callback (typically
 * interrupt context) reports with at_target = true when the output
 * has settled within tolerance at the target - the policy engine's
 * trigger to send PS_RDY. A later at_target = false reports lost
 * regulation, which the policy engine answers with error recovery. A
 * new setOutput() replaces the previous target. On a Request the
 * sequence is: send Accept, wait tSrcTransition, setOutput(), await
 * the settled callback, send PS_RDY.
 *
 * Failure contract as in Cc.hpp: the bool functions report whether the
 * driver accepted the operation.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/annotation/PolicyEngine.hpp>
#include <usbc/spec/Units.hpp>

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

// The interface a SinkPower-derived class provides: the sink load
// limit plus the contract notifications
template<typename T>
concept sink_power_client = requires(T client, millivolt voltage, milliamp current) {
    { client.setLimit(voltage, current) } -> std::convertible_to<bool>;
    client.onContract(voltage, current);
    client.onContractLost();
};

// The interface a SourcePower-derived class provides
template<typename T>
concept source_power_client = requires(T client, millivolt voltage, milliamp current) {
    client.onContract(voltage, current);
    client.onContractLost();
};

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
class SinkPower : public fsm::observing<SinkPower<DERIVED>> {
public:
    // the derived class's interface is checked once the machine is
    // built - the CRTP class is complete by then
    template<typename TABLE>
    static constexpr void validate()
    {
        static_assert(
            concepts::sink_power_client<DERIVED>,
            "SinkPower: the derived class must provide setLimit(millivolt, "
            "milliamp), onContract(millivolt, milliamp), onContractLost()"
        );
    }

    void notifyEntry(pe::power_level level)
    {
        apply_pending_ = level == pe::power_level::explicit_contract;
    }

    // delivered on every entry of Ready: the body stays out of the
    // many edges leading there
    void notifyEntry(pe::active_contract contract)
    {
        if (apply_pending_) {
            applyContract(contract);
        }
    }

    void notifyEntry(pe::standby_limit limit)
    {
        derived().setLimit(limit.voltage, pe::i_snk_stdby);
    }

    void notifyEntry(pe::restore_default_action) { restoreDefaults(); }

private:
    DERIVED& derived() { return static_cast<DERIVED&>(*this); }

    void applyContract(pe::active_contract contract)
    {
        apply_pending_ = false;
        contract_active_ = true;
        derived().setLimit(contract.voltage, contract.current);
        derived().onContract(contract.voltage, contract.current);
    }

    void restoreDefaults()
    {
        if (contract_active_) {
            contract_active_ = false;
            derived().setLimit(pe::v_safe_5v, pe::i_default_current);
            derived().onContractLost();
        }
    }

    bool apply_pending_ = false;
    bool contract_active_ = false;
};

template<typename DERIVED>
class SourcePower : public fsm::observing<SourcePower<DERIVED>> {
public:
    // the derived class's interface is checked once the machine is
    // built - the CRTP class is complete by then
    template<typename TABLE>
    static constexpr void validate()
    {
        static_assert(
            concepts::source_power_client<DERIVED>,
            "SourcePower: the derived class must provide onContract(millivolt, "
            "milliamp) and onContractLost()"
        );
    }

    // every root state carries the level; its edge to Explicit
    // Contract arrives before the entry's contract terms (an instance
    // value) and marks the report pending for them
    void notifyEntry(pe::power_level level)
    {
        apply_pending_ = level == pe::power_level::explicit_contract;
    }

    // delivered on every entry of Ready: the body stays out of the
    // many edges leading there
    void notifyEntry(pe::active_contract contract)
    {
        if (apply_pending_) {
            applyContract(contract);
        }
    }

    void notifyEntry(pe::restore_default_action) { restoreDefaults(); }

private:
    DERIVED& derived() { return static_cast<DERIVED&>(*this); }

    void applyContract(pe::active_contract contract)
    {
        apply_pending_ = false;
        contract_active_ = true;
        derived().onContract(contract.voltage, contract.current);
    }

    void restoreDefaults()
    {
        if (contract_active_) {
            contract_active_ = false;
            derived().onContractLost();
        }
    }

    bool apply_pending_ = false;
    bool contract_active_ = false;
};

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
