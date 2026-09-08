# Task: adopt the fsm library's annotation sets

The fsm library gained annotation sets (McuTemplateLibrary, branch
`statemachine`, commit a464e23 "Samples: lamps and power rail as
annotation sets"; documentation in its README.md under "Features" and
in AGENTS.md). Adopt them in the USB-C stack where they make observers
simpler.

## What it is

A state may carry several annotations keyed by type:

```cpp
static constexpr auto annotations = fsm::annotate(rp_value::p_1a5, vconn_role::source);
```

An `fsm::observing` observer then needs no `observe_static()`: every
element type for which it has a `notifyEntry(T)` / `notifyExit(T)`
overload is delivered, change-suppressed per element at compile time
(the values are constexpr), in set order, between the static and the
nonstatic hook. A state without the set, or without that type in it,
counts as "no value" - a change on entry and on exit - exactly like a
missing member does today.

Rules:

- The types of one set are distinct and meaningful: a strong type per
  annotation, never `int` or `bool`. The type is the key.
- An element type may declare `static constexpr bool idempotent = true`;
  an observer may still declare `renotify_safe`. Both keep the shared
  wildcard body available.
- The shared-wildcard facts (`exit_silent`, `entry_shared_from`) and the
  coverage traits (`is_observed_v`, `is_notified_of_v`,
  `all_states_notified_v`) include set elements.
- `observe_static` / `observe_nonstatic` keep working and may coexist
  with a set in one observer.

## Steps

1. Bump `lib/McuTemplateLibrary` to a464e23 (`origin/statemachine`).
   Work on a branch off `fsm-features`.
2. Find the observers that exist only because one observer could watch
   one annotation: the members bundled into `fsm::observer_group`
   (`contract_store` and its siblings behind `SinkPower`/`SourcePower`),
   `hw_driver` / `src_hw_driver`, `phy_driver`, and any state carrying
   several `static constexpr` annotations watched by several observers.
   Where one-observer-per-annotation was a workaround, collapse them
   into one observer with overloads and move the annotations into one
   `fsm::annotate(...)` set on the states. Keep observers apart where
   they are separate concerns.
3. Do not change behaviour: the same notifications on the same edges.
   `all_states_notified_v` in the `validate()` hooks must keep passing;
   if a collapsed observer no longer covers a state, that is a real
   finding, not something to paper over.
4. Verify: host tests (`cmake -B build -G Ninja && cmake --build build &&
   ./build/test/UsbTypeCTests`), and `samples/pd_drp` on stm32g081b_eval
   must stay at 57976 B flash (57996 B before the library's feature-tag
   work). Use your own Python venv for west, for example
   `/home/alex/Documents/McuTemplateLibrary/build-venv`, never the
   developer's. Report the size before and after; any growth needs an
   explanation.
5. Update the stack's design notes where they describe observers.
