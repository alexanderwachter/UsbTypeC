# UsbTypeC

A portable C++ implementation of the USB Type-C connection state machines,
including USB Power Delivery (PD), according to the

- USB Type-C Cable and Connector Specification
- USB Power Delivery Specification

The library implements the state machines required by the standard and exposes
a clean interface for integrating them into your own firmware. It is
platform-agnostic at its core; Zephyr is a first-class target platform and the
repository can be consumed directly as a Zephyr module.

## Scope

What belongs in this repository:

- **Type-C port state machines** — CC-line based connection handling for
  Source, Sink, and DRP roles (attach/detach detection, orientation,
  Rp/Rd handling, Try.SRC/Try.SNK, ...).
- **USB Power Delivery** —
  - Protocol layer (message construction/parsing, GoodCRC handling,
    message counters, timers)
  - Policy engine state machines (Source/Sink policy, power negotiation,
    role swaps)
  - A user-facing device policy interface (capability selection, callbacks
    for contract/state changes)
- **Integration interface** — abstractions the user implements to bind the
  stack to their hardware and OS: TCPC / PHY access, timers, and an
  execution context for event processing.
- **Zephyr module glue** — build files so the library plugs into a Zephyr
  application via the module system.

The state machines are built on the template-based `fsm` state machine from
[McuTemplateLibrary](https://github.com/alexanderwachter/McuTemplateLibrary)
(`mtl`), included as a git submodule at `lib/McuTemplateLibrary`.

## Repository layout

| Directory | Content |
|---|---|
| `include/` | Public headers (`include/usbc/...`) |
| `src/` | Library sources |
| `test/` | Host-side unit tests for the state machines |
| `zephyr/` | Zephyr module definition (`module.yml`, Kconfig, CMake glue) |
| `lib/McuTemplateLibrary` | `mtl` submodule providing the state machine framework |

## Building

Clone with submodules:

```sh
git clone --recurse-submodules <this repo>
```

Host build with tests:

```sh
cmake -B build
cmake --build build
ctest --test-dir build
```

## Using as a Zephyr module

Add the repository to your west manifest (or `ZEPHYR_EXTRA_MODULES`) and
enable it via Kconfig:

```
CONFIG_USB_TYPEC_STACK=y
```

## License

Apache-2.0, see [LICENSE](LICENSE).
