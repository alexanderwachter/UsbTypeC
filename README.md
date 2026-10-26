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
  stack to their hardware and OS: the CC block, the PD PHY and the VBUS
  block as observers of the state machines (`usbc/interface/Cc.hpp`,
  `usbc/interface/PdPhy.hpp`, `usbc/interface/Vbus.hpp`), the power
  side and the policies (`usbc/interface/Power.hpp`,
  `usbc/interface/Policy.hpp`), timers, and an execution context for
  event processing.
- **Zephyr module glue** — build files so the library plugs into a Zephyr
  application via the module system.

The state machines are built on the template-based `fsm` state machine from
[McuTemplateLibrary](https://github.com/alexanderwachter/McuTemplateLibrary)
(`mtl`), a project of the west manifest (`west.yml`) fetched into the
workspace at `modules/lib/mtl` by `west update`.

## Repository layout

| Directory | Content |
|---|---|
| `include/usbc/` | The ports an application instantiates (`PdSink`, `PdSource`, `PdDrp`, `TypeCSink`, `TypeCSource`, `TypeCDrp`) |
| `include/usbc/interface/` | What one injected object provides: the hardware blocks, the power side, the policies, the questions the tables ask |
| `include/usbc/annotation/` | The facts the machines publish, what any observer hears |
| `include/usbc/state/` | The states, guards and transition tables, what the compliance checks and the DOT graphs look at |
| `include/usbc/layer/` | The machines the ports compose: a table in a queued machine behind a port API |
| `include/usbc/spec/` | The specifications' parameters and wire formats |
| `test/` | Host-side unit tests for the state machines |
| `boards/shields/` | Zephyr shields for Type-C expansion boards (`x_nucleo_usbpdm1`, `x_nucleo_drp1m1`) |
| `zephyr/` | Zephyr module definition (`module.yml`, Kconfig, CMake glue) and the module's own drivers (TCPP03-M20 power path controller) |

## Building

Set up the west workspace (fetches Zephyr and mtl):

```sh
west init -m <this repo> workspace
cd workspace
west update
```

Host build with tests, from the manifest repository:

```sh
cmake -B build
cmake --build build
ctest --test-dir build
```

Without a west workspace, point the host build at an mtl checkout with
`-DMTL_ROOT=<path>`.

## Using as a Zephyr module

Add the repository to your west manifest (or `ZEPHYR_EXTRA_MODULES`) and
enable it via Kconfig:

```
CONFIG_USB_TYPEC_STACK=y
```

## License

Apache-2.0, see [LICENSE](LICENSE).
