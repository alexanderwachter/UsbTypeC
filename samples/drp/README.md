# USB Type-C dual-role port sample

DRP toggling with the UsbTypeC stack on Zephyr driver adapters: the
port alternates between presenting Rd and Rp (tDRP/dcSRC.DRP) and
resolves to whichever role the attached partner complements - a
charger makes it a sink, a sink makes it a source. The injected
`usbc::zephyr::StateLogger` traces every transition on the `usbc_fsm`
log module at debug level.

Toggle timing and role preference are compile-time configuration:
derive from `usbc::default_drp_timing` (spec ranges enforced at
compile time) and pick `usbc::drp_preference::source`/`::sink` for
Try.SRC/Try.SNK role resolution.

PD-directed role swaps (`swapToSource()`/`swapToSink()`/
`swapDataRole()`) are arbitrated by injected observers providing
`allowSwap(power_role)`/`allowSwap(data_role)` - without one, swaps of
that kind are refused. The sample injects an accepting arbiter, a data
role observer (`onDataRole`), and attempts a data role swap every few
seconds while attached, standing in for the PD layer.

## Build

Set up a workspace with this repository as the manifest:

```sh
west init -m https://github.com/alexanderwachter/UsbTypeC workspace
cd workspace
west update
west build -b stm32g081b_eval usbc/samples/drp
west flash
```

Plugging in a charger logs an attach as sink; plugging in a sink
(e.g. a phone) logs an attach as source with VBUS applied.

## Requirements

A board whose devicetree provides a `usb-c-connector` node with `tcpc`
and `vbus` phandles and dual power-role support (see
`boards/stm32g081b_eval.overlay`). The DRP actively drives both
terminations, so the TCPC must not be strapped for dead-battery Rd.
