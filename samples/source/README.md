# USB Type-C source sample

Attach/detach detection for a power-sourcing port with the UsbTypeC
stack on Zephyr driver adapters: `usbc::zephyr::Tcpc` and
`usbc::zephyr::Vbus` wrap the devices referenced by the board's
`usb-c-connector` node, and the Type-C source state machine runs its
debounce timer on the stack's work queue.

The port presents Rp (default USB current), waits for a sink's Rd to
debounce and VBUS to sit at vSafe0V, then switches the source path on.
When the sink is removed, VBUS is switched off and discharged back to
vSafe0V before the port accepts the next attach.

## Build

Set up a workspace with this repository as the manifest:

```sh
west init -m https://github.com/alexanderwachter/UsbTypeC workspace
cd workspace
west update
west build -b stm32g081b_eval usbc/samples/source
west flash
```

Plugging in a sink logs the orientation and applies VBUS; unplugging
logs the detach.

## Requirements

A board whose devicetree provides a `usb-c-connector` node with `tcpc`
and `vbus` phandles, a TCPC driver with source control
(`tcpc_set_src_ctrl`), and a VBUS driver with discharge support (see
`boards/stm32g081b_eval.overlay`).
