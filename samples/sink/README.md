# USB Type-C sink sample

Attach/detach detection with the UsbTypeC stack on Zephyr driver
adapters: `usbc::zephyr::Tcpc` and `usbc::zephyr::Vbus` wrap the
devices referenced by the board's `usb-c-connector` node, and the
Type-C sink state machine runs its debounce timer on the system
workqueue (`mtl::zephyr::WorkqueueTimer`).

## Build

Set up a workspace with this repository as the manifest:

```sh
west init -m https://github.com/alexanderwachter/UsbTypeC workspace
cd workspace
west update
west build -b stm32g081b_eval usbc/samples/sink
west flash
```

Plugging in a charger logs the orientation and the Rp current
advertisement; unplugging logs the detach.

## Requirements

A board whose devicetree provides a `usb-c-connector` node with `tcpc`
and `vbus` phandles (see `boards/stm32g081b_eval.overlay`).
