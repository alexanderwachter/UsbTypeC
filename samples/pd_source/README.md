# USB PD source sample

Attach detection plus PD power advertisement with the UsbTypeC stack
on Zephyr driver adapters. The Type-C source layer presents Rp,
applies VBUS from vSafe0V, and forwards the PD alerts to the source
policy engine, which advertises two fixed PDOs (5 V / 1.5 A and
9 V / 1 A), evaluates the sink's Request through `RequestPolicy`, and
walks the Accept / tSrcTransition / supply / PS_RDY choreography. A
PD-incapable sink leaves the port a plain Type-C source after
nCapsCount advertisements.

The board has no programmable supply: the sample's `Supply` logs the
requested operating point and reports it settled from the stack's work
queue. A real implementation programs the regulator and fires the
callback when the output is within tolerance.

## Build

Set up a workspace with this repository as the manifest:

```sh
west init -m https://github.com/alexanderwachter/UsbTypeC workspace
cd workspace
west update
west build -b stm32g081b_eval usbc/samples/pd_source
west flash
```

Plugging in a PD sink logs the attach, the negotiated contract, and
the supply operating points; unplugging logs the detach and the
contract loss.

## Requirements

A board whose devicetree provides a `usb-c-connector` node with `tcpc`
and `vbus` phandles, a TCPC driver with source control, and a VBUS
driver with discharge support (see `boards/stm32g081b_eval.overlay`).
