# USB PD dual-role port sample

DRP toggling plus PD power negotiation in whichever role the attach
resolves to: a charger makes the port a negotiating sink, a sink makes
it an advertising source. One policy engine per role is instantiated
up front; the `PortRouter` observer activates the one the resolved
role needs, routes the PD alerts, arbitrates role swaps
(`allowSwap`), and keeps the TCPC's message header aligned with the
current power and data roles.

Role swaps ride the attach observation: a swap's standby exits the old
attached state (tearing that engine down) and its completion enters
the new one (bringing the other engine up) - the same path as a
plug-in. The joystick stands in for the PD swap messaging: SEL swaps
the power role, LEFT the data role. Both are local-only until the
policy engines speak PR_Swap/DR_Swap; a partner will not follow.

## Build

Set up a workspace with this repository as the manifest:

```sh
west init -m https://github.com/alexanderwachter/UsbTypeC workspace
cd workspace
west update
west build -b stm32g081b_eval usbc/samples/pd_drp
west flash
```

## Requirements

A board whose devicetree provides a `usb-c-connector` node with `tcpc`
and `vbus` phandles, dual power-role support, and two buttons (`sw0`,
`sw1` aliases). The DRP actively drives both terminations, so the TCPC
must not be strapped for dead-battery Rd.
