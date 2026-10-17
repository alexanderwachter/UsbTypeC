# USB PD dual-role port sample

DRP toggling plus PD power negotiation in whichever role the attach
resolves to: a charger makes the port a negotiating sink, a sink makes
it an advertising source. The `usbc::PdDrp` facade owns both policy
engines and the routing between them - the application provides only
the domain pieces: drivers, timers, capabilities, policies, and the
power effects (the source offers come from the connector node's
`source-pdos`).

On the STM32G081B-EVAL the sample drives the board's real source
power path, the same wiring as Zephyr's `usb_c/drp` sample: a PWM
duty cycle selects the DCDC output voltage (5/9/15 V), GPIOs gate the
DCDC and the VBUS source switch, and the ADC-based vbus driver
measures and discharges VBUS.

On a NUCLEO-G474RE with the X-NUCLEO-DRP1M1 shield the source is the
shield's SOURCE path, 5 V from the Nucleo's rail through the CN4
jumper: the TCPP03-M20 power path controller switches it onto VBUS,
opens and closes the sink path, discharges VBUS and carries the VCONN
switch (registered as the UCPD driver's VCONN callbacks). The offer
of 1.5 A presumes the rail is fed from E5V/VIN, not from the ST-LINK's
USB alone.

Role swaps are full PD exchanges. A DR_Swap flips the data role on the
partner's Accept. A PR_Swap runs the spec's choreography: Accept, the
old source's transition to off, the PS_RDY hand-off, and the Type-C
termination flip at the Assert_Rd/Assert_Rp moments - the connection
layer's swap standby suspends detach detection while VBUS is
legitimately absent, and its tPSSourceOff/tPSSourceOn timeouts restart
connection resolution if the partner never completes. The buttons
trigger the requests: `sw0` a PR_Swap, `sw1` a DR_Swap (the eval
board's joystick SEL and LEFT; a Nucleo has only `sw0`); the partner's
incoming requests are the engine tables' questions, answered by the
injected observers' `check(usbc::pe::pr_swap_allowed)` and kin - the
answers are what enable the features.

A role lock keeps the port sink-only at runtime - a nearly empty
battery that must keep charging, say: inject an object answering
`bool check(usbc::sourcing_allowed)` next to the swap policies. While
it answers no, the port rests at Rd instead of toggling (asking again
every slice, so lifting the lock resumes toggling within one),
attaches as sink where a source-preferring port would try Rp, and
rejects PR_Swaps to source; a port already sourcing keeps its
contract until the application requests the swap. This sample injects
none: the question's default answer is yes.

## Build

Set up a workspace with this repository as the manifest:

```sh
west init -m https://github.com/alexanderwachter/UsbTypeC workspace
cd workspace
west update
west build -b stm32g081b_eval usbc/samples/pd_drp
west flash
```

With the shield:

```sh
west build -b nucleo_g474re --shield x_nucleo_drp1m1 usbc/samples/pd_drp
```

## Requirements

A board whose devicetree provides a `usb-c-connector` node with `tcpc`
and `vbus` phandles, dual power-role support, a source power path -
either a `zephyr,usb-c-pwrctrl` node (aliased `usbc-port0-pwrctrl`)
or a `ppc` phandle on the connector switching a fixed 5 V rail - and
a button (`sw0` alias; `sw1` adds the DR_Swap trigger). The DRP
actively drives both terminations, so the TCPC must not be strapped
for dead-battery Rd.
