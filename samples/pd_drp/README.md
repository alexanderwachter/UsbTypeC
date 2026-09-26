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

Role swaps are full PD exchanges. A DR_Swap flips the data role on the
partner's Accept. A PR_Swap runs the spec's choreography: Accept, the
old source's transition to off, the PS_RDY hand-off, and the Type-C
termination flip at the Assert_Rd/Assert_Rp moments - the connection
layer's swap standby suspends detach detection while VBUS is
legitimately absent, and its tPSSourceOff/tPSSourceOn timeouts restart
connection resolution if the partner never completes. The joystick
triggers the requests: SEL a PR_Swap, LEFT a DR_Swap; the partner's
incoming requests are the engine tables' questions, answered from the
injected observers' allowSwap hooks.

A role lock keeps the port sink-only at runtime - a nearly empty
battery that must keep charging, say: inject an object answering
`bool check(usbc::tc::drp::sourcing_allowed)` next to the swap
policies. While it answers no, the port rests at Rd instead of
toggling (asking again every slice, so lifting the lock resumes
toggling within one), attaches as sink where a source-preferring port
would try Rp, and rejects PR_Swaps to source; a port already sourcing
keeps its contract until the application requests the swap. This
sample injects none: the question's default answer is yes.

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
and `vbus` phandles, dual power-role support, a
`zephyr,usb-c-pwrctrl` node (aliased `usbc-port0-pwrctrl`) for the
source power path, and two buttons (`sw0`, `sw1` aliases). The DRP
actively drives both terminations, so the TCPC must not be strapped
for dead-battery Rd.
