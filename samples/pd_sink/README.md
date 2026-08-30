# USB PD sink sample

Attach detection plus USB Power Delivery negotiation: the Type-C layer
reports attach and forwards PD alerts, the sink policy engine
negotiates a contract that `usbc::PowerPolicy` selects from the sink
capabilities (5 W minimum, 27 W target), and the resulting contract is
logged along with the load limits a real input regulator would receive.

## Build

```sh
west init -m https://github.com/alexanderwachter/UsbTypeC workspace
cd workspace
west update
west build -b stm32g081b_eval usbc/samples/pd_sink
west flash
```

Plugging into a PD source logs the negotiated contract (e.g. 9 V at
3 A from a 27 W charger); a plain 5 V charger stays on the implicit
Type-C contract.
