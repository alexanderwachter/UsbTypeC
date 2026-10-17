.. _x-nucleo-drp1m1:

X-NUCLEO-DRP1M1: USB Type-C Power Delivery dual role power shield
#################################################################

Overview
********

The X-NUCLEO-DRP1M1 is a USB Type-C Power Delivery dual role power (DRP)
and dual role data expansion board for STM32 Nucleo-64 boards. It carries
a Type-C receptacle, a TCPP03-M20 port protector (gate drivers for the
N-MOSFET pairs of the sink and source VBUS paths, VBUS and VCONN
discharge, a VCONN switch, CC and VBUS overvoltage protection, VBUS
overcurrent protection, dead-battery Rd) and a 3.3 V LDO that can power
the Nucleo from VBUS. The sink path ends at the SINK screw terminal (CN2),
the source path starts at the SOURCE screw terminal (CN3); a jumper on CN4
feeds the source path from the Nucleo's 5 V rail.

More information about the board can be found at the
`X-NUCLEO-DRP1M1 website`_.

Hardware
********

The shield connects to the host through the Arduino UNO R3 and ST morpho
connectors:

+--------+------------------------------+-------------------------------------+
| Signal | Header                       | Role                                |
+========+==============================+=====================================+
| CC1    | Arduino D10 (CN10 pin 17)    | UCPD CC1 (also CN10 pin 23, for an  |
|        |                              | STM32G0 host)                       |
+--------+------------------------------+-------------------------------------+
| CC2    | Arduino D5 (CN10 pin 27)     | UCPD CC2 (also CN10 pin 26, for an  |
|        |                              | STM32G0 host)                       |
+--------+------------------------------+-------------------------------------+
| D+     | CN10 pin 12                  | USB full-speed data                 |
+--------+------------------------------+-------------------------------------+
| D-     | CN10 pin 14                  | USB full-speed data                 |
+--------+------------------------------+-------------------------------------+
| VBUS   | Arduino A0                   | ADC through a 200 kΩ / 40.2 kΩ      |
|        |                              | divider, connector side             |
+--------+------------------------------+-------------------------------------+
| SCL    | Arduino D15 (CN10 pin 3)     | TCPP03-M20 I2C, address 0x34        |
+--------+------------------------------+-------------------------------------+
| SDA    | Arduino D14 (CN10 pin 5)     | TCPP03-M20 I2C                      |
+--------+------------------------------+-------------------------------------+
| ENABLE | CN10 pin 2                   | TCPP03-M20 enable, GPIO             |
+--------+------------------------------+-------------------------------------+
| FLGn   | Arduino D0 (CN10 pin 37)     | TCPP03-M20 fault flag, open drain   |
+--------+------------------------------+-------------------------------------+
| IANA   | Arduino A4                   | VBUS current, 42 V/V across 7 mΩ    |
+--------+------------------------------+-------------------------------------+
| SOURCE | Arduino A1                   | Source terminal voltage, same       |
|        |                              | divider as VBUS                     |
+--------+------------------------------+-------------------------------------+
| SINK   | Arduino A2                   | Sink terminal voltage, same divider |
+--------+------------------------------+-------------------------------------+

The shield overlay describes the connector (``power-role = "dual"``), the
VBUS measurement (``zephyr,usb-c-vbus-adc`` on ``arduino_adc`` channel 0),
the TCPP03-M20 on ``arduino_i2c`` as the port's power path controller
(``st,tcpp03-m20``, the ``ppc`` of the connector, its ENABLE on the morpho
header) and the port node aliased as ``usbc-port0``. The UCPD and USB
alternate functions are pin assignments of the host MCU, so each supported
board has its own overlay under ``boards/`` that routes them and names the
TCPC.

The TCPP03-M20 driver (``zephyr/drivers/usb_c/ppc/st_tcpp03_m20.c``)
implements Zephyr's power path controller API: the sink and source VBUS
paths, VBUS discharge, VBUS presence and the fault events. The VCONN
switch and discharge, which that API has no operations for, are
``tcpp03_m20_set_vconn()`` and ``tcpp03_m20_set_vconn_discharge()``
(``zephyr/drivers/usb_c/tcpp03_m20.h``): a UCPD-based port registers them
as the TCPC driver's VCONN callbacks.

FLGn is not wired in the board overlays: on the Nucleo-64 it lands on the
LPUART1 RX of the ST-LINK virtual COM port. Without it the driver reports
no fault events and does not write the recovery word an overcurrent trip
needs; wire ``irq-gpios`` of the ``st,tcpp03-m20`` node on a board that
frees the pin. The current and terminal voltage outputs are not described
either.

The TCPP03-M20 starts in its dead-battery mode (sink path closed, Rd on
CC) until the driver enables it, and the stack opens the sink path when
the port starts: a Nucleo powered from VBUS alone, through the shield's
LDO, loses its supply at that moment. Power the Nucleo through its
ST-LINK or E5V/VIN.

Requirements
************

This shield can only be used with a board that provides a UCPD peripheral
on the CC pins and an I2C bus on the Arduino D14/D15 pins. Supported
boards:

- ``nucleo_g474re``: UCPD1 on PB6 / PB4, USB on PA11 / PA12 (FDCAN1 is
  disabled because it owns those pins on the board), ADC1_IN1 for VBUS,
  I2C1 on PB8 / PB9, ENABLE on PC8. SPI1 (chip select on D10), SPI2 (MOSI
  on PB15, the shield's second CC2 link), the TIM3 PWM (D5) and DAC1 (A2)
  are disabled. The board does not describe its Arduino analog header, so
  the board overlay of the shield provides the ``arduino_adc`` mapping.

Programming
***********

Set ``--shield x_nucleo_drp1m1`` when you invoke ``west build``. For
example, from the workspace topdir:

.. code-block:: sh

   west build -b nucleo_g474re --shield x_nucleo_drp1m1 UsbTypeC/samples/pd_drp

References
**********

.. target-notes::

.. _X-NUCLEO-DRP1M1 website:
   https://www.st.com/en/ecosystems/x-nucleo-drp1m1.html
