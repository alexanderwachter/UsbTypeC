.. _x-nucleo-usbpdm1:

X-NUCLEO-USBPDM1: USB Type-C Power Delivery sink shield
######################################################

Overview
********

The X-NUCLEO-USBPDM1 is a USB Type-C Power Delivery sink expansion board
for STM32 Nucleo-64 boards. It carries a Type-C receptacle, a TCPP01-M12
port protector (CC overvoltage protection, VBUS gate, dead-battery Rd) and
a 3.3 V LDO that can power the Nucleo from VBUS. It is a sink-only board:
there is no source power path.

More information about the board can be found at the
`X-NUCLEO-USBPDM1 website`_.

Hardware
********

The shield connects to the host through the Arduino UNO R3 and ST morpho
connectors:

+--------+------------------------------+-------------------------------------+
| Signal | Header                       | Role                                |
+========+==============================+=====================================+
| CC1    | Arduino D10 (CN10 pin 17)    | UCPD CC1                            |
+--------+------------------------------+-------------------------------------+
| CC2    | Arduino D5 (CN10 pin 27)     | UCPD CC2                            |
+--------+------------------------------+-------------------------------------+
| D+     | CN10 pin 12                  | USB full-speed data                 |
+--------+------------------------------+-------------------------------------+
| D-     | CN10 pin 14                  | USB full-speed data                 |
+--------+------------------------------+-------------------------------------+
| VBUS   | Arduino A3                   | ADC through a 200 kΩ / 40.2 kΩ      |
|        |                              | divider                             |
+--------+------------------------------+-------------------------------------+

The TCPP01-M12 fault output (FLT) and dead-battery output (DB) reach the
Nucleo through the shield's JP2 "interface configuration" jumpers; this
shield does not describe them.

The shield overlay describes the connector, the VBUS measurement
(``zephyr,usb-c-vbus-adc`` on ``arduino_adc`` channel 3) and the port node
aliased as ``usbc-port0``. The UCPD and USB alternate functions are pin
assignments of the host MCU, so each supported board has its own overlay
under ``boards/`` that routes them and names the TCPC.

Requirements
************

This shield can only be used with a board that provides a UCPD peripheral
on the CC pins. Supported boards:

- ``nucleo_g474re``: UCPD1 on PB6 / PB4, USB on PA11 / PA12 (FDCAN1 is
  disabled because it owns those pins on the board), ADC1_IN15 for VBUS.
  SPI1 (chip select on D10) and the TIM3 PWM (D5) are disabled. The board
  does not describe its Arduino analog header, so the board overlay of the
  shield provides the ``arduino_adc`` mapping.

Programming
***********

Set ``--shield x_nucleo_usbpdm1`` when you invoke ``west build``. For
example, from the workspace topdir:

.. code-block:: sh

   west build -b nucleo_g474re --shield x_nucleo_usbpdm1 UsbTypeC/samples/pd_sink

References
**********

.. target-notes::

.. _X-NUCLEO-USBPDM1 website:
   https://www.st.com/en/ecosystems/x-nucleo-usbpdm1.html
