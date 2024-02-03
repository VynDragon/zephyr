.. _xt_bl_zb1:

BL702 Module
#######################

Overview
********

BL702 is a low-power, high-performance IoT chip that supports BLE/Zigbee wireless networking,
built-in single-core RISC-V 32-bit CPU, supports multiple security mechanisms.
The chip provides rich cache and memory resources, integrates a variety of peripherals,
and provides an industry-leading single-chip multi-purpose solution for IoT products, w
hich is suitable for a wide range of IoT application scenarios.

.. image:: img/bl702_1.jpg
     :width: 450px
     :align: center
     :alt: bl702 module front (probably clone)

.. image:: img/bl702_2.jpg
     :width: 450px
     :align: center
     :alt: bl702 module back (probably clone)

Hardware
********


For more information about the Bouffalo Lab BL-602 MCU:

- `Bouffalo Lab BL702 MCU Website`_
- `Bouffalo Lab BL702 MCU Datasheet`_
- `Bouffalo Lab Development Zone`_

Supported Features
==================

The board configuration supports the following hardware features:

+-----------+------------+-----------------------+
| Interface | Controller | Driver/Component      |
+===========+============+=======================+
| MTIMER    | on-chip    | RISC-V Machine Timer  |
+-----------+------------+-----------------------+
| PINCTRL   | on-chip    | pin muxing            |
+-----------+------------+-----------------------+
| UART      | on-chip    | serial port-polling   |
+-----------+------------+-----------------------+


The default configurations can be found in the Kconfig
:zephyr_file:`boards/riscv/xt_bl_zb1/xt_bl_zb1_defconfig`.

System Clock
============

The BL702 Development Board is configured to run at max speed (144MHz).

Serial Port
===========

The xt_bl_zb1_ uses UART0 as default serial port.  It is connected to
USB Serial converter and port is used for both program and console.


Programming and Debugging
*************************

BL Flash tool
=============

The BL702 have a ROM bootloader that allows users to flash the device by serial port.
There are some tools available at internet and this will describe one of them.

#. Install blflash.

#. Test

   .. code-block:: console

      $ blflash -V

   It will print blflash version

   .. code-block:: console

      $ blflash 0.3.3

Samples
=======

#. Build the Zephyr kernel and the :ref:`hello_world` sample application:

   .. zephyr-app-commands::
      :zephyr-app: samples/hello_world
      :board: xt_bl_zb1
      :goals: build
      :compact:

#. To flash an image using blflash runner:

   #. Pull up GPIO28

   #. Pull down EN for a short time (or reset power)

   #. Leave GPIO28 floating again

   .. code-block:: console

      west flash -r blflash

#. Run your favorite terminal program to listen for output. Under Linux the
   terminal should be :code:`/dev/ttyUSB0`. For example:

   .. code-block:: console

      $ minicom -D /dev/ttyUSB0 -o

   The -o option tells minicom not to send the modem initialization
   string. Connection should be configured as follows:

      - Speed: 115200
      - Data: 8 bits
      - Parity: None
      - Stop bits: 1

   Then, press and release EN button

.. _Bouffalo Lab BL702 MCU Website:
	https://en.bouffalolab.com/product/?type=detail&id=8

.. _Bouffalo Lab BL602 MCU Datasheet:
	https://github.com/bouffalolab/bl_docs/tree/main/BL702_DS/enn

.. _Bouffalo Lab Development Zone:
	https://dev.bouffalolab.com/home?id=guest

