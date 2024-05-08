Command line interface tools
============================

limeDevice
----------

Utility to list detected devices and retrieve device information descriptors and status.

.. code-block:: bash

	user@computer:~$ limeDevice --full
	0: LimeXTRX0, media=PCIe, addr=/dev/LimeXTRX0_control, serial=0000000000000000
	Expansion name		: UNSUPPORTED
	Firmware version	: 1
	Gateware version	: 1
	Gateware revision	: 0
	Gateware target board	: LimeSDR XTRX
	Hardware version	: 0
	Protocol version	: 1
	Serial number		: 2748
	SPI slave devices	:
				  FPGA
				  LMS7002M
	Memory devices		:
				  FPGA FLASH
	GPS Lock:
		GPS - Undefined
		Glonass - Undefined
		Galileo - Undefined
		Beidou - Undefined

limeConfig
----------

Utility to configure generic software defined radio parameters.

.. code-block:: bash

	user@computer:~$ limeConfig --initialize --samplerate=20e6 --rxen=1 --rxlo=2.442e9 --rxpath=LNAW --rxlpf=120e6

limeTRX
-------

Utility for receiving and/or transmitting RF samples.

.. note::
    The device has to be already in configured working state, otherwise it will not stream samples.

.. code-block:: bash

	user@computer:~$ limeTRX --fft --output="receivedSamples.wfm" --samplesCount=20000000

limeSPI
-------

Utility for reading/writing device's SPI registers

.. code-block:: bash

	user@computer:~$ limeSPI --chip LMS7002M --write=0020fffd
	user@computer:~$ limeSPI --chip LMS7002M --read=0020
	0020fffd

limeFLASH
---------

Utility for writing firmware/gateware into device's FLASH memory. The gateware files for each device can be found in their github repositories.

.. warning::
    Be aware that the gateware file must be designed for your specific hardware, otherwise the device will stop functioning and will require JTAG tools to recover. Recovery steps are device specific and described in their documentation.

.. code-block:: bash

	user@computer:~$ limeFLASH --target="FPGA FLASH" flash_programming_file.bin