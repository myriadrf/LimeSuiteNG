
.. _cli-tools-ref:

============================
Command line interface tools
============================

.. _`limeDeviceUtilityRef`:

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

Full table of limeConfig utility configuration flags/options:

+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| Configuration flags                 | Description                                                                                               | Option(s)                                    |
+=====================================+===========================================================================================================+==============================================+
| **General configuration flags**                                                                                                                                                                |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| -h, \-\-help                        | Prints list of all posible LimeSDR device configuration options.                                          | \-                                           |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| -d[device], \-\-device=[device]     | Specifies which device to use. Auto selects device if only a single device is present.                    | Device name. See :ref:`limeDeviceUtilityRef` |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| -c[chip], \-\-chips=[chip]          | Selects destination chip/chips.                                                                           | LMS7002M                                     |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| -l[log-level], \-\-log=[log level]  | Log verbosity.                                                                                            | - info                                       |
|                                     |                                                                                                           | - warning                                    |
|                                     |                                                                                                           | - error                                      |
|                                     |                                                                                                           | - verbose                                    |
|                                     |                                                                                                           | - debug                                      |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| -i, \-\-initialize                  | Reset and initialize LimeSDR device.                                                                      | \-                                           |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-refclk=[reference clock]        | Reference clock in Hz.                                                                                    | See :ref:`dev-supp-list-ref`                 |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-samplerate=[sample rate]        | Sampling rate in Hz.                                                                                      | See :ref:`dev-supp-list-ref`                 |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| **Receiver configuration flags**                                                                                                                                                               |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-rxen=[rx enable]                | Enable receiver.                                                                                          | - 0 (Disabled)                               |
|                                     |                                                                                                           | - 1 (Enabled)                                |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-rxlo=[rxlo]                     | Sets receiver center frequency.                                                                           | Frequency in Hz                              |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-rxpath=[antenna name]           | Receiver antenna path.                                                                                    | - NONE                                       |
|                                     |                                                                                                           | - LNAH                                       |
|                                     |                                                                                                           | - LNAL_NC                                    |
|                                     |                                                                                                           | - LNAW                                       |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-rxlpf=[Hz]                      | Sets receiver low pass filter bandwidth.                                                                  | Bandwidth in Hz.                             |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-rxoversample=[value]            | Receiver decimation.                                                                                      | - 1                                          |
|                                     |                                                                                                           | - 2                                          |
|                                     |                                                                                                           | - 4                                          |
|                                     |                                                                                                           | - 8                                          |
|                                     |                                                                                                           | - 16                                         |
|                                     |                                                                                                           | - 32                                         |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-rxtestsignal                    | Enables receiver test signal if available.                                                                | \-                                           |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-rxgain=[dB]                     | Sets receiver gain in dB.                                                                                 | Supported range:                             |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-rxcalibrate                     | Calibrates Rx DC and IQ imbalance.                                                                        | \-                                           |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| **Transmitter configuration flags**                                                                                                                                                            |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-txen=[tx enable]                | Enable transmitter.                                                                                       | - 0 (Disabled)                               |
|                                     |                                                                                                           | - 1 (Enabled)                                |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-txlo=[txlo]                     | Sets transmitter center frequency.                                                                        | Frequency in Hz.                             |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-txpath=[antenna name]           | Transmitter antenna path.                                                                                 | - None                                       |
|                                     |                                                                                                           | - Band1                                      |
|                                     |                                                                                                           | - Band2                                      |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-txlpf=[Hz]                      | Sets transmitter low pass filter bandwidth.                                                               | Bandwidth in Hz.                             |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-txoversample=[value]            | Transmitter interpolation.                                                                                | - 1                                          |
|                                     |                                                                                                           | - 2                                          |
|                                     |                                                                                                           | - 4                                          |
|                                     |                                                                                                           | - 8                                          |
|                                     |                                                                                                           | - 16                                         |
|                                     |                                                                                                           | - 32                                         |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-txtestsignal                    | Enables transmitter test signal if available.                                                             | \-                                           |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-txgain=[dB]                     | Sets transmitter gain in dB.                                                                              | Supported range:                             |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-txcalibrate                     | Calibrate Tx DC and IQ imbalance.                                                                         | \-                                           |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+
| \-\-ini=[path]                      | Path to LMS7002M .ini configuration file to use as a base.                                                | Path to configuration file.                  |
+-------------------------------------+-----------------------------------------------------------------------------------------------------------+----------------------------------------------+

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

	user@computer:~$ limeSPI write --chip LMS7002M --stream=0020fffd
	user@computer:~$ limeSPI read --chip LMS7002M --stream=0020
	0020fffd

limeFLASH
---------

Utility for writing firmware/gateware into device's FLASH memory. The gateware files for each device can be found in their github repositories.

.. warning::
    Be aware that the gateware file must be designed for your specific hardware, otherwise the device will stop functioning and will require JTAG tools to recover. Recovery steps are device specific and described in their documentation.

.. code-block:: bash

	user@computer:~$ limeFLASH --target="FPGA FLASH" flash_programming_file.bin
