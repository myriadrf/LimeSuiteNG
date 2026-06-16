
.. _cli-ref:

CLI Tools
=========

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

Utility to configure generic software-defined radio parameters.

.. code-block:: bash

	user@computer:~$ limeConfig --initialize --samplerate=20e6 --rxen=1 --rxlo=2.442e9 --rxpath=LNAW --rxlpf=120e6

Full table of limeConfig utility configuration flags:

+-------------------------------------+----------------------------------------------------------------------------------------+
| Configuration flags                 | Description                                                                            |
+=====================================+========================================================================================+
| **General configuration flags**                                                                                              |
+-------------------------------------+----------------------------------------------------------------------------------------+
| -h, \-\-help                        | Prints list of all possible LimeSDR device configuration flags.                        |
+-------------------------------------+----------------------------------------------------------------------------------------+
| -d[device],                         | Specifies which device to use. Auto selects device if only a single device is present. |
|                                     |                                                                                        |
| \-\-device=[device]                 |                                                                                        |
+-------------------------------------+----------------------------------------------------------------------------------------+
| -c[chip],                           | Selects destination chip to update its configuration. Chip indexes start from 0.       |
|                                     |                                                                                        |
| \-\-chips=[chip]                    |                                                                                        |
+-------------------------------------+----------------------------------------------------------------------------------------+
| -l[log-level],                      | Log verbosity: info, warning, error, verbose, debug.                                   |
|                                     |                                                                                        |
| \-\-log=[log level]                 |                                                                                        |
+-------------------------------------+----------------------------------------------------------------------------------------+
| -i,                                 | Reset and initialize LimeSDR device.                                                   |
|                                     |                                                                                        |
| \-\-initialize                      |                                                                                        |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-refclk=[reference clock]        | Selected LimeSDR board reference clock in Hz. Set this option only if the actual       |
|                                     | reference clock of the board is changed or modified.                                   |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-samplerate=[sample rate]        | Sampling rate in Hz.                                                                   |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-ini=[path]                      | Path to LMS7002M .ini configuration file to use as a base.                             |
+-------------------------------------+----------------------------------------------------------------------------------------+
| **Receiver configuration flags**                                                                                             |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-rxen=[rx enable]                | Enable receiver.                                                                       |
|                                     |                                                                                        |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-rxlo=[rxlo]                     | Sets receiver center frequency in Hz.                                                  |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-rxpath=[antenna name]           | Receiver antenna path.                                                                 |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-rxlpf=[Hz]                      | Sets receiver low pass filter bandwidth in Hz.                                         |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-rxoversample=[value]            | Receiver sample decimation 1, 2, 4, 8, ...                                             |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-rxtestsignal                    | Enables receiver test signal if available.                                             |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-rxgain=[dB]                     | Sets generic device RX gain in dB. Gain range is auto clamped.                         |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-rxcalibrate                     | Calibrates RX DC and IQ imbalance.                                                     |
+-------------------------------------+----------------------------------------------------------------------------------------+
| **Transmitter configuration flags**                                                                                          |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-txen=[tx enable]                | Enable transmitter.                                                                    |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-txlo=[txlo]                     | Sets transmitter center frequency in Hz.                                               |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-txpath=[antenna name]           | Transmitter antenna path.                                                              |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-txlpf=[Hz]                      | Sets transmitter low pass filter bandwidth in Hz.                                      |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-txoversample=[value]            | Transmitter sample interpolation.                                                      |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-txtestsignal                    | Enables transmitter test signal if available.                                          |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-txgain=[dB]                     | Sets generic device TX gain in dB. Gain range is auto clamped.                         |
+-------------------------------------+----------------------------------------------------------------------------------------+
| \-\-txcalibrate                     | Calibrates TX DC and IQ imbalance.                                                     |
+-------------------------------------+----------------------------------------------------------------------------------------+

limeTRX
-------

Utility for receiving and transmitting RF samples.

.. note::
    The device must be in a configured working state; otherwise, it will not stream samples.

.. code-block:: bash

	user@computer:~$ limeTRX --fft --output="receivedSamples.wfm" --samplesCount=20000000

Full table of limeTRX utility configuration flags:

+---------------------------------+----------------------------------------------------------------------------------------+
| Configuration flags             | Description                                                                            |
+=================================+========================================================================================+
| **General configuration flags**                                                                                          |
+---------------------------------+----------------------------------------------------------------------------------------+
| -h, \-\-help                    | Prints list of all possible limeTRX utility flags.                                     |
+---------------------------------+----------------------------------------------------------------------------------------+
| -d[name],                       | Specifies which device to use. Auto selects device if only a single device is present. |
|                                 |                                                                                        |
| \-\-device=[name]               |                                                                                        |
+---------------------------------+----------------------------------------------------------------------------------------+
| -c[index],                      | Specify chip(s) for streaming using chip index, or index list for aggregation          |
|                                 | [0,1, ...]. Index starts from 0.                                                       |
| \-\-chip=[index]                |                                                                                        |
+---------------------------------+----------------------------------------------------------------------------------------+
| -i[file path],                  | Samples to transmit from selected waveform file.                                       |
|                                 |                                                                                        |
| \-\-input[file path]            |                                                                                        |
+---------------------------------+----------------------------------------------------------------------------------------+
| -o[file path],                  | Waveform file to store received samples.                                               |
|                                 |                                                                                        |
| \-\-output[file path]           |                                                                                        |
+---------------------------------+----------------------------------------------------------------------------------------+
| \-\-looptx                      | Transmits input file contents in a loop.                                               |
+---------------------------------+----------------------------------------------------------------------------------------+
| -s[sample count],               | Number of samples to receive.                                                          |
|                                 |                                                                                        |
| \-\-sampleCount[sample count]   |                                                                                        |
+---------------------------------+----------------------------------------------------------------------------------------+
| \-\-time=[ms]                   | Duration of sampling in RX channel in milliseconds.                                    |
+---------------------------------+----------------------------------------------------------------------------------------+
| \-\-fft                         | Display RX channel FFT plot.                                                           |
+---------------------------------+----------------------------------------------------------------------------------------+
| -l[verbosity],                  | Log verbosity: info, warning, error, verbose, debug.                                   |
|                                 |                                                                                        |
| \-\-log=[verbosity]             |                                                                                        |
+---------------------------------+----------------------------------------------------------------------------------------+
| \-\-mimo=[channel count]        | Use multiple channels to receive and transmit. Default channel count is 1.             |
+---------------------------------+----------------------------------------------------------------------------------------+
| \-\-repeater=[delaySamples]     | Retransmit received samples with a delay.                                              |
+---------------------------------+----------------------------------------------------------------------------------------+
| \-\-linkFormat=[format]         | Data transfer format. Default: I12. Supported formats: I16, I12.                       |
+---------------------------------+----------------------------------------------------------------------------------------+
| \-\-syncPPS                     | Start sampling on the next PPS.                                                        |
+---------------------------------+----------------------------------------------------------------------------------------+
| \-\-rxSamplesInPacket=[samples] | Number of IQ samples in a single RX packet. Recommended to use only for debugging.     |
+---------------------------------+----------------------------------------------------------------------------------------+
| \-\-txSamplesInPacket=[samples] | Number of IQ samples in a single TX packet. Recommended to use only for debugging.     |
+---------------------------------+----------------------------------------------------------------------------------------+
| \-\-rxPacketsInBatch=[packets]  | Number of packets in a single RX data transfer. Recommended to use only for debugging. |
+---------------------------------+----------------------------------------------------------------------------------------+
| \-\-txPacketsInBatch=[packets]  | Number of packets in a single TX data transfer. Recommended to use only for debugging. |
+---------------------------------+----------------------------------------------------------------------------------------+

limeSPI
-------

Utility for reading and writing device SPI registers

.. code-block:: bash

	user@computer:~$ limeSPI write --chip LMS7002M --stream=0020fffd
	user@computer:~$ limeSPI read --chip LMS7002M --stream=0020
	0020fffd

Full table of limeSPI utility commands and configuration flags:

+----------------------+---------------------------------------------------------------------------------------+
| Configuration flags  | Description                                                                           |
+======================+=======================================================================================+
| -h, \-\-help         | Prints list of all possible limeSPI utility flags.                                    |
+----------------------+---------------------------------------------------------------------------------------+
| **Commands**                                                                                                 |
+----------------------+---------------------------------------------------------------------------------------+
| read                 | Register reading operation.                                                           |
+----------------------+---------------------------------------------------------------------------------------+
| write                | Register writing operation.                                                           |
+----------------------+---------------------------------------------------------------------------------------+
| **Arguments**                                                                                                |
+----------------------+---------------------------------------------------------------------------------------+
| -d[name],            | Specifies which device to use. Auto selects device if only a single device is present.|
|                      |                                                                                       |
| \-\-device=[name]    |                                                                                       |
+----------------------+---------------------------------------------------------------------------------------+
| -c[name],            | Selects destination chip by its name.                                                 |
|                      |                                                                                       |
| \-\-chip=[name]      |                                                                                       |
+----------------------+---------------------------------------------------------------------------------------+
| **Data options**                                                                                             |
+----------------------+---------------------------------------------------------------------------------------+
| \-f[file path],      | File with a sequence of register address and data bytes for write command or a file   |
|                      | with a sequence of register address bytes for read command. See the stream flag for   |
|                      | the byte structure required for write and read commands.                              |
| \-\-file=[file path] |                                                                                       |
+----------------------+---------------------------------------------------------------------------------------+
| \-s[stream],         | Direct data stream.                                                                   |
|                      |                                                                                       |
| \-\-stream=[stream]  | Write command format:                                                                 |
|                      |                                                                                       |
|                      | - Register address 2x bytes and 2x data bytes \-\-stream=[reg addr | data]            |
|                      |                                                                                       |
|                      | Read command format:                                                                  |
|                      |                                                                                       |
|                      | - Register address 2x bytes \-\-stream=[reg addr]                                     |
+----------------------+---------------------------------------------------------------------------------------+

limeFLASH
---------

Utility for writing firmware and gateware into device FLASH memory. The gateware files for each device can be found in their GitHub repositories.

.. warning::
    Be aware that the gateware file must be designed for your specific hardware, otherwise the device will stop functioning and will require JTAG tools to recover. Recovery steps are device specific and described in their documentation.

.. code-block:: bash

	user@computer:~$ limeFLASH --target="FPGA FLASH" flash_programming_file.bin

Full table of limeFLASH utility configuration flags:

+---------------------+--------------------------------------------------------------------------------------------------------------+
| Configuration flags | Description                                                                                                  |
+=====================+==============================================================================================================+
| -h, \-\-help        | Prints list of all possible limeFLASH utility flags.                                                         |
+---------------------+--------------------------------------------------------------------------------------------------------------+
| -d[name],           | Specifies which device to use. Auto selects device if only a single device is present.                       |
|                     |                                                                                                              |
| \-\-device=[name]   |                                                                                                              |
+---------------------+--------------------------------------------------------------------------------------------------------------+
| -t[TARGET],         | Specifies which target to use. See the ``\-l, \-\-list`` flag.                                               |
|                     |                                                                                                              |
| \-\-target=[TARGET] |                                                                                                              |
+---------------------+--------------------------------------------------------------------------------------------------------------+
| -l, \-\-list        | List available device targets.                                                                               |
+---------------------+--------------------------------------------------------------------------------------------------------------+
| "\-\-"              | Can be used to terminate flag options and force all following arguments to be treated as positional options. |
+---------------------+--------------------------------------------------------------------------------------------------------------+


limeOEM
-------

Utility for device testing and custom device-specific operations.

.. warning::
	This software utility is designed for developers. Users should only use ``--test`` and optionally (if multiple devices are plugged into the system) the ``--device`` flag to run basic device tests.

.. note::
	Currently, limeOEM can only perform tests for the LimeSDR XTRX device.

.. code-block:: bash

	user@computer:~$ limeOEM --test

Full table of limeOEM utility configuration flags:

+-----------------------------------+-------------------------------------------------------------------------------------------+
| Configuration flags               | Description                                                                               |
+===================================+===========================================================================================+
| -h, \-\-help                      | Prints list of all possible limeOEM utility flags.                                        |
+-----------------------------------+-------------------------------------------------------------------------------------------+
| -d[device],                       | Specifies which device to use. Auto selects device if only a single device is present.    |
|                                   |                                                                                           |
| \-\-device=[device]               |                                                                                           |
+-----------------------------------+-------------------------------------------------------------------------------------------+
| -l[level],                        | Log verbosity: info, warning, error, verbose, debug.                                      |
|                                   |                                                                                           |
| \-\-log=[level]                   |                                                                                           |
+-----------------------------------+-------------------------------------------------------------------------------------------+
| -o[path],                         | File to append test results. File must be created prior to utility launch.                |
|                                   |                                                                                           |
| \-\-output=[path]                 |                                                                                           |
+-----------------------------------+-------------------------------------------------------------------------------------------+
| \-\-write-serial-number=[decimal] | One time programmable serial number to be written to device.                              |
+-----------------------------------+-------------------------------------------------------------------------------------------+
| \-\-test                          | Run tests to check device functionality.                                                  |
+-----------------------------------+-------------------------------------------------------------------------------------------+
| \-\-version                       | Prints limeOEM version.                                                                   |
+-----------------------------------+-------------------------------------------------------------------------------------------+
| \-\-interactive                   | Wait for user input after each test.                                                      |
+-----------------------------------+-------------------------------------------------------------------------------------------+


limeCSR
-------

Utility for reading and writing device configuration space registers.

.. code-block:: bash

	user@computer:~$ limeCSR write --stream=100cc000007000801a0000e00000feed,0000000000700080000000e00000feed
	user@computer:~$ limeCSR read --stream=--stream=100cc00000700080,0000000000700080

Table of commands and arguments:

+----------------------+---------------------------------------------------------------------------------------+
| Configuration flags  | Description                                                                           |
+======================+=======================================================================================+
| -h, \-\-help         | Prints list of all possible limeCSR utility flags.                                    |
+----------------------+---------------------------------------------------------------------------------------+
| **Commands**                                                                                                 |
+----------------------+---------------------------------------------------------------------------------------+
| read                 | Register reading operation.                                                           |
+----------------------+---------------------------------------------------------------------------------------+
| write                | Register writing operation.                                                           |
+----------------------+---------------------------------------------------------------------------------------+
| **Arguments**                                                                                                |
+----------------------+---------------------------------------------------------------------------------------+
| -d[name],            | Specifies which device to use. Auto selects device if only a single device is present.|
|                      |                                                                                       |
| \-\-device=[name]    |                                                                                       |
+----------------------+---------------------------------------------------------------------------------------+
| **Data options**                                                                                             |
+----------------------+---------------------------------------------------------------------------------------+
| \-f[file path],      | Stream from file. For word format in file, see --stream flag description.             |
|                      |                                                                                       |
| \-\-file=[file path] |                                                                                       |
+----------------------+---------------------------------------------------------------------------------------+
| \-s[stream],         | Stream from CLI. For write command, provide 16 byte word (address + data) in hex      |
|                      | format without prefix '0x'. For read command, provide 8 byte word (address) in hex    |
| \-\-stream=[stream]  | format without prefix '0x'. Multiple words can be separated using comma. Files also   |
|                      | support space and newline delimiters.                                                 |
+----------------------+---------------------------------------------------------------------------------------+

limePPSDO
---------

Utility for controlling and monitoring the PPS Disciplined Oscillator. Currently supported PPS sources: GPS.

.. code-block:: bash

	user@computer$ limePPSDO --enable --clk-freq=10 --ppm=0.1 --check
	user@computer$ limePPSDO --check --num=50 --banner=20 --delay=5
	user@computer$ limePPSDO --disable --reset-delay=5

Table of commands and arguments:

+-----------------------+---------------------------------------------------------------------------------------+
| Configuration flags   | Description                                                                           |
+=======================+=======================================================================================+
| -h, \-\-help          | Prints list of all possible limePPSDO utility flags.                                  |
+-----------------------+---------------------------------------------------------------------------------------+
| **Commands**                                                                                                  |
+-----------------------+---------------------------------------------------------------------------------------+
| check                 | Run monitoring mode.                                                                  |
+-----------------------+---------------------------------------------------------------------------------------+
| dump                  | Dump registers.                                                                       |
+-----------------------+---------------------------------------------------------------------------------------+
| reset                 | Reset PPSDO.                                                                          |
+-----------------------+---------------------------------------------------------------------------------------+
| enable                | Configure and enable PPSDO.                                                           |
+-----------------------+---------------------------------------------------------------------------------------+
| disable               | Disable PPSDO.                                                                        |
+-----------------------+---------------------------------------------------------------------------------------+
| **Arguments**                                                                                                 |
+-----------------------+---------------------------------------------------------------------------------------+
| -l[name],             | Enables additional device, API and limePPSDO app log output. Log verbosity: info,     |
|                       | warning, error, verbose, debug. Log level \'info\' prints intermediate calculations.  |
| \-\-log=[name]        | Log level \'debug\' prints detailed CSR register R/W operations.                      |
+-----------------------+---------------------------------------------------------------------------------------+
| \-\-device=[name]     | Specifies which device to use. Auto selects device if only a single device is present.|
+-----------------------+---------------------------------------------------------------------------------------+
| -n[iter],             | Number of iterations (With \-\-check \-\-num=0 for infinite loop; for \-\-dump:       |
|                       | default 1 if not specified). Default: 0                                               |
| \-\-num=[iter]        |                                                                                       |
+-----------------------+---------------------------------------------------------------------------------------+
| -d[time],             | Delay between iterations (seconds, for \-\-check and --dump). Default: 1s             |
|                       |                                                                                       |
| \-\-delay=[time]      |                                                                                       |
+-----------------------+---------------------------------------------------------------------------------------+
| -b[interval],         | Banner repeat interval (for \-\-check). Default: every 10 iterations of status        |
|                       | message.                                                                              |
| \-\-banner=[interval] |                                                                                       |
+-----------------------+---------------------------------------------------------------------------------------+
| \-r[time]             | Delay after disable before re-enable (seconds, for \-\-reset). Default: 2s            |
|                       |                                                                                       |
| \-\-reset-delay[time] |                                                                                       |
+-----------------------+---------------------------------------------------------------------------------------+
| \-c[MHz],             | Clock frequency in MHz.                                                               |
|                       |                                                                                       |
| \-\-clk-freq=[MHz]    |                                                                                       |
+-----------------------+---------------------------------------------------------------------------------------+
| \-p[parts],           | Tolerance in ppm.                                                                     |
|                       |                                                                                       |
| \-\-ppm=[parts]       |                                                                                       |
+-----------------------+---------------------------------------------------------------------------------------+