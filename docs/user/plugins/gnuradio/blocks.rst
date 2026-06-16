.. _gnuradio-blocks-ref:

GNU Radio Blocks
################

The gnuradio-limesuiteng plugin provides sink and source blocks that can be used in GNU Radio to interact with supported devices. Once installed, the blocks will appear in the GNU Radio UI on the right side of the application window in the drop-down menu, as shown in the image below.

.. image:: gnuradio_images/limesuiteBlocks.*

Drag and drop, or double-click a block title to add it to the GNU Radio flow graph. Connect the Lime Suite NG Sink or Source block's input or output to other blocks of your choice.

Source block
************

The Lime Suite NG Source block is used to set up the selected SDR device RX channel. To configure the source block, double-click it to open the configuration window. Once the GNU Radio flow graph is executed, the block outputs a digitised data sample stream that must be redirected to other GNU Radio blocks for further processing or display.

.. image:: gnuradio_images/limesuitengSourceBlock.*

Configurable parameters of General tab are provided in the table below:

+--------------------------+-------------------------------------------------------------------------------------------------------------------------------+
| Configurable parameters  | Description                                                                                                                   |
+==========================+===============================================================================================================================+
| Type                     | Sampled and digitized data type.                                                                                              |
|                          |                                                                                                                               |
+--------------------------+-------------------------------------------------------------------------------------------------------------------------------+
| Custom config file       | LMS7002M .ini configuration file.                                                                                             |
+--------------------------+-------------------------------------------------------------------------------------------------------------------------------+
| Device Handle            | Serial number required to identify selected LimeSDR device. Auto selected when a single LimeSDR device is present in system.  |
+--------------------------+-------------------------------------------------------------------------------------------------------------------------------+
| LO Frequency             | Local oscillator/center/carrier frequency in Hz. See :ref:`dev-supp-list-ref` for supported frequencies of individual boards. |
+--------------------------+-------------------------------------------------------------------------------------------------------------------------------+
| Sample Rate              | Sampling rate of received RF signal. See :ref:`dev-supp-list-ref` for supported sample rates of individual boards.            |
+--------------------------+-------------------------------------------------------------------------------------------------------------------------------+
| RF Oversampling          | ADC decimation compared to IQ sample rate. Specifies required amount of samples to produce a single sample in the output.     |
|                          |                                                                                                                               |
|                          |                                                                                                                               |
|                          |                                                                                                                               |
|                          |                                                                                                                               |
|                          |                                                                                                                               |
|                          |                                                                                                                               |
+--------------------------+-------------------------------------------------------------------------------------------------------------------------------+
| NCO offset               | Numerically controlled oscillator frequency offset in Hz.                                                                      |
+--------------------------+-------------------------------------------------------------------------------------------------------------------------------+
| Low Pass Filter          | Low pass filter cutoff frequency in Hz.                                                                                       |
+--------------------------+-------------------------------------------------------------------------------------------------------------------------------+
| GFIR filter              | General Finite Impulse Response filter cutoff frequency in Hz.                                                                |
+--------------------------+-------------------------------------------------------------------------------------------------------------------------------+
| Antenna port             | By default antenna port is selected automatically based on set LO Frequency option.                                           |
+--------------------------+-------------------------------------------------------------------------------------------------------------------------------+
| Data channel indexes     | Index(es) of antenna(s) to use for data retrieval. Antenna indexes start from 0. To use a single antenna specify index [0].   |
|                          | If selected LimeSDR device has multiple RX antennas, specify indexes of all required antennas. For example: [0, 1, 2] or      |
|                          | [0, 1] (if you want to use only the 2/3 antennas).                                                                            |
+--------------------------+-------------------------------------------------------------------------------------------------------------------------------+
| Gain (dB)                | Generic device chip gain adjustment. Gain range is auto clamped.                                                              |
+--------------------------+-------------------------------------------------------------------------------------------------------------------------------+
| Calibrate DC & IQ        | Calibrates Rx DC & IQ imbalance.                                                                                              |
|                          |                                                                                                                               |
+--------------------------+-------------------------------------------------------------------------------------------------------------------------------+

Configurable parameters of LMS7002M Advanced tab are provided in the table below:

+----------------------------+--------------------------------------------------------+
| Configurable parameters    | Description                                            |
+============================+========================================================+
| LMS7002M Rx Gains override | Overrides pre-programmed RX channel gains options.     |
|                            |                                                        |
+----------------------------+--------------------------------------------------------+
| RX LNA                     | Overrides RX Low Noise Amplifier gain setting.         |
+----------------------------+--------------------------------------------------------+
| RX PGA                     | Overrides RX Programmable Gain Amplifier gain setting. |
+----------------------------+--------------------------------------------------------+
| RX TIA                     | Overrides RX Transimpedance Amplifier gain setting.    |
+----------------------------+--------------------------------------------------------+ 

Sink block
**********

The Lime Suite NG Sink block is used to set up the selected SDR device TX channel. To configure the sink block, double-click it to open the configuration window. Once the GNU Radio flow graph is executed, this block sends data to the SDR device for transmission.

.. image:: gnuradio_images/limesuitengSinkBlock.*

Configurable parameters of General tab are provided in the table below:

+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+
| Configurable parameters | Description                                                                                                                    |
+=========================+================================================================================================================================+
| Type                    | Sampled and digitized data type.                                                                                               |
|                         |                                                                                                                                |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+
| Custom config file      | LMS7002M .ini configuration file.                                                                                              |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+
| Device Handle           | Serial number required to identify selected LimeSDR device. Auto selected when a single LimeSDR device is present in system.   |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+
| LO Frequency            | Local oscillator/center/carrier frequency in Hz. See :ref:`dev-supp-list-ref` for supported frequencies of individual boards.  |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+
| Sample Rate             | Sampling rate of transmission signal. See :ref:`dev-supp-list-ref` for supported sample rates of individual boards.            |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+
| RF Oversampling         | DAC interpolation compared to IQ sample rate. Specifies how many new samples to add between the two nearest input samples.     |
|                         |                                                                                                                                |
|                         |                                                                                                                                |
|                         |                                                                                                                                |
|                         |                                                                                                                                |
|                         |                                                                                                                                |
|                         |                                                                                                                                |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+
| NCO offset              | Numerically controlled oscillator frequency offset in Hz.                                                                       |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+
| Low Pass Filter         | Low pass filter cutoff frequency in Hz.                                                                                        |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+
| GFIR filter             | General Finite Impulse Response filter cutoff frequency in Hz.                                                                 |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+
| Antenna port            | By default antenna port is selected automatically based on set LO Frequency option.                                            |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+
| Data channel indexes    | Index(es) of antenna(s) to use for data transmission. Antenna indexes start from 0. To use a single antenna specify index [0]. |
|                         | If selected LimeSDR device has multiple TX antennas, specify indexes of all required antennas. For example: [0, 1, 2] or       |
|                         | [0, 1] (if you want to use only the 2/3 antennas).                                                                             |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+
| Gain (dB)               | Generic device chip gain adjustment. Gain range is auto clamped.                                                               |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+
| Calibrate DC & IQ       | Calibrates Tx DC & IQ imbalance.                                                                                               |
|                         |                                                                                                                                |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+
