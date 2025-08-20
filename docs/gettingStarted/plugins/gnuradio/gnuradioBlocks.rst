.. _gnuradio-blocks-ref:

=========================================
GNURadio LimeSuiteNG blocks
=========================================

Gnuradio-limesuiteng plugin provides sink and source blocks that can be used in GNURadio GUI to interact with selected LimeSDR devices. Once installed the blocks will appear in GNURadio UI on right side of application window in drop down menu as show in the image below.

.. image:: gnuradio_images/limesuiteBlocks.*

Drag and drop or double click on block title to add it to GNURadio flow graph. Connect the LimeSuiteNG Sink or Source block's input/output to other blocks of your choice.

-------------------------
LimeSuiteNG Source block
-------------------------

LimeSuiteNG Source block is used to set up selected LimeSDR device RX channel. To configure source block, double click it to open the configuration window. Once the GNURadio flow graph is executed, block outputs digitized data sample stream that must be redirected to other GNURadio blocks for further processing or display.

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
| NCO offset               | Numerically controled oscillator frequency offset in Hz.                                                                      |
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



-------------------------
LimeSuiteNG Sink block
-------------------------

LimeSuiteNG Sink block is used to set up selected LimeSDR device TX channel. To configure sink block, double click it to open the configuration window. Once the GNURadio flow graph is executed, block collects digitized data samples from other GNURadio blocks and sends data to LimeSDR device for transmission.

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
| NCO offset              | Numerically controled oscillator frequency offset in Hz.                                                                       |
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
