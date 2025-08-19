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

+--------------------------+------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Configurable parameters  | Description                                                                                                                  | Option(s)                    |
+==========================+==============================================================================================================================+==============================+
| Type                     | Sampled and digitized data type.                                                                                             | - Complex Float32            |
|                          |                                                                                                                              | - Complex Int16              |
+--------------------------+------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Custom config file       | LMS7002M .ini configuration file.                                                                                            | Absolute path to .ini file.  |
+--------------------------+------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Device Handle            | Serial number required to identify selected LimeSDR device. Auto selected when a single LimeSDR device is present in system. | LimeSDR device serial number |
+--------------------------+------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| LO Frequency             | Local oscillator/center/carrier frequency. See :ref:`dev-supp-list-ref` for supported frequencies of individual boards.      | Frequency value, Hz          |
+--------------------------+------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Sample Rate              | Sampling rate of received RF signal. See :ref:`dev-supp-list-ref` for supported sample rates of individual boards.           | Integer                      |
+--------------------------+------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| RF Oversampling          | ADC decimation compared to IQ sample rate. Specifies required amount of samples to produce a single sample in the output.    | - auto                       |
|                          |                                                                                                                              | - x1                         |
|                          |                                                                                                                              | - x2                         |
|                          |                                                                                                                              | - x4                         |
|                          |                                                                                                                              | - x8                         |
|                          |                                                                                                                              | - x16                        |
|                          |                                                                                                                              | - x32                        |
+--------------------------+------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| NCO offset               | Numerically controled oscillator offset.                                                                                     | Frequency value, Hz          |
+--------------------------+------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Low Pass Filter          | Low pass filter cutoff frequency.                                                                                            | Frequency value, Hz          |
+--------------------------+------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| GFIR filter              | General Finite Impulse Response filter cutoff frequency.                                                                     | Frequency value, Hz          |
+--------------------------+------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Antenna port             | By default antenna port is selected automatically based on set LO Frequency option.                                          | \-                           |
+--------------------------+------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Data channel indexes     | Index(es) of antenna(s) to use for data retrieval. Antenna indexes start from 0. To use a single antenna specify index [0].  | For SISO only index [0].     |
|                          | If selected LimeSDR device has multiple RX antennas, specify indexes of all required antennas. For example: [0, 1, 2] or     |                              |
|                          | [0, 1] (if you want to use only the 2/3 antennas).                                                                           | For MIMO [0, 1, ...].        |
+--------------------------+------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Gain (dB)                | Generic device chip gain adjustment. Gain range is auto clamped.                                                             | In dB.                       |
+--------------------------+------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Calibrate DC & IQ        | Calibrates Rx DC & IQ imbalance.                                                                                             | - Enabled                    |
|                          |                                                                                                                              | - Disabled                   |
+--------------------------+------------------------------------------------------------------------------------------------------------------------------+------------------------------+

Configurable parameters of LMS7002M Advanced tab are provided in the table below:

+----------------------------+------------------------------------------------------------+---------------------------------------------------------+
| Configurable parameters    | Description                                                | Options(s)                                              |
+============================+============================================================+=========================================================+
| LMS7002M Rx Gains override | Overrides limeGUI pre-programmed Rx channel gains options. | - Enabled                                               |
|                            |                                                            | - Disabled                                              |
+----------------------------+------------------------------------------------------------+---------------------------------------------------------+
| Rx LNA                     | Overrides Rx Low Noise Amplifier gain setting.             | Supported gmax gains in dB:                             |
|                            |                                                            |  - max 0                                                |
|                            |                                                            |  - max-1                                                |
|                            |                                                            |  - max-2                                                |
|                            |                                                            |  - max-3                                                |
|                            |                                                            |  - max-4                                                |
|                            |                                                            |  - max-5                                                |
|                            |                                                            |  - max-6                                                |
|                            |                                                            |  - max-9                                                |
|                            |                                                            |  - max-12                                               |
|                            |                                                            |  - max-15                                               |
|                            |                                                            |  - max-18                                               |
|                            |                                                            |  - max-21                                               |
|                            |                                                            |  - max-24                                               |
|                            |                                                            |  - max-27                                               |
|                            |                                                            |  - max-30                                               |
+----------------------------+------------------------------------------------------------+---------------------------------------------------------+
| Rx PGA                     | Overrides Rx Programmable Gain Amplifier gain setting.     | Supported gain range from -12 to 19 dB with 1 dB step.  |
+----------------------------+------------------------------------------------------------+---------------------------------------------------------+
| Rx TIA                     | Overrides Rx Transimpedance Amplifier gain setting.        | Supported gmax gains in dB:                             |
|                            |                                                            |  - max 0                                                |
|                            |                                                            |  - max-3                                                |
|                            |                                                            |  - max-12                                               |
+----------------------------+------------------------------------------------------------+---------------------------------------------------------+



-------------------------
LimeSuiteNG Sink block
-------------------------

LimeSuiteNG Sink block is used to set up selected LimeSDR device TX channel. To configure sink block, double click it to open the configuration window. Once the GNURadio flow graph is executed, block collects digitized data samples from other GNURadio blocks and sends data to LimeSDR device for transmission.

.. image:: gnuradio_images/limesuitengSinkBlock.*

Configurable parameters of General tab are provided in the table below:

+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Configurable parameters | Description                                                                                                                    | Options(s)                   |
+=========================+================================================================================================================================+==============================+
| Type                    | Sampled and digitized data type.                                                                                               | - Complex Float32            |
|                         |                                                                                                                                | - Complex Int16              |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Custom config file      | LMS7002M .ini configuration file.                                                                                              | Absolute path to .ini file.  |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Device Handle           | Serial number required to identify selected LimeSDR device. Auto selected when a single LimeSDR device is present in system.   | LimeSDR device serial number |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| LO Frequency            | Local oscillator/center/carrier frequency. See :ref:`dev-supp-list-ref` for supported frequencies of individual boards.        | Frequency value, Hz          |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Sample Rate             | Sampling rate of transmission signal. See :ref:`dev-supp-list-ref` for supported sample rates of individual boards.            | Integer                      |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| RF Oversampling         | DAC interpolation compared to IQ sample rate. Specifies how many new samples to add between the two nearest input samples.     | - auto                       |
|                         |                                                                                                                                | - x1                         |
|                         |                                                                                                                                | - x2                         |
|                         |                                                                                                                                | - x4                         |
|                         |                                                                                                                                | - x8                         |
|                         |                                                                                                                                | - x16                        |
|                         |                                                                                                                                | - x32                        |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| NCO offset              | Numerically controled oscillator offset.                                                                                       | Frequency value, Hz          |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Low Pass Filter         | Low pass filter cutoff frequency.                                                                                              | Frequency value, Hz          |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| GFIR filter             | General Finite Impulse Response filter cutoff frequency.                                                                       | Frequency value, Hz          |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Antenna port            | By default antenna port is selected automatically based on set LO Frequency option.                                            | \-                           |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Data channel indexes    | Index(es) of antenna(s) to use for data transmission. Antenna indexes start from 0. To use a single antenna specify index [0]. | For SISO only index [0].     |
|                         | If selected LimeSDR device has multiple TX antennas, specify indexes of all required antennas. For example: [0, 1, 2] or       |                              |
|                         | [0, 1] (if you want to use only the 2/3 antennas).                                                                             | For MIMO [0, 1, ...].        |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Gain (dB)               | Generic device chip gain adjustment. Gain range is auto clamped.                                                               | In dB.                       |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+------------------------------+
| Calibrate DC & IQ       | Calibrates Tx DC & IQ imbalance.                                                                                               | - Enabled                    |
|                         |                                                                                                                                | - Disabled                   |
+-------------------------+--------------------------------------------------------------------------------------------------------------------------------+------------------------------+
