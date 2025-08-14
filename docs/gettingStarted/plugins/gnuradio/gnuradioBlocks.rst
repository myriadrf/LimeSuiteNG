.. _gnuradio-blocks-ref:

=========================================
GNURadio LimeSuiteNG blocks
=========================================

Gnuradio-limesuiteng plugin provides sink and source blocks that can be used in GNURadio GUI to interact with selected LimeSDR devices. Once installed the blocks will appear in GNURadio UI on right side of application window in drop down menu as show in the image below.

.. image:: gnuradio_images/limesuiteBlocks.*

Drag and drop or double click on block title to add it to GNURadio block field. Connect the blocks to respective blocks of your choice.

-------------------------
LimeSuiteNG Source block
-------------------------

LimeSuiteNG Source block is used to set up selected LimeSDR device RX channel. To configure source block, double click it to open the configuration window. Once configured, block outputs digitized data sample stream that must be redirected to other GNURadio blocks for further processing or display.

.. image:: gnuradio_images/limesuitengSourceBlock.*

Configuration options of General tab are provided in the table below:

+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Configuration Options | Description                                                                                                                  | Option(s)                           |
+=======================+==============================================================================================================================+=====================================+
| Type                  | Sampled and digitized data type.                                                                                             | - Complex Float32                   |
|                       |                                                                                                                              | - Complex Int16                     |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Custom config file    |                                                                                                                              | Config file filename                |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Device Handle         | Serial number required to identify selected LimeSDR device. Auto selected when a single LimeSDR device is present in system. | LimeSDR device serial number        |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| LO Frequency          | Local oscillator/center/carrier frequency. See :ref:`dev-supp-list-ref` for supported frequencies of individual boards.      | Frequency value, Hz                 |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Sample Rate           | Sampling rate of received RF signal. See :ref:`dev-supp-list-ref` for supported sample rates of individual boards.           | Integer                             |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| RF Oversampling       | Oversampling rate of received RF signal.                                                                                     | - auto                              |
|                       |                                                                                                                              | - x1                                |
|                       |                                                                                                                              | - x2                                |
|                       |                                                                                                                              | - x4                                |
|                       |                                                                                                                              | - x8                                |
|                       |                                                                                                                              | - x16                               |
|                       |                                                                                                                              | - x32                               |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| NCO offset            | Numerically controled oscillator offset.                                                                                     | Frequency value, Hz                 |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Low Pass Filter       | Low pass filter cutoff frequency.                                                                                            | Frequency value, Hz                 |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| GFIR filter           | General Finite Impulse Response filter cutoff frequency.                                                                     | Frequency value, Hz                 |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Antenna port          |                                                                                                                              |                                     |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Data channel indexes  |                                                                                                                              |                                     |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Gain (dB)             | Sets receiver antenna gain in decibels.                                                                                      | Supported range:                    |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Calibrate DC & IQ     | Calibrates Rx DC & IQ imbalance.                                                                                             | - Enabled                           |
|                       |                                                                                                                              | - Disabled                          |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+

Configuration options of LMS7002M Advanced tab are provided in the table below:

+----------------------------+------------------------------------------------------------------------------------------------------------------------------+---------------------------------------------------------+
| Configuration Options      | Description                                                                                                                  | Options(s)                                              |
+============================+==============================================================================================================================+=========================================================+
| LMS7002M Rx Gains override | Overrides limeGUI pre-programmed Rx channel gains options.                                                                   | - Enabled                                               |
|                            |                                                                                                                              | - Disabled                                              |
+----------------------------+------------------------------------------------------------------------------------------------------------------------------+---------------------------------------------------------+
| Rx LNA                     | Overrides Rx Low Noise Amplifier gain setting.                                                                               | Supported gmax gains in dB:                             |
|                            |                                                                                                                              |  - max 0                                                |
|                            |                                                                                                                              |  - max-1                                                |
|                            |                                                                                                                              |  - max-2                                                |
|                            |                                                                                                                              |  - max-3                                                |
|                            |                                                                                                                              |  - max-4                                                |
|                            |                                                                                                                              |  - max-5                                                |
|                            |                                                                                                                              |  - max-6                                                |
|                            |                                                                                                                              |  - max-9                                                |
|                            |                                                                                                                              |  - max-12                                               |
|                            |                                                                                                                              |  - max-15                                               |
|                            |                                                                                                                              |  - max-18                                               |
|                            |                                                                                                                              |  - max-21                                               |
|                            |                                                                                                                              |  - max-24                                               |
|                            |                                                                                                                              |  - max-27                                               |
|                            |                                                                                                                              |  - max-30                                               |
+----------------------------+------------------------------------------------------------------------------------------------------------------------------+---------------------------------------------------------+
| Rx PGA                     | Overrides Rx Programmable Gain Amplifier gain setting.                                                                       | Supported gain range from -12 to 19 dB with 1 dB step.  |
+----------------------------+------------------------------------------------------------------------------------------------------------------------------+---------------------------------------------------------+
| Rx TIA                     | Overrides Rx Transimpedance Amplifier gain setting.                                                                          | Supported gmax gains in dB:                             |
|                            |                                                                                                                              |  - max 0                                                |
|                            |                                                                                                                              |  - max-3                                                |
|                            |                                                                                                                              |  - max-12                                               |
+----------------------------+------------------------------------------------------------------------------------------------------------------------------+---------------------------------------------------------+



-------------------------
LimeSuiteNG Sink block
-------------------------

LimeSuiteNG Sink block is used to set up selected LimeSDR device TX channel. To configure sink block, double click it to open the configuration window. Once configured, block collects digitized data sample stream from other GNURadio blocks and sends data to LimeSDR device for transmission.

.. image:: gnuradio_images/limesuitengSinkBlock.*

Configuration options of General tab are provided in the table below:

+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Configuration Options | Description                                                                                                                  | Options(s)                          |
+=======================+==============================================================================================================================+=====================================+
| Type                  | Sampled and digitized data type.                                                                                             | - Complex Float32                   |
|                       |                                                                                                                              | - Complex Int16                     |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Custom config file    |                                                                                                                              | Config file filename                |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Device Handle         | Serial number required to identify selected LimeSDR device. Auto selected when a single LimeSDR device is present in system. | LimeSDR device serial number        |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| LO Frequency          | Local oscillator/center/carrier frequency. See :ref:`dev-supp-list-ref` for supported frequencies of individual boards.      | Frequency value, Hz                 |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Sample Rate           | Sampling rate of transmission signal. See :ref:`dev-supp-list-ref` for supported sample rates of individual boards.          | Integer                             |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| RF Oversampling       | Oversampling rate of transmission signal.                                                                                    | - auto                              |
|                       |                                                                                                                              | - x1                                |
|                       |                                                                                                                              | - x2                                |
|                       |                                                                                                                              | - x4                                |
|                       |                                                                                                                              | - x8                                |
|                       |                                                                                                                              | - x16                               |
|                       |                                                                                                                              | - x32                               |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| NCO offset            | Numerically controled oscillator offset.                                                                                     | Frequency value, Hz                 |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Low Pass Filter       | Low pass filter cutoff frequency.                                                                                            | Frequency value, Hz                 |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| GFIR filter           | General Finite Impulse Response filter cutoff frequency.                                                                     | Frequency value, Hz                 |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Antenna port          |                                                                                                                              |                                     |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Data channel indexes  |                                                                                                                              |                                     |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Gain (dB)             | Sets transmitter antenna gain in decibels.                                                                                   | Supported range:                    |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Calibrate DC & IQ     | Calibrates Tx DC & IQ imbalance.                                                                                             | - Enabled                           |
|                       |                                                                                                                              | - Disabled                          |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
