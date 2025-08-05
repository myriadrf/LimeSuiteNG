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
| Configuration Options | Description                                                                                                                  | Value(s)                            |
+=======================+==============================================================================================================================+=====================================+
| Type                  | Sampled and digitized data type.                                                                                             | [ Complex Float32 | Complex Int16 ] |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Custom config file    |                                                                                                                              | Config file filename                |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Device Handle         | Serial number required to identify selected LimeSDR device. Auto selected when a single LimeSDR device is present in system. | LimeSDR device serial number        |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| LO Frequency          | Local oscilator/center/carrier frequency. See :ref:`dev-supp-list-ref` for supported frequencies of individual boards.       | Frequency value, Hz                 |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Sample Rate           | Sampling rate of received RF signal. See :ref:`dev-supp-list-ref` for supported sample rates of individual boards.           | Integer                             |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| RF Oversampling       | Oversampling rate of received RF signal.                                                                                     | [ auto | x1 | x2 | x4 | x16 | x32 ] |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| NCO offset            | Numerically controled oscilator offset.                                                                                      | Frequency value, Hz                 |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Low Pass Filter       | Low pass filter cutoff frequency.                                                                                            | Frequency value, Hz                 |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| GFIR filter           | General Finite Impulse Response filter cutoff frequency.                                                                     | Frequency value, Hz                 |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Antenna port          |                                                                                                                              |                                     |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Data channel indexes  |                                                                                                                              |                                     |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Gain (dB)             |  Set antenna gain in decibels                                                                                                | Integer, dB                         |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+
| Calibrate DC & IQ     |                                                                                                                              |                                     |
+-----------------------+------------------------------------------------------------------------------------------------------------------------------+-------------------------------------+




-------------------------
LimeSuiteNG Sink block
-------------------------

LimeSuiteNG Sink block is used to set up selected LimeSDR device TX channel. To configure sink block, double click it to open the configuration window. Once configured, block collects digitized data sample stream from other GNURadio blocks and sends data to LimeSDR device for transmission.

.. image:: gnuradio_images/limesuitengSinkBlock.*
