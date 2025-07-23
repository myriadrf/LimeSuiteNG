GNURadio
========

LimeSDR devices can be used with GNU Radio on Windows and Linux platforms. Below are links for setting up Lime Suite NG plugin for gnuradio on different operating systems:

.. toctree:: 
   :maxdepth: 1

   Linux/index
   Windows/index

More about `GNURadio`_.

.. _gnuradio-limesuiteng-example-ref:

====================
Plugin demonstration
====================

Plug in your limeSDR device into USB port. Execute the following command in an active conda environment (or in linux terminal) to retrieve SDR device serial number:

.. code-block:: bash

   (LimeSuiteNG) D:\LimeSuiteNG\plugins\gr-limesuiteng>limedevice
   Found 1 device(s) :
   0: LimeSDR Mini, addr=0403:601f, serial=00000000000000

Launch gnuradio in conda environment (or in linux terminal) by executing the following command:

.. code-block:: bash

   (LimeSuiteNG) D:\LimeSuiteNG\plugins\gr-limesuiteng>gnuradio-companion

Open ``FM_receiver.grc`` example from repository ``<repo root>\plugin\gr-limesuiteng\examples`` directory. Enter SDR serial number as shown in figure below and run the example.

.. image:: settingUpSdr.png

If the example runs successfully, a pop up window will open as shown below. Adjust RX baseband parameter to frequency of a local radio station and adjust volume and gain parameters to normalize sound quality and volume.

.. image:: settingUpSdrParams.png

If the audio stream from selected radio station is clear, then the plugin is working correctly. Explore more examples in ``<repo root>\plugin\gr-limesuiteng\examples`` directory.

.. hint::
   Make sure that the device is supported by Lime Suite NG library. See :ref:`dev-supp-list-ref`.



.. Links
.. _GNURadio: https://www.gnuradio.org/