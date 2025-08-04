GNURadio
========

LimeSDR devices can be used with GNU Radio on Windows and Linux platforms. Below are links for setting up LimeSuiteNG plugin for gnuradio on different operating systems:

.. toctree:: 
   :maxdepth: 1

   Linux/index
   Windows/index

More about `GNURadio`_.

.. _gnuradio-limesuiteng-example-ref:

====================
Plugin demonstration
====================

Plug in your LimeSDR device(s) into USB port. If multiple LimeSDR devices are pluged in, execute the following command in an active conda environment (or in linux terminal) to retrieve required SDR device serial number:

.. code-block:: bash

   (LimeSuiteNG) D:\LimeSuiteNG\plugins\gr-limesuiteng>limedevice
   Found 1 device(s) :
	0: LimeSDR Mini, media=USB 3.0, addr=0403:601f, serial=1D9EFA3E84B944
	1: LimeSDR-USB, media=USB 3.0, addr=1d50:6108, serial=00090706024F2403

Launch gnuradio in conda environment (or in linux terminal) by executing the following command:

.. code-block:: bash

   (LimeSuiteNG) D:\LimeSuiteNG\plugins\gr-limesuiteng>gnuradio-companion FM_receiver.grc

.. note::
   
   If the ``FM_receiver.grc`` does not open when launching GNURadio, open example from one of the following directories: ``<repo root>\plugin\gr-limesuiteng\examples``, ``<radioconda install dir>\envs\<your env>\Library\share\gnuradio\examples\limesuiteng`` (on Windows) or ``/usr/local/share/gnuradio/examples/limesuiteng`` (on Linux).

If a single LimeSDR device is pluged in, run the example by pressing the play button. The software will automatically detect the LimeSDR device and use it to run the example. If multiple LimeSDR devices are present, to run the example you must enter serial number of appropriate LimeSDR device as shown in figure below.

.. image:: settingUpSdr.png

If the example runs successfully, a pop up window will open as shown below. Adjust RX baseband parameter to frequency of a local radio station and adjust volume and gain parameters inside the pop up window to normalize sound quality and volume.

.. image:: settingUpSdrParams.png

If the audio stream from selected radio station is clear, then the plugin is working correctly. Explore more examples in ``<repo root>\plugin\gr-limesuiteng\examples`` directory.

.. hint::
   Make sure that the device is supported by LimeSuiteNG library. See :ref:`dev-supp-list-ref`.



.. Links
.. _GNURadio: https://www.gnuradio.org/