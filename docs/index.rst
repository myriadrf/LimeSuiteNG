Introduction
############

Lime Suite NG is a software suite designed for working with LimeSDR family devices.

Project repository: https://github.com/myriadrf/LimeSuiteNG

Project contents:
  * limesuiteng - C++ library for control of the LimeSDR devices.
  * :doc:`gettingStarted/common/GUI` - graphical user interface for configuring device parameters and inspecting Rx RF data using FFT.
  * :doc:`gettingStarted/common/cliTools`.
  * :doc:`gettingStarted/plugins/index` - LimeSDR devices integration into various software platforms.

See :doc:`gettingStarted/index` guide how to install software and setup devices.


.. _dev-supp-list-ref:

=================
Supported devices
=================

* USB interface:

    - `LimeSDR USB`_
    - `LimeSDR Mini v1`_
    - `LimeSDR Mini v2`_

* PCIe interface (currently PCIe driver available only for Linux):

    - `LimeSDR XTRX`_
    - `LimeSDR X3`_
    - LimeMM X8

* To be done:
    - `LimeRFE`_

.. _LimeSDR USB: https://wiki.myriadrf.org/LimeSDR-USB
.. _LimeSDR Mini v1: https://wiki.myriadrf.org/LimeSDR-Mini
.. _LimeSDR Mini v2: https://limesdr-mini.myriadrf.org/
.. _LimeSDR XTRX: https://limesdr-xtrx.myriadrf.org/
.. _LimeSDR X3: https://limesdr-x3.myriadrf.org/
.. _LimeRFE: https://www.crowdsupply.com/lime-micro/limerf

.. toctree::
   :maxdepth: 2
   :hidden:

   Introduction <self>
   gettingStarted/index
   development/index
   doxygen/index
