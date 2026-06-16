Introduction
############

Lime Suite NG is Lime Microsystems' next-generation software suite for the LimeSDR family of software-defined radio (SDR) devices. It provides tools, libraries, and drivers for configuring, managing, and using LimeSDR devices across various platforms and applications.

Project contents:

* **liblimesuiteng**. A C++ library that provides device APIs.
* :doc:`user/common/cli`. Utilities for device management, configuration, and basic use.
* :doc:`user/plugins/index`. Plugins for SDR ecosystem software.
* :doc:`developer/gui_tool/GUI`. A graphical user interface for configuring device parameters and inspecting Rx RF data using FFT.

Refer to the :doc:`user/index` guide for information on getting started with Lime Suite NG, including installation instructions and usage examples.

For building custom applications using the library, refer to the :doc:`developer/index` guide.

.. _dev-supp-list-ref:

=================
Supported devices
=================

* USB:

  * LimeSDR Mini v2
  * LimeSDR Mini v1
  * LimeSDR USB

* PCIe (PCIe driver currently only available for Linux):

  * LimeSDR Micro
  * LimeSDR XTRX
  * LimeSDR X3
  * LimeMM X8

.. toctree::
   :maxdepth: 2
   :hidden:

   Introduction <self>
   User Guide <user/index>
   Developer Guide <developer/index>
   Updating Docs <docs/documentation.rst>
