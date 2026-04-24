Introduction
############

Lime Suite NG is Lime Microsystems' next generation software suite for the LimeSDR family of software-defined radio (SDR) devices. It provides a comprehensive set of tools, libraries and drivers for configuring, managing, and utilising LimeSDR devices across various platforms and applications.

Project contents:

  * **liblimesuiteng**. C++ library which provides device APIs.
  * :doc:`user/common/cli`. Utilities for device management, configuration and basic use.
  * :doc:`user/plugins/index`. Plugins for SDR ecosystem software.
  * :doc:`developer/gui_tool/GUI` - graphical user interface for configuring device parameters and inspecting Rx RF data using FFT.

See the :doc:`user/index` guide for more information on getting started with Lime Suite NG, including installation instructions and usage examples.

For building custom applications using the library, please refer to the :doc:`developer/index` guide for detailed documentation and resources.

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
