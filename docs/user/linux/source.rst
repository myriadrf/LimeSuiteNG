Build from Source
#################

Prerequisites
*************

The following components are required to compile Lime Suite NG from source:

- C++ compiler (GCC, Clang)
- `CMake`_

They can be installed by: ``sudo apt-get install build-essential cmake``

Optional packages that can be installed to enable specific functionality:

- `libusb-1.0-dev`_. Enables USB interface device support.
- `wxWidgets`_. Enables the graphical user interface.
- `libsoapysdr-dev`_. Enables building the Lime Suite NG plugin for SoapySDR.
- ``linux-kernel-headers``. Enables building the PCIe driver module for the Linux kernel.
- `gnuradio-dev`_. Enables building the plugin for GNU Radio.

.. note::
   If you are on Ubuntu 20.04 or newer, or Debian 11 or newer, a script ``install_dependencies.sh`` is available to install all required dependencies.

Compilation
***********

In the root directory of the repository run these commands:

.. code-block:: bash

  sudo ./install_dependencies.sh # Optional, installs dependencies
  mkdir build && cd build
  cmake ..
  make

.. note::
   Append ``-j <number>`` to the ``make`` command to use multiple CPU cores and reduce compilation time. To use all available CPU cores, specify ``-j $(nproc)``.

After successful compilation, the resulting binaries are placed in the ``build/bin/`` directory in the repository root.

Installation
************

From the ``build`` folder, execute:

.. code-block:: bash

    sudo make install
    sudo ldconfig

.. _`CMake`: https://cmake.org/
.. _`wxWidgets`: https://wxwidgets.org/
.. _`libusb-1.0-dev`: https://libusb.info/
.. _`libsoapysdr-dev`: https://github.com/pothosware/SoapySDR
.. _`gnuradio-dev`: https://www.gnuradio.org/
