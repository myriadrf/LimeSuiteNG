Build from source on Linux
==========================

Prerequisites
-------------

Required components to compile LimeSuiteNG project:

- C++ compiler (GCC, Clang)
- `CMake`_

They can be installed by: ``sudo apt-get install build-essential cmake``

Optional packages that can be installed to enable specific functionality:

- `libusb-1.0-dev`_ : enables support of USB interface devices
- `wxWidgets`_ : enables graphical user interface
- `libsoapysdr-dev`_ : enables building of limesuiteng plugin for SoapySDR
- `linux-kernel-headers` : enables building of PCIe driver module for Linux kernel.
- `gnuradio-dev`_ : enables building of plugin for GNU Radio.

.. note::
    If you are on Ubuntu 20.04 or newer or Debian 11 or newer,
    there also exists a script ``install_dependencies.sh`` to install all the needed dependencies.

Compilation
-----------

In the root directory of the repository run these commands:

.. code-block:: bash

  sudo ./install_dependencies.sh # Optional, instals dependencies
  mkdir build && cd build
  cmake ..
  make

.. note::
    Append ``-j <number>`` to the ``make`` command to use more than one CPU core to greatly speed up compilation times.
    To automatically use all the cores available on your CPU use ``-j $(nproc)``.

After a successful compilation the resulting binaries are placed in the ``build/bin/`` directory
located in the root directory of the repository.

Installing the built software
-----------------------------

Continuing on from the previous command block, in the ``build`` folder, execute:

.. code-block:: bash

    sudo make install
    sudo ldconfig

.. _`CMake`: https://cmake.org/
.. _`wxWidgets`: https://www.wxwidgets.org/
.. _`libusb-1.0-dev`: https://libusb.info/
.. _`libsoapysdr-dev`: https://github.com/pothosware/SoapySDR
.. _`gnuradio-dev`: https://www.gnuradio.org/
