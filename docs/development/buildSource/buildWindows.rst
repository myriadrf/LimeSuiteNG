Build from source on Windows
============================

Prerequisites
-------------

Required components to compile LimeSuiteNG project:

- C++ compiler (GCC, Clang, MSVC)
- `CMake`_

Optional components that add specific functionality:

- `wxWidgets`_ : allows graphical user interface
- `SoapySDR`_ : allows building of limesuiteng plugin for SoapySDR

Compilation
-----------

In the root directory of the repository run these commands:

.. code-block:: bash

  cmake .. -B build
  cmake --build build

After a successful compilation the resulting binaries are placed in the ``build/bin/`` directory
located in the root directory of the repository.

Installing the built software
-----------------------------

Continuing on from the previous command block, in the ``build`` folder, execute:

.. code-block:: bash

    cmake build . --target install

.. _`CMake`: https://cmake.org/
.. _`wxWidgets`: https://www.wxwidgets.org/
.. _`SoapySDR`: https://github.com/pothosware/SoapySDR
.. _`FX3SDK`: https://softwaretools.infineon.com/tools/com.ifx.tb.tool.ezusbfx3sdk
