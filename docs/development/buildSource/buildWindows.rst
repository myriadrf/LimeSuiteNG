Build from source on Windows
============================

Prerequisites
-------------

Required components to compile LimeSuiteNG project:

#. Visual Studio Build Tools 2022 components:

  #. MSVC v143 - VS 2022
  #. Windows 11 SDK

#. Conda packages:

  #. conda-build
  #. conda-forge-pinning
  #. vs2022_win-64

Optional components that add specific functionality:

- `SoapySDR`_ : allows building of limesuiteng plugin for SoapySDR

Compilation
-----------

Activate your conda enivronment:

.. code-block:: bash

   conda activate -n <environment name>

.. hint::
   Check out radioconda and conda environment set up process.
.. Add reference to radioconda and conda setup

Install conda packages:

.. code-block:: bash

   conda install conda-build conda-forge-pinning vs2022_win-64

Downgrade cmake package:

.. code-block:: bash

   conda install cmake=3.26.4

Restart radioconda prompt and activate your conda environment. This will setup appropriate build environment variables. Clone repository:

.. code-block:: bash

   git clone <repository url>

Enter repository directory, create and enter build directory:

.. code-block:: bash
   
   mkdir build && cd build

Configure LimeSuiteNG library build files:

.. code-block:: bash

   cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="%CONDA_PREFIX%\Library" -DCMAKE_PREFIX_PATH="%CONDA_PREFIX%\Library" -DGR_PYTHON_DIR="%CONDA_PREFIX%\Lib\site-packages" ..

Build library:

.. code-block:: bash

   cmake --build .

Built library is located in ``build\lib`` directory and executables are located in ``build\bin`` directory.
  

Installing the built software
-----------------------------

Optionally can be installed into system, installation requires to be ran with Administrative privileges.

Continuing on from the previous command block, execute:

.. code-block:: bash

   cmake --install .

.. tip::
   To uninstall plugin from conda environment use: ``cmake -P cmake_uninstall.cmake`` command inside the build directory.

.. _`CMake`: https://cmake.org/
.. _`wxWidgets`: https://www.wxwidgets.org/
.. _`SoapySDR`: https://github.com/pothosware/SoapySDR
.. _`FX3SDK`: https://softwaretools.infineon.com/tools/com.ifx.tb.tool.ezusbfx3sdk
