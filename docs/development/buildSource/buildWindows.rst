.. _windows-lib-build-ref:

============================
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
   #. cmake (supported versions by project: 3.15 - 3.31)
   #. ninja

Optional components that add specific functionality:

- `SoapySDR`_ : allows building of limesuiteng plugin for SoapySDR

Compilation
-----------

Activate your conda enivronment:

.. code-block:: bash

   conda activate -n <environment name>

.. hint::
   Check out radioconda and conda environment set up process. See :ref:`radioconda-setup-ref`.
.. Add reference to radioconda and conda setup

Install conda packages:

.. code-block:: bash

   conda install conda-build conda-forge-pinning vs2022_win-64 cmake=3.26.4 ninja

Restart radioconda prompt and activate your conda environment. This will setup appropriate build environment variables. Clone repository:

.. code-block:: bash

   git clone <repository url>

Enter repository directory, create and enter build directory:

.. code-block:: bash
   
   mkdir build && cd build

Configure LimeSuiteNG library build files:

.. code-block:: bash

   cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="%CONDA_PREFIX%\Library" -DCMAKE_PREFIX_PATH="%CONDA_PREFIX%\Library" -DGR_PYTHON_DIR="%CONDA_PREFIX%\Lib\site-packages" -DINSTALL_DEVELOPMENT=ON ..

.. warning::
   It is possible that during cmake configuration python bindings for gnuradio-limesuiteng plugin sink and source blocks will be out of sync. This issue is common when building plugin for different versions of GNURadio on different operating systems. To fix the bindings, enter gnuradio-limesuiteng root directory ``cd <repo root>\plugins\gr-limesuiteng`` and manually rebind the files using the following commands ``gr_modtool bind -u sdrdevice_source`` and ``gr_modtool bind -u sdrdevice_sink``. Conda environment must be active to use the gnuradio gr_modtool.

Build suite:

.. code-block:: bash

   cmake --build .

Built suite files are located in ``build\lib`` directory and executables are located in ``build\bin`` directory.
  

Installing the built software
-----------------------------

Optionally library can be installed into system, installation requires to be ran with Administrative privileges.

Continuing on from the previous command block, execute:

.. code-block:: bash

   cmake --install .

.. tip::
   To uninstall library from conda environment use: ``cmake -P cmake_uninstall.cmake`` command inside the build directory.

.. _`CMake`: https://cmake.org/
.. _`wxWidgets`: https://www.wxwidgets.org/
.. _`SoapySDR`: https://github.com/pothosware/SoapySDR
.. _`FX3SDK`: https://softwaretools.infineon.com/tools/com.ifx.tb.tool.ezusbfx3sdk
