.. _windows-lib-build-ref:

Build from Source
#################

Prerequisites
*************

Required components to compile the Lime Suite NG project:

#. Visual Studio Build Tools 2022 components:

   #. MSVC v143 - VS 2022
   #. Windows 11 SDK

#. Conda packages:

   #. conda-build
   #. conda-forge-pinning
   #. vs2022_win-64
   #. cmake
   #. ninja

Optional components that add specific functionality:

- `SoapySDR`_ : allows building of Lime Suite NG plugin for SoapySDR

Compilation
***********

Open radioconda with administrative privileges and activate your conda environment:

.. code-block:: bash

   conda activate -n <environment name>

.. hint::
   For radioconda and conda environment setup, see :ref:`conda-windows-ref`.
.. Add reference to radioconda and conda setup

Install all necessary build components by executing the following script in repository root directory:

.. code-block:: bash

   conda_deps.bat [--v] [required-gnuradio-version]

The script will check the conda environment for missing packages and install all conda packages required to build the Lime Suite NG library. Additionally, the GNU Radio version flag can be supplied to install the appropriate version of GNU Radio and additional dependencies required to build the `gnuradio-limesuiteng` plugin. Example:

.. code-block:: bash

   conda_deps.bat --v 3.10.11.0

A list of verified plugin builds against different GNU Radio versions is provided in the ``conda_requirements.txt`` file. After successful component installation, restart the radioconda prompt and activate your conda environment. This will set up the appropriate build environment variables. Clone the repository:

.. code-block:: bash

   git clone <repository url>

Enter repository directory, create and enter build directory:

.. code-block:: bash
   
   mkdir build && cd build

Configure Lime Suite NG library build files:

.. code-block:: bash

   cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="%CONDA_PREFIX%\Library" -DCMAKE_PREFIX_PATH="%CONDA_PREFIX%\Library" -DGR_PYTHON_DIR="%CONDA_PREFIX%\Lib\site-packages" -DINSTALL_DEVELOPMENT=ON ..

Build suite:

.. code-block:: bash

   cmake --build .

The built suite files are located in the ``build\lib`` directory and executables are in the ``build\bin`` directory.



Installing the built software
*****************************

Optionally, the library can be installed into the system. From the previous build directory, execute:

.. code-block:: bash

   cmake --install .

.. tip::
   To uninstall the library from the conda environment, use ``cmake -P cmake_uninstall.cmake`` inside the build directory.

.. _`CMake`: https://cmake.org/
.. _`wxWidgets`: https://wxwidgets.org/
.. _`SoapySDR`: https://github.com/pothosware/SoapySDR
.. _`FX3SDK`: https://softwaretools.infineon.com/tools/com.ifx.tb.tool.ezusbfx3sdk
