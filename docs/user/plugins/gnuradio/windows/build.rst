.. _windows-plugin-build-ref:

Build from source
#################

Prerequisites
*************

Required components to compile gnuradio-limesuiteng plugin:

#. Visual Studio Build Tools 2022 (version 17.14) components:

   #. MSVC v143 - VS 2022
   #. Windows 11 SDK

#. Conda packages:

   #. conda-build
   #. conda-forge-pinning
   #. gnuradio
   #. boost
   #. vs2022_win-64
   #. Lime Suite NG (Manually built and installed)
   #. cmake
   #. ninja

.. warning::
   External Python and numpy installations that are not related to any conda environment can interfere with build process. It is advised to remove these components or temporarily disable environment variables that point to their file system location.

.. warning::
   It is advised to use a manually built and installed Lime Suite NG library in conda environment to avoid build process problems.

.. note::
   MSVC version must be equal or higher than compiler version that was used to build gnuradio. To check compiler compatibility use ``gnuradio-config-print --cxx`` inside activated conda environment with installed gnuradio.

Compilation
***********

Open radioconda with administrative privileges and activate your conda environment:

.. code-block:: bash

   conda activate -n <environment name>

.. hint::
   For radioconda and conda environment setup, see :ref:`conda-windows-ref`.

Install all necessary build components by executing the following script in repository root directory:

.. code-block:: bash

   conda_deps.bat --v <required-gnuradio-version>

The script will check the conda environment for missing packages and install all conda packages required to build the gnuradio-limesuiteng plugin alongside the requested GNU Radio version. Example:

.. code-block:: bash

   conda_deps.bat --v 3.10.11.0

A list of verified plugin builds against different GNU Radio versions is provided in the ``conda_requirements.txt`` file. After successful component installation, restart the radioconda prompt and activate your conda environment. This will set up the appropriate build environment variables. Clone the repository:

.. code-block:: bash

   git clone <repository url>

Enter plugin directory:

.. code-block:: bash

   cd <repo root>\plugin\gr-limesuiteng

Create and enter build directory:

.. code-block:: bash

   mkdir build && cd build

Run cmake to configure build files:

.. code-block:: bash

   cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="%CONDA_PREFIX%\Library" -DCMAKE_PREFIX_PATH="%CONDA_PREFIX%\Library" -DGR_PYTHON_DIR="%CONDA_PREFIX%\Lib\site-packages" ..

Build plugin:

.. code-block:: bash

   cmake --build .

Installation
************

Install plugin into conda environment:

.. code-block:: bash

   cmake --install .

.. tip::
   To uninstall the plugin from the conda environment, use ``cmake -P cmake_uninstall.cmake`` inside the build directory.

See :ref:`gnuradio-limesuiteng-example-ref` for a plugin demonstration.