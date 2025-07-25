.. _windows-plugin-build-ref:

=================
Build from source
=================

Prerequisites
-------------

Required components to compile gnuradio-limesuiteng plugin:

#. Visual Studio Build Tools 2022 (version 17.14) components:

   #. MSVC v143 - VS 2022
   #. Windows 11 SDK

#. Conda packages:

   #. conda-build
   #. conda-forge-pinning
   #. gnuradio=3.10.9.2
   #. gnuradio-build-deps=3.10.9.2
   #. boost=1.82
   #. vs2022_win-64
   #. LimeSuiteNG (Manually built)
   #. cmake (supported versions by project: 3.15 - 3.31)
   #. ninja

.. warning::
   External Python and numpy installations that are not related to any conda environment can interfere with build process. It is advised to remove these components or temporarily disable environment variables that point to their locations in PC file system.

.. warning::
   It is advised to use a manually built and installed LimeSuiteNG library in conda environment to avoid build process problems.

.. note::
   MSVC version must be equal or higher than compiler version that was used to build gnuradio. To check compiler compatibility use ``gnuradio-config-print --cxx`` inside activated conda environment with installed gnuradio.

.. note::
   Visual Studio Build Tools 2019 can also be used to build the plugin, but the MSVC compiler version must be higher or equal to version 19.29.


Compilation
-----------

Activate your conda enivronment:

.. code-block:: bash

   conda activate -n <environment name>

.. hint::
   Check out radioconda and conda environment set up process. See :ref:`radioconda-setup-ref`.

Install conda packages:

.. code-block:: bash

   conda install conda-build conda-forge-pinning gnuradio=3.10.9.2 gnuradio-build-deps=3.10.9.2 boost=1.82 vs2022_win-64 cmake=3.26.4 python=3.12.9 pybind11=2.11.1

Restart radioconda prompt and activate your conda environment. This will setup appropriate build environment variables. Clone repository:

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

Install plugin into conda environment:

.. code-block:: bash

   cmake --install .

.. tip::
   To uninstall plugin from conda environment use: ``cmake -P cmake_uninstall.cmake`` command inside the build directory.

Check out :ref:`gnuradio-limesuiteng-example-ref` section for plugin demonstration.