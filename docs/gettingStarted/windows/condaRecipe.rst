Build using conda recipe
========================

.. important::

   Compared to standard build procedure, building library package using conda-build tool takes more time (5-8 min.) and resources. For development purposes it is advised to build from source using CMake. See :ref:`windows-lib-build-ref`.

.. hint::
   
   Check out radioconda and conda environment setup. See :ref:`radioconda-setup-ref`.

LimeSuiteNG library can also be built using conda recipe. Plugin recipe files are provided in ``<repo root>\.conda\`` directory.

To start the build process execute the following commands:

.. code-block:: bash

   conda activate <custom env name>
   cd <repo root>
   conda-build .conda\recipe\ -m .conda\build_config.yaml

Conda will start building package. After successfull build, conda package is populated in ``<radioconda install dir>\envs\<your custom env>\conda-bld\win-64`` directory with the following name structure ``limesuiteng-version-build_number.conda``.

.. note::

   To store built package in another directory, add ``--output-folder=[path]`` flag to conda-build tool. Directory must exist prior to using conda-build.

To use built LimeSuiteNG library components, package must be installed using the conda install command as shown below. Specified path to package must be absolute.

.. code-block:: bash

   conda install <absolute_path>\limesuiteng-version-build_number.conda

.. hint::

   After successful install of LimeSuiteNG library package, various command line utilities and development components are available for use. See :ref:`cli-tools-ref`.  
