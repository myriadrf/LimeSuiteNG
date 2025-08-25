Build using conda recipe
========================

Notice
------

This section of documentation is reserved for project developers, package maintainers and other contributors. Users are recommended to build Lime Suite NG from source (See :ref:`windows-lib-build-ref`) or install as a conda package from conda package manager (See :ref:`limesuiteng-install-ref`).

.. important::

   Compared to standard build procedure, building library package using conda-build tool takes more time (5-8 min.) and resources. For development purposes it is advised to build from source using CMake. See :ref:`windows-lib-build-ref`.

.. hint::
   
   Check out radioconda and conda environment setup. See :ref:`radioconda-setup-ref`.

Recipe build steps
------------------

LimeSuiteNG library can also be built using conda recipe. Library recipe files are provided in ``<repo root>\.conda\`` directory. To start the build process, execute the following commands:

.. code-block:: bash

   conda activate <custom env name>
   cd <repo root>
   conda-build .conda\recipe\ -m .conda\build_config.yaml

Conda will start building package. The ``build_config.yaml`` file alongside the ``-m`` flag enables package build from locally stored source code. To build from git ``develop`` branch omit the flag and arguments. After successfull build, conda package is populated in ``<radioconda install dir>\envs\<your custom env>\conda-bld\win-64`` directory with the following name structure ``limesuiteng-version-build_number.conda``.

.. note::

   To store built package in another directory, add ``--output-folder=[path]`` flag to conda-build tool. Directory must exist prior to using conda-build.

To use built LimeSuiteNG library components, package must be installed using the conda install command as shown below. Specified path to package must be absolute.

.. code-block:: bash

   conda install <absolute_path>\limesuiteng-version-build_number.conda

.. hint::

   After successful install of LimeSuiteNG library package, various command line utilities and development components are available for use. See :ref:`cli-tools-ref`.  
