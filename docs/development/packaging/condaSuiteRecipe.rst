Build using conda recipe
========================

Notice
------

This section of documentation describes how to run conda-build tool, update recipes and is reserved for project developers, package maintainers and other contributors. Users are recommended to build LimeSuiteNG from source (See :ref:`windows-lib-build-ref`) or install as a conda package from conda package manager (See :ref:`limesuiteng-install-ref`).

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

Conda will start building package. The ``build_config.yaml`` file alongside the ``-m`` flag enables package build from locally stored source code. To build from git ``develop`` branch, omit the flag and arguments. After successfull build, conda package is populated in ``<radioconda install root>\envs\<your custom env>\conda-bld\win-64`` directory with the following name structure ``limesuiteng-version-build_string.conda``. Radioconda install root can be found using the following conda command:

.. code-block:: bash

   conda config --show root_prefix

Installing built package
------------------------

To access LimeSuiteNG binaries or library for other builds, package must be installed using the conda install command. Before installing the package, path to local channel for conda tool must be specified (Check out :ref:`conda-local-channel-setup` section for instructions on how set up local channel). To install locally built LimeSuiteNG package, execute the following command:

.. code-block:: bash

   conda install limesuiteng

Optionally, you can specify version ``limesuiteng=25.1.0`` or version and build string ``limesuiteng=25.1.0=hb7fb3a4_0`` if there are multiple versions of the same package.

.. hint::

   After successful install of LimeSuiteNG library package, various command line utilities and development components are available for use. See :ref:`cli-tools-ref`.  
