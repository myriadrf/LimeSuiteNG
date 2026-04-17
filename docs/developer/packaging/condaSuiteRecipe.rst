Minimal Lime Suite NG recipe
============================

.. important::

   Compared to standard build procedure, building library package using conda-build tool takes more time (5-8 min.) and resources. For development purposes it is advised to build from source using CMake. See :ref:`windows-lib-build-ref`.

.. note::

   Recipe build was tested in radioconda environment for Windows and Linux platforms.

Recipe structure
----------------

A full conda recipe is made out of three main files:

#. meta.yaml
#. bld.bat (For Windows platform)
#. bld.sh (For Linux/MacOS platforms)

Meta.yaml file is used to specify required dependencies for build. Since most of dependencies in Lime Suite NG are resolved internally (through CMake scripts), you will only need to modify any required build tools, such as compilers and build generators. Scripts bld.bat and bld.sh are used for the actual build. Inside the scripts you can add additional CMake flags and any other commands that will be executed in command line/terminal environment.

More details about `conda recipes`_.

Recipe build steps
------------------

To start the build process, execute the following commands:

.. code-block:: bash

   conda activate <custom env name>
   cd <repo root>
   conda-build .conda\recipe\ -m .conda\build_config.yaml

Conda will start building package. The ``build_config.yaml`` file alongside the ``-m`` flag enables package build from locally stored source code. To build from git ``develop`` branch, omit the flag and arguments. After successfull build, conda package is populated in ``<radioconda install root>\envs\<your custom env>\conda-bld\`` directory with the following name structure ``limesuiteng-version-build_string.conda``. Radioconda install root can be found using the following conda command:

.. code-block:: bash

   conda config --show root_prefix

Installing built package
------------------------

To access and test Lime Suite NG binaries or library for other builds, package must be installed using the conda install command. Before installing the package, path to local channel for conda tool must be specified (Check out :ref:`conda-local-channel-setup` section for instructions on how to set up local channel). To install locally built Lime Suite NG package, execute the following command:

.. code-block:: bash

   conda install limesuiteng

Optionally, you can specify version ``limesuiteng=25.1.0`` or version and build string ``limesuiteng=25.1.0=hb7fb3a4_0`` if there are multiple versions of the same package.

.. _conda recipes: https://docs.conda.io/projects/conda-build/en/stable/resources/index.html