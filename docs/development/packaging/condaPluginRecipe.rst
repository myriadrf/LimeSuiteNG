Gnuradio-limesuiteng recipe
===========================

.. important::

   Compared to standard build procedure, building plugin package using conda-build tool takes more time (5-8 min.) and resources. For development purposes it is advised to build from source using CMake. See :ref:`windows-plugin-build-ref`.

.. note::

   Conda packaging has only been tested on Windows platform.

Recipe structure
----------------

A full conda recipe is made out of three main files:

#. meta.yaml
#. bld.bat (For Windows platform)
#. bld.sh (For Linux/MacOS platforms)

Meta.yaml file is used to specify required dependencies for build. When building gnuradio-limesuiteng plugin from recipe, it is important to keep track of the following dependency versions:

#. python
#. numpy
#. pybind11
#. volk
#. boost

Required exact versions of dependencies can be found out by inspecting the recipe meta.yaml file of gnuradio package. Failure to match the correct dependency versions can result in plugin operation failure. Scripts bld.bat and bld.sh are used for the actual build. Inside the scripts you can add additional CMake flags and any other commands that will be executed in command line/terminal environment.

More details about `conda recipes`_.

Recipe build steps
------------------

To build the plugin, LimeSuiteNG conda package must be built and installed inside the current conda environment. To start the build process, execute the following commands:

.. code-block:: bash

   conda activate <custom env name>
   cd <repo root>\plugins\gr-limesuiteng
   conda-build .conda\recipe\ -m .conda\build_config.yaml

Conda will start building package. The ``build_config.yaml`` file alongside the ``-m`` flag enables package build from locally stored source code. To build from git ``develop`` branch, omit the flag and arguments. After successfull build, conda package is populated in ``<radioconda install dir>\envs\<your custom env>\conda-bld\win-64`` directory with the following name structure ``gnuradio-limesuiteng-version-build_string.conda``.

.. warning::

   Currently gnuradio-limsuiteng plugin package cannot be built for ``gnuradio=3.10.12.0``, because it requires ``boost=1.86`` package which is currently not available in conda environment. In order to pass the plugin build, user must specify gnuradio version in meta.yaml file as stated below in important note.


.. important::

   To target a specific gnuradio version plugin build, edit the meta.yaml file inside ``recipe`` directory. Specify required version for all entries of gnuradio package. Example: ``gnuradio =3.10.10.0``. If gnuradio package version is not specified, plugin will be built for the newest gnuradio version. 
   
Installing built package
------------------------

To test built gnuradio-limesuiteng plugin, package must be installed using the ``conda install`` command. Before installing the package, path to local channel for conda tool must be specified (Check out :ref:`conda-local-channel-setup` section for instructions on how set up local channel). To install locally built gnuradio-limesuiteng package, execute the following command:

.. code-block:: bash

   conda install gnuradio-limesuiteng

Optionally, you can specify version ``gnuradio-limesuiteng=1.0.0`` or version and build string ``limesuiteng=1.0.0=py312hfd7c3e5_0`` if there are multiple versions of the same package.

.. _conda recipes: https://docs.conda.io/projects/conda-build/en/stable/resources/index.html