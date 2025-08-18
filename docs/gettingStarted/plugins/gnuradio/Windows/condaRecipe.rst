Build using conda recipe
========================

.. important::

   Compared to standard build procedure, building plugin package using conda-build tool takes more time (5-8 min.) and resources. For development purposes it is advised to build from source using CMake. See :ref:`windows-plugin-build-ref`.

.. hint::
   
   Check out radioconda and conda environment setup. See :ref:`radioconda-setup-ref`.

Gnuradio-limesuiteng plugin for GNURadio can also be built using conda recipe. To build the plugin LimeSuiteNG conda package must be built and installed localy inside the current conda environment. Plugin recipe files are provided in ``<project>\plugins\gr-limesuiteng\.conda\`` directory.

.. note::

   ``.conda\local_recipe\`` directory contains recipe for building packages locally on computer. ``.conda\recipe\`` directory contains recipe for building packages using CI/CD systems.

To start the build process execute the following commands:

.. code-block:: bash

   conda activate <custom env name>
   cd <repo root>\plugins\gr-limesuiteng
   conda-build .conda\local_recipe\ -m %CONDA_PREFIX%\conda_build_config.yaml

Conda will start building package. After successfull build conda package is populated in ``<radioconda install dir>\envs\<your custom env>\conda-bld\win-64`` directory with the following name structure ``package_name-version-build_number.conda``.

.. warning::

   Currently gnuradio-limsuiteng plugin package cannot be built for ``gnuradio =3.10.12.0``, because it requires ``boost=1.86`` package which is currently not available in conda environment. In order to pass the plugin build user must specify gnuradio version in .yaml file as stated below in important note.


.. important::

   To target a specific gnuradio version plugin build, edit the .yaml file inside ``local_recipe`` or ``recipe`` directory. Specify required version for all entries of gnuradio package. Example: ``gnuradio =3.10.10.0``. If gnuradio package version is not specified, plugin will be built for the newest gnuradio version. 
   
To use built gnuradio-limesuiteng plugin, package must be installed using the following command:

.. code-block:: bash

   conda install <absolute_path>\package_name-version-build_number.conda

Check out :ref:`gnuradio-limesuiteng-example-ref` section for plugin demonstration.