Build using conda recipe
========================

.. important::

   Compared to standard build procedure, building plugin package using conda-build tool takes more time and resources. For development purposes it is advised to build from source using cmake. See :ref:`windows-plugin-build-ref`.

.. hint::
   
   Check out radioconda and conda environment setup. See :ref:`radioconda-setup-ref`.

Gnuradio-limesuiteng plugin for GNURadio 3.10.9.2 (Python 3.12.9) can also be built using conda recipe. Plugin recipe files are provided in ``<project>\plugins\gr-limesuiteng\.conda\`` directory.

To start the build process execute the following commands:

.. code-block:: bash

   conda activate <custom env name>
   cd <repo dir>\plugins\gr-limesuiteng
   conda-build .conda\recipe\ -m %CONDA_PREFIX%\conda_build_config.yaml

Conda will start building package. After successfull build conda package is populated in ``<radioconda install dir>\envs\<your custom env>\conda-bld\win-64`` directory with the following name structure ``package_name-version-build_number.conda``. To use built gnuradio-limesuiteng plugin, package must be installed using the following command:

.. code-block:: bash

   conda install <absolute_path>\package.conda
