Build using conda recipe
========================

.. important::

   Compared to standard build procedure, building plugin package using conda-build tool takes more time (5-8 min.) and resources. For development purposes it is advised to build from source using cmake. See :ref:`windows-plugin-build-ref`.

.. hint::
   
   Check out radioconda and conda environment setup. See :ref:`radioconda-setup-ref`.

Gnuradio-limesuiteng plugin for GNURadio 3.10.9.2 (Python 3.12.9) can also be built using conda recipe. Plugin recipe files are provided in ``<project>\plugins\gr-limesuiteng\.conda\`` directory.

To start the build process execute the following commands:

.. code-block:: bash

   conda activate <custom env name>
   cd <repo root>\plugins\gr-limesuiteng
   conda-build .conda\local_recipe\ -m %CONDA_PREFIX%\conda_build_config.yaml

.. warning::
   It is possible that during cmake configuration python bindings for gnuradio-limesuiteng plugin sink and source blocks will be out of sync. This issue is common when building plugin for different versions of GNURadio on different operating systems. To fix the bindings enter gnuradio-limesuiteng root directory ``cd <repo root>\plugins\gr-limesuiteng`` and manually rebind the files using the following commands ``gr_modtool bind -u sdrdevice_source`` and ``gr_modtool bind -u sdrdevice_sink``. Conda environment must be active to use the gnuradio gr_modtool.

Conda will start building package. After successfull build conda package is populated in ``<radioconda install dir>\envs\<your custom env>\conda-bld\win-64`` directory with the following name structure ``package_name-version-build_number.conda``. To use built gnuradio-limesuiteng plugin, package must be installed using the following command:

.. code-block:: bash

   conda install <absolute_path>\package.conda
