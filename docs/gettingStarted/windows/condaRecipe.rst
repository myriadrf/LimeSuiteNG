Build using conda recipe
========================

.. important::

   Compared to standard build procedure, building library package using conda-build tool takes more time (5-8 min.) and resources. For development purposes it is advised to build from source using cmake. See :ref:`windows-lib-build-ref`.

.. hint::
   
   Check out radioconda and conda environment setup. See :ref:`radioconda-setup-ref`.

LimeSuiteNG library can also be built using conda recipe. Plugin recipe files are provided in ``<project>\.conda\`` directory.

To start the build process execute the following commands:

.. code-block:: bash

   conda activate <custom env name>
   cd <project root dir>
   conda-build .conda\local_recipe\

Conda will start building package. After successfull build conda package is populated in ``<radioconda install dir>\envs\<your custom env>\conda-bld\win-64`` directory with the following name structure ``package_name-version-build_number.conda``. To use built LimeSuiteNG library components, package must be installed using the following command:

.. code-block:: bash

   conda install <absolute_path>\package.conda

.. hint::

   After successful install of LimeSuiteNG library package, various command line utilities and development components are available for use. See :ref:`cli-tools-ref`.  
