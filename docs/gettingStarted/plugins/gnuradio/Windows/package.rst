.. _windows-gnuradio-plugin-install-ref:

Conda package
=============

.. hint::
   Check out radioconda and conda environment setup process. See :ref:`radioconda-setup-ref`.

Gnuradio-limesuiteng plugin for GNURadio can also be downloaded as a conda package.


Prerequisites
-------------

Conda packages required to run the gnuradio plugin:

#. Python
#. Numpy
#. gnuradio

.. tip::
   You can check if packages are present by using ``conda list <package name>``.
   
Installing
----------

If gnuradio package in the current conda environment is already installed, execute the following commands in conda environment to install ``gnuradio-limesuiteng`` plugin package:

.. code-block:: bash

   conda activate <environment>
   conda install gnuradio-limesuiteng=[version]

If gnuradio package is missing in the current conda environment, execute the following commands in conda environment to install ``gnuradio-limesuiteng`` plugin package:

.. code-block:: bash

   conda install gnuradio=[version] gnuradio-limesuiteng

This will ensure that the correct GNURadio plugin version is installed alongside GNURadio package.

.. note::

   Plugin and gnuradio package version specification is optional. You can specify version ``gnuradio-limesuiteng=1.0.0`` or version and build string ``gnuradio-limesuiteng=1.0.0=py312hfdb1c45_0`` to install exact plugin and gnuradio package version. If version is not specified, conda will install the latest most compatible package versions.

.. note::

   You can check out plugin versions by executing the following command - ``conda search gnuradio-limesuiteng``. To search for gnuradio versions use - ``conda search gnuradio``.


Check out :ref:`gnuradio-limesuiteng-example-ref` and :ref:`gnuradio-blocks-ref` sections for demonstration and plugin block documentation.