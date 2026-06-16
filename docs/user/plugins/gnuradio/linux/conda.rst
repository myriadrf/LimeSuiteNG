.. _linux-gnuradio-plugin-install-ref:

Radioconda package
##################

.. warning:: 
   
   Currently, Lime Suite NG and its software components are not available as standalone conda packages.
   Instead, build the newest Lime Suite NG software from source.

.. hint::
   For radioconda and conda environment setup, see :ref:`conda-linux-ref`.

The gnuradio-limesuiteng plugin for GNU Radio can also be downloaded as a conda package.


Prerequisites
*************

Conda packages required to run the gnuradio plugin:

#. Python
#. Numpy
#. gnuradio

.. tip::
   You can check if packages are present by using ``conda list <package name>``.
   
Installing
**********

If the gnuradio package is already installed in the current conda environment, execute the following commands to install the ``gnuradio-limesuiteng`` plugin package:

.. code-block:: bash

   conda activate <environment>
   conda install gnuradio-limesuiteng=[version]

If the gnuradio package is missing from the current conda environment, execute the following commands to install the ``gnuradio-limesuiteng`` plugin package:

.. code-block:: bash

   conda install gnuradio=[version] gnuradio-limesuiteng

This will ensure that the correct GNURadio plugin version is installed alongside the GNURadio package.

.. note::

   Plugin and gnuradio package version specification is optional. You can specify version ``gnuradio-limesuiteng=1.0.0`` or version and build string ``gnuradio-limesuiteng=1.0.0=py312hfdb1c45_0`` to install exact plugin and gnuradio package version. If version is not specified, conda will install the latest most compatible package versions.

.. note::

   To view available plugin versions, execute ``conda search gnuradio-limesuiteng``. To search for gnuradio versions, use ``conda search gnuradio``.


See :ref:`gnuradio-limesuiteng-example-ref` and :ref:`gnuradio-blocks-ref` for a demonstration and plugin block documentation.

Known issues
************

Conda package compatibility
===========================

Conda Forge contains multiple ``gnuradio`` packages of the same version but with different build strings. Some of these packages may not be compatible with available ``gnuradio-limesuiteng`` packages.

It has been observed that particularly the ``spdlog`` conda package can cause problems when running or installing the ``gnuradio`` package alongside the ``gnuradio-limesuiteng`` package.

Solution
--------

When installing ``gnuradio`` and ``gnuradio-limesuiteng`` packages, also install the appropriate ``spdlog`` package version: ``conda install gnuradio=... limesuiteng=... spdlog=...``. Newer ``gnuradio`` packages (3.10.11.0 and 3.10.10.0) use ``spdlog`` 1.14.1 and 1.13.0, while the older ``gnuradio`` versions (3.10.9.2 and lower) use ``spdlog`` 1.12.0. 