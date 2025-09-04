.. _gnuradio-plugin-install-ref:

Conda package
=============

.. hint::
   Check out radioconda and conda environment set up process. See :ref:`radioconda-setup-ref`.

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

.. code-block:: bash

   conda activate <environment>
   conda install gnuradio-limesuiteng

Optionally, you can specify metapackage version ``gnuradio-limesuiteng=1.0.0`` or version and build string ``gnuradio-limesuiteng=1.0.0=py312hf2d996d_0`` to install exact plugin package version.

.. note::

   You can check out gnuradio-limesuiteng versions by executing the following command - ``conda search gnuradio-limesuiteng``.

Check out :ref:`gnuradio-limesuiteng-example-ref` and :ref:`gnuradio-blocks-ref` sections for demonstration and plugin block documentation.