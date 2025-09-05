.. _limesuiteng-install-ref:

Install from packages
=====================

.. hint::
   Check out radioconda and conda environment set up process. See :ref:`radioconda-setup-ref`.

LimeSuiteNG library can also be installed as a conda package. LimeSuiteNG provides multiple packages for installation. You can install all possible LimeSuiteNG components by installing the LimeSuiteNG metapackage. If you want to install entire LimeSuiteNG metapackage and already have a gnuradio package in your current environment, execute the following commands:

.. code-block:: bash

   conda activate <environment>
   conda install limesuiteng=[version]

If you want to install entire LimeSuiteNG metapackage, but you are missing gnuradio package in your current environment, execute the following commands:

.. code-block:: bash

   conda activate <environment>
   conda install gnuradio=[version] limesuiteng=[version]

This ensures, that metapackage install pulls the correct ``gnuradio-limesuiteng`` plugin sub-package version.

.. note::

   Optionally, you can specify metapackage version ``limesuiteng=25.1.0`` or version and build string ``limesuiteng=25.1.0=hba12d79_0`` to install exact metapackage version and it's components.

.. note::

   You can check out limesuiteng versions by executing the following command - ``conda search limesuiteng``.

If you only need certain LimeSuiteNG components, for example, development files, you can install a metapackage sub-package as a standalone package:

.. code-block:: bash

   conda activate <environment>
   conda install liblimesuiteng-dev

LimeSuiteNG metapackage links the following packages together:

#. liblimesuiteng - Release libraries and library API files.
#. liblimesuiteng-dev - Development libraries, files and library API files.
#. limesuiteng-cli - command line tools for LimeSDR devices.
#. limesuiteng-gui - LimeSDR device configuration tool with GUI support.
#. gnuradio-limesuiteng - LimeSDR plugin for GNURadio. To install plugin as a separate package check out :ref:`gnuradio-plugin-install-ref` section.

All of the above listed packages can be installed and used independently.