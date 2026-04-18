.. _conda-linux-ref:

Conda Packages
##############

Installation with Conda packages makes use of the radioconda environment and this needs to be set up first.

Radioconda
**********

Initial setup
=============

The radioconda installation files can be found in radioconda github `release page`_.

Download ``radioconda-<release-date>-Linux-x86_64.sh``, open a terminal and install the radioconda environment:

.. code-block:: bash

   cd Downloads
   bash radioconda-<release-date>-Linux-x86_64.sh

The radioconda base environment should load automatically once the terminal is opened.

.. tip::

   If you want to exit radioconda base environment, enter the ``conda deactivate`` command.

Create a new radioconda environment with a custom name and activate it using the following commands:

.. code-block:: bash
   
   conda create -n <custom environment name>
   conda activate <custom environment name>

Install the following packages that contain necessary build tools for the current environment.

.. code-block:: bash

   conda install conda-build conda-forge-pinning

The custom radioconda environment setup is complete.

.. tip::

   Make sure that your LimeSDR device is compatible with the new generation library and install appropriate SDR device drivers. See :ref:`dev-supp-list-ref` and :ref:`driver-supp-list-ref`.

Lime Suite NG
*************

Lime Suite NG provides multiple packages for installation. You can install all possible Lime Suite NG components by installing the Lime Suite NG metapackage. If you want to install entire Lime Suite NG metapackage and already have a gnuradio package in your current environment, execute the following commands:

.. code-block:: bash

   conda activate <environment>
   conda install limesuiteng=[version]

If you want to install entire Lime Suite NG metapackage, but you are missing gnuradio package in your current environment, execute the following commands:

.. code-block:: bash

   conda activate <environment>
   conda install gnuradio=[version] limesuiteng=[version]

This ensures, that metapackage install pulls the correct  ``gnuradio-limesuiteng`` plugin sub-package version that is compatible with the current gnuradio installation.

.. note::

   Metapackage and gnuradio package version specification is optional. You can specify version ``limesuiteng=25.1.0`` or version and build string ``limesuiteng=25.1.0=hba12d79_0`` to install exact metapackage or gnuradio package version and it's components. If version is not specified, conda will install the latest most compatible package versions.

.. note::

   You can check out limesuiteng versions by executing the following command - ``conda search limesuiteng``. To search for gnuradio versions use - ``conda search gnuradio``.

If you only need certain Lime Suite NG components, for example, development files, you can install a metapackage sub-package as a standalone package:

.. code-block:: bash

   conda activate <environment>
   conda install liblimesuiteng-dev

On Linux Lime Suite NG conda metapackage links the following sub-packages together:

#. liblimesuiteng - runtime libraries.
#. liblimesuiteng-dev - runtime libraries, API and development files.
#. limesuiteng-cli - command line tools for LimeSDR device control.
#. gnuradio-limesuiteng - LimeSDR plugin for GNURadio. To install plugin as a separate package check out :ref:`linux-gnuradio-plugin-install-ref` section.
#. limesuiteng-soapy - Lime Suite NG bindings for SoapySDR.

All of the above listed packages can be installed and used independently.

.. note::

	Currently none of the above listed Lime Suite NG conda sub-packages contain limeGUI application for conda environment on Linux. To use limeGUI install Lime Suite NG via the APT repository or build Lime Suite NG from source.


.. _`release page`: https://github.com/radioconda/radioconda-installer/releases