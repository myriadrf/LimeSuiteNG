.. _conda-windows-ref:

Conda Packages
##############

.. warning:: 
   
   Currently, Lime Suite NG and its software components are not available as standalone conda packages.
   Instead, build the newest Lime Suite NG software from source.

Radioconda
**********

Initial setup
=============

The radioconda installation files can be found on the radioconda GitHub `release page`_.

For Windows, install the radioconda terminal using the ``radioconda-<release-date>-Windows-x86_64.exe`` installer. To access the radioconda environment on Windows, search for **radioconda terminal** and open it with administrator privileges.

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

Lime Suite NG provides multiple packages for installation. To install all Lime Suite NG components, install the Lime Suite NG metapackage. If the gnuradio package is already present in your current environment, execute the following commands:

.. code-block:: bash

   conda activate <environment>
   conda install limesuiteng=[version]

If the gnuradio package is missing from your current environment, execute the following commands to install it alongside the metapackage:

.. code-block:: bash

   conda activate <environment>
   conda install gnuradio=[version] limesuiteng=[version]

This ensures that the metapackage installation pulls the correct ``gnuradio-limesuiteng`` plugin sub-package version.

.. note::

   Metapackage and gnuradio package version specification is optional. You can specify version ``limesuiteng=25.1.0`` or version and build string ``limesuiteng=25.1.0=hba12d79_0`` to install an exact metapackage or gnuradio package version and its components. If version is not specified, conda will install the latest most compatible package versions.

.. note::

   To view available limesuiteng versions, execute ``conda search limesuiteng``. To search for gnuradio versions, use ``conda search gnuradio``.

If you only need certain Lime Suite NG components, for example, development files, you can install a metapackage sub-package as a standalone package:

.. code-block:: bash

   conda activate <environment>
   conda install liblimesuiteng-dev

On Windows, the Lime Suite NG conda metapackage links the following sub-packages together:

#. liblimesuiteng - runtime libraries.
#. liblimesuiteng-dev  - runtime libraries, API and development files.
#. limesuiteng-cli - command line tools for LimeSDR device control.
#. limesuiteng-gui - LimeSDR device configuration tool with GUI support.
#. gnuradio-limesuiteng - LimeSDR plugin for GNU Radio. To install the plugin as a separate package, see :ref:`windows-gnuradio-plugin-install-ref`.
#. limesuiteng-soapy - Lime Suite NG bindings for SoapySDR.

All of the above listed packages can be installed and used independently.

.. _`release page`: https://github.com/radioconda/radioconda-installer/releases