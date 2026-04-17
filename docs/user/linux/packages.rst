Install from packages
=====================

apt
---

The apt repository contains packages for both x86-64 and arm64.

Install the GPG key:

.. code-block:: bash

	wget -qO - https://repo.myriadrf.org/lime-microsystems-public.gpg | gpg --dearmor | sudo tee /etc/apt/keyrings/lime-microsystems-public.gpg > /dev/null

Add source for your distribution:

.. code-block:: bash

	echo "deb [signed-by=/etc/apt/keyrings/lime-microsystems-public.gpg] https://repo.myriadrf.org/apt stable main" | sudo tee /etc/apt/sources.list.d/repo.myriadrf.org.list

Update apt sources and install Lime Suite NG:

.. code-block:: bash

	sudo apt-get update
	sudo apt-get install limesuiteng

Conda
-----

.. hint::
   Check out radioconda and conda environment setup process. See :ref:`radioconda-setup-ref`.

Lime Suite NG library can also be installed as a conda package. Lime Suite NG provides multiple packages for installation. You can install all possible Lime Suite NG components by installing the Lime Suite NG metapackage. If you want to install entire Lime Suite NG metapackage and already have a gnuradio package in your current environment, execute the following commands:

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

	Currently none of the above listed Lime Suite NG conda sub-packages contain limeGUI application for conda environment on Linux. To use limeGUI install Lime Suite NG from apt store or build Lime Suite NG from source.