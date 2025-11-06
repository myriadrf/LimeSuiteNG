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

Update apt sources and install limesuiteng:

.. code-block:: bash

	sudo apt-get update
	sudo apt-get install limesuiteng

Conda
-----

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

This ensures, that metapackage install pulls the correct  ``gnuradio-limesuiteng`` plugin sub-package version that is compatible with the current gnuradio installation.

.. note::

   Metapackage and gnuradio package version specification is optional. You can specify version ``limesuiteng=25.1.0`` or version and build string ``limesuiteng=25.1.0=hba12d79_0`` to install exact metapackage or gnuradio package version and it's components. If version is not specified, conda will install the latest most compatible package versions.

.. note::

   You can check out limesuiteng versions by executing the following command - ``conda search limesuiteng``. To search for gnuradio versions use - ``conda search gnuradio``.

If you only need certain LimeSuiteNG components, for example, development files, you can install a metapackage sub-package as a standalone package:

.. code-block:: bash

   conda activate <environment>
   conda install liblimesuiteng-dev

On Linux LimeSuiteNG conda metapackage links the following sub-packages together:

#. liblimesuiteng - runtime libraries.
#. liblimesuiteng-dev - runtime libraries, API and development files.
#. limesuiteng-cli - command line tools for LimeSDR device control.
#. gnuradio-limesuiteng - LimeSDR plugin for GNURadio. To install plugin as a separate package check out :ref:`gnuradio-plugin-install-ref` section.
#. limesuiteng-soapy - LimeSuiteNG bindings for SoapySDR.

All of the above listed packages can be installed and used independently.

.. note::

	Currently none of the above listed LimeSuiteNG conda sub-packages contain limeGUI application for conda environment on Linux. To use limeGUI install LimeSuiteNG from apt store or build LimeSuiteNG from source.