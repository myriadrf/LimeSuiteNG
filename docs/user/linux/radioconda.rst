.. _radioconda-setup-linux-ref:

======================
Radioconda environment
======================

Initial setup
---------------

Radioconda installation files can be found in radioconda github `release page`_.

Download ``radioconda-<release-date>-Linux-x86_64.sh``, open terminal and install the radioconda environment:

.. code-block:: bash

   cd Downloads
   bash radioconda-<release-date>-Linux-x86_64.sh

Radioconda base environment should load automatically once the terminal is opened.

.. tip::

   If you want to exit radioconda base environment in terminal, enter ``conda deactivate`` command.

Create new radioconda environment with a custom name and activate it using the following commands:

.. code-block:: bash
   
   conda create -n <custom environment name>
   conda activate <custom environment name>

Install the following packages that contain necessary build tools for the current environment.

.. code-block:: bash

   conda install conda-build conda-forge-pinning

Custom radioconda environment setup is complete.

.. tip::

   Make sure that your LimeSDR device is compatible with the new generation library and install appropriate SDR device drivers. See :ref:`dev-supp-list-ref` and :ref:`driver-supp-list-ref`.

.. _`release page`: https://github.com/radioconda/radioconda-installer/releases