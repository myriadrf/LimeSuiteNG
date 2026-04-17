.. _radioconda-setup-windows-ref:

======================
Radioconda environment
======================

Initial setup
-------------

Radioconda installation files can be found in radioconda github `release page`_.

For Windows, install radioconda terminal using ``radioconda-<release-date>-Windows-x86_64.exe`` installer. To access radioconda environment on Windows, search for **radioconda terminal** and open it with administrator privileges.

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