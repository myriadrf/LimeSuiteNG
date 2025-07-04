Radioconda environment
======================

Install radioconda terminal using windows installer from radioconda `release page`_. Search for **radioconda prompt** and open it. Create a new conda environment using the following command:

.. code-block:: bash
   
   conda create -n <custom environment name>

Install the following packages that contain necessary build tools for the current environment.

.. code-block:: bash

   conda install conda-build conda-forge-pinning

Conda environment is ready for use with Lime Suite NG library and any of its plugins.

.. tip::
   Make sure that your limeSDR is compatible with the new generation library and install appropriate SDR device drivers.


.. _`release page`: https://github.com/ryanvolz/radioconda/releases