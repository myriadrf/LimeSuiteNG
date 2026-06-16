Build from Source
##################

Prerequisites
*************

Required components to compile the gnuradio-limesuiteng plugin:

#. GNU Radio
#. Boost
#. Pybind11
#. Gmp
#. Python
#. C++ compiler
#. CMake
#. Lime Suite NG

Compilation
***********

Clone repository:

.. code-block:: bash

   git clone <repository url>

Enter repository, create and enter build directory:

.. code-block:: bash

   cd <repo root>/plugins/gr-limesuiteng
   mkdir build
   cd build

Run CMake to configure build files and build:

.. code-block:: bash

   cmake .. -DCMAKE_BUILD_TYPE=Release
   sudo make

.. tip::
   Add the ``-jx`` flag to speed up the build, where ``x`` is the processor core count.
    

Installation
************

Install the plugin using the following command:

.. code-block:: bash

   sudo make install
   sudo ldconfig

.. tip::
   To uninstall the plugin from the gnuradio folder, use ``sudo make uninstall`` inside the build directory.

See :ref:`gnuradio-limesuiteng-example-ref` for a plugin demonstration.
