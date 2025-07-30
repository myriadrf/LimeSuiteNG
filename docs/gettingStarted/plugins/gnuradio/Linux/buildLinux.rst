Build from source
=================

Prerequisites
-------------

Required components to compile gnuradio-limesuiteng plugin:

#. GNURadio (version 3.10.x.x)
#. Boost    (version 1.83 or higher)
#. Pybind11 (version 2.11.1 or higher)
#. Gmp
#. Python   (version 3.12.3 or higher)
#. C++ compiler (GCC major version 13.xx)
#. CMake
#. LimeSuiteNG

Compilation
-----------

Clone repository:

.. code-block:: bash

   git clone <repository url>

Enter repository, create and enter build directory:

.. code-block:: bash

   cd <repo root>/plugins/gr-limesuiteng
   mkdir build
   cd build

Run cmake to configure build files and then build:

.. code-block:: bash

   cmake .. -DCMAKE_BUILD_TYPE=Release
   sudo make

.. tip::
   Add ``-jx`` flag to speed up building process, where ``x`` is processor core count.
    

Plugin installation
-------------------

Install plugin using the following command:

.. code-block:: bash

   sudo make install
   sudo ldconfig

.. tip::
   To uninstall plugin from gnuradio folder use ``sudo make uninstall`` inside the build directory.

Check out :ref:`gnuradio-limesuiteng-example-ref` section for plugin demonstration.
