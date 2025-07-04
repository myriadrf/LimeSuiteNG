Build from source
=================

Prerequisites
-------------

Required components to compile gnuradio-limesuiteng plugin:

#. gnuradio
#. boost

Compilation
-----------

Clone repository:

.. code-block:: bash

   git clone <repository url>

Enter repository, create and enter build directory:

.. code-block:: bash

   mkdir build
   cd build

Run cmake to configure build files and then build:

.. code-block:: bash

   cmake .. -DCMAKE_BUILD_TYPE=Release
   make

Plugin installation
-------------------

Install plugin using the following command:

.. code-block:: bash

   sudo make install
   sudo ldconfig

.. tip::
   To uninstall plugin from gnuradio folder use ``sudo make uninstall``.


