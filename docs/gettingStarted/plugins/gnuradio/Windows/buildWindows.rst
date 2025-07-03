Build from source
=================

Prerequisites
-------------

Required components to compile gnuradio-limesuiteng plugin:

#. Visual Studio Build Tools 2022 17.14 components:

   #. MSVC v143 - VS 2022
   #. Windows 11 SDK

#. conda-build (conda package)
#. conda-forge-pinnings (conda package)
#. gnuradio=3.10.9.2 (conda package)
#. gnuradio-build-deps=3.10.9.2 (conda package)
#. boost=1.82 (conda package)
#. vs2022_win-64 (conda package)


.. note::
   MSVC version must be equal or higher than compiler version that was used to build gnuradio. To check compiler compatibility use gnuradio-config-print --cxx inside activated conda environment.


Compilation
-----------

Activate your conda enivronment: