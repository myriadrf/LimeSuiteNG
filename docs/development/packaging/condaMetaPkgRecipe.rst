Metapackage recipe
==================

.. note::

   Recipe build was tested on in radioconda environment for Windows and Linux.

Recipe structure
----------------

Metapackage recipe is made out of 4 files:

#. meta.yaml
#. bld.bat (For Windows platform)
#. bld.sh (For Linux/MacOS platform)
#. conda_build_config.yaml

``Meta.yaml`` file is used to specify required dependencies for package build. Scripts ``bld.bat`` and ``bld.sh`` are used for the actual build. Inside the scripts you can add additional CMake flags and any other commands that will be executed in command line/terminal environment. All metapackage tools and dependencies required to build the LimeSuiteNG project and it's components must be specified in ``meta.yaml`` top ``requirements:`` section using ``build:``, ``host:`` and ``run:`` sub-sections. ``output:`` section is used to organize built components into separate sub-packages with their own independent requirements. Example of ``liblimesuiteng-dev`` sub-package description:

.. code-block:: yaml

   - name: liblimesuiteng-dev                   # Development version of the release package
     version: {{ lib_ver }}
     files:                                     # Specify which files will be moved to sub-package from the main build.
      - Library/include/limesuiteng             # Adding only development files such as headers and cmake files.
      - Library/lib/cmake/limesuiteng           # Re-adding static and dynamic libraries could cause errors.
      - Library/lib/pkgconfig/limesuiteng.pc    # Instead pin the sub-package that contains those libraries.
     requirements:
      build:
       - {{ compiler("c") }}   #[not win]
       - {{ compiler("cxx") }} #[not win]
       - vs2022_win-64         #[win]
       ...
 
      run:
       - {{ pin_subpackage('liblimesuiteng', exact=True) }}    # Pin a sub-package(s) that contain(s) the missing files.
         
     test:        # Test if the files are present in the sub-package
      commands:
       - if not exist %PREFIX%\\Library\\include\\limesuiteng\\complex.h exit 1
       - if not exist %PREFIX%\\Library\\include\\limesuiteng\\config.h exit 1
       ...

     about:    # Update sub-package information for end user.
      home: https://github.com/myriadrf/LimeSuiteNG
      license: Apache-2.0
      summary: Conda package that contains development files of Lime Suite NG core library for interaction with LimeSDR based devices.
      ...

A new sub-package description starts with ``- name: sub_package_name`` sub-section in the metapackage  ``output:`` section. Built files are organized into sub-packages with the help of ``files:`` sub-section. Each sub-package contains a list of files that will be moved to the sub-package after LimeSuiteNG project and it's components are built. Conda will move all files from current build environment into respective sub-packages, if the necessary files were installed during build procedure (bld.bat/bld.sh script). Therefore, it is necessary to make sure that any new LimeSuiteNG component files are included in project cmake install rules, otherwise the files of new project component will be skipped. Sub-packages should only contain files that are unique to them. For example, a release library sub-package contains libraries and API headers, but a development version of the same release library sub-package can contain extra header files and etc. Therefore, the development version must not explicitly list the release libraries and API files in the ``files:`` sub-section, but instead pin the release library sub-package in the sub-package requirements ``run:`` sub-section as shown above in the example. Pinning the release library sub-package will indicate that this is a must have runtime package, that will be installed alongside development files when prompted by the user. This pinning behaviour is also prefered in order to avoid dublication of files at package install time and to avoid any file inclusion errors at metapackage build time. Sub-package ``requirements:`` sections is not mandatory to fill out, but it is a good practice to specify sub-package requirements using ``build:``, ``host:`` and ``run:`` sub-sections, since sub-packages do not inherit the metapackage build, link and runtime requirements. Each sub-package contents must be tested individually in the ``test:`` section using ``commands:`` sub-section. ``commands:`` sub-section accepts standard command line/terminal commands. Test scripts can also be run. In the ``about:`` section of a sub-package, only the ``summary:`` sub-section must have a unique sub-package description. Other sub-sections, such as ``license:`` and etc., can be copied from other sub-packages ``about:`` sections.
Sub-packages must be pinned to metapackage in the metapackage ``requirements:`` section, ``run:`` sub-section using ``- {{ pin_subpackage('sub_package_name', exact=True) }}`` command. Excerpt from LimeSuiteNG metapackage ``meta.yaml`` file:

.. code-block:: yaml
   
   ...
   run:
  # This metapackage depends on the following LimeSuiteNG sub-packages
    - {{ pin_subpackage('liblimesuiteng', exact=True) }}
    - {{ pin_subpackage('liblimesuiteng-dev', exact=True) }}
    - {{ pin_subpackage('limesuiteng-cli', exact=True) }}
    - {{ pin_subpackage('limesuiteng-gui', exact=True) }}
    - {{ pin_subpackage('gnuradio-limesuiteng', exact=True) }}

   ...

Any new LimeSuiteNG sub-packages must also be pinned to the metapackage requirements ``run:`` sub-section.

Since ``gnuradio-limesuiteng`` sub-package must be built against different versions of GNURadio and it's respective dependencies, a single ``limesuiteng`` metapackage build run will yield multiple metapackages with different build strings. These different metapackage versions will pin to exact ``gnuradio-limesuiteng`` sub-package versions. Other sub-package pins will stay identical in different metapackage versions, since they are not built against dependencies with different versions. ``gnuradio-limesuiteng`` sub-package build against different GNURadio versions is controlled using ``conda_build_config.yaml`` file. This file is appended to build by default, since it is in the same directory as other recipe files. File provides a list of different version dependencies that should be used to build ``gnuradio-limesuiteng`` sub-package. Excerpt from ``conda_build_config.yaml``:

.. code-block:: yaml

   python:
  # gnuradio 3.10.11.0
  - 3.12.*                 # Dependency group 1
  - 3.11.*                 # Dependency group 2
  ...
  # gnuradio 3.10.10.0 
  - 3.12.*                 # Dependency group 3
  - 3.11.*                 # Dependency group 4
  ...
  numpy:
  # gnuradio 3.10.11.0
  - 2.0.2               # Dependency group 1
  - 2.0.2               # Dependency group 2
  ...
  # gnuradio 3.10.10.0
  - 2.0.0               # Dependency group 3
  - 2.0.0               # Dependency group 4
  ...
  gnuradio_core:
  - 3.10.11.0     # Dependency group 1
  - 3.10.11.0     # Dependency group 2
  ...
  - 3.10.10.0     # Dependency group 3
  - 3.10.10.0     # Dependency group 4
  ...
  zip_keys:
  - python
  - numpy
  - boost
  - pybind11
  - volk
  - gnuradio_core

Dependency versions for ``gnuradio-limesuiteng`` sub-package build are grouped using ``zip_keys:`` statement which is located at the end of the ``conda_build_config.yaml`` file. ``zip_keys:`` statement creates multiple different version dependency groups that will then be used to build multiple ``gnuradio-limesuiteng`` sub-packages. Grouping is performed starting from the top of a each dependency version list by taking a single dependency version entry from each dependency version list and grouping them into a single variant. From the provided ``conda_build_config.yaml`` excerpt, first ``gnuradio-limesuiteng`` sub-package build variant will have the following dependency variant ``{python=3.12.*, numpy=2.0.2, ..., gnuradio_core=3.10.11.0}``, second dependency variant ``{python=3.11.*, numpy=2.0.2, ..., gnuradio_core=3.10.11.0}`` and etc. This approach allows to build for multiple ``gnuradio-limesuiteng`` sub-package versions, that are linked to different dependency versions, and pin them to respective metapackages in a single build run. These dependency lists can be updated, but be aware that all dependency lists must contain identical amount of version entries. Otherwise the build will fail.

More about `conda recipes`_ and `conda build variants`_.

Recipe build steps
------------------

To start the build process, execute the following commands:

.. code-block:: bash

   conda activate <custom env name>
   cd <repo root>
   conda-build .conda\metapkg\ -m .conda\build_config.yaml

Conda will start building metapackages and sub-packages. The ``build_config.yaml`` file alongside the ``-m`` flag enables package build from locally stored source code. To build from git ``develop`` branch, omit the flag and arguments. After successfull build, conda packages are populated in ``<radioconda install root>\envs\<your custom env>\conda-bld\win-64`` directory. Radioconda install root can be found using the following conda command:

.. code-block:: bash

   conda config --show root_prefix

.. note::

   Metapackage build typically lasts several hours.

Installing built packages
-------------------------

To access and test LimeSuiteNG metapackage components, metapackage must be installed using the conda install command. Before installing the metapackage, path to local channel for conda tool must be specified (Check out :ref:`conda-local-channel-setup` section for instructions on how set up local channel). To install locally built LimeSuiteNG metapackage, execute the following command:

.. code-block:: bash

   conda install limesuiteng

Optionally, you can specify version ``limesuiteng=25.1.0`` or version and build string ``limesuiteng=25.1.0=hb7fb3a4_0`` if there are multiple versions of the same metapackage.

.. _conda recipes: https://docs.conda.io/projects/conda-build/en/stable/resources/index.html
.. _conda build variants: https://docs.conda.io/projects/conda-build/en/stable/resources/variants.html