Metapackage recipe
==================

.. note::

   Conda metapackage packaging has only been tested on Windows platform.

Recipe structure
----------------

Metapackage recipe is made out of 4 files:

#. meta.yaml
#. bld.bat (For Windows platform)
#. bld.sh (For Linux/MacOS platform)
#. conda_build_config.yaml

Meta.yaml file is used to specify required dependencies for package build. Scripts bld.bat and bld.sh are used for the actual build. Inside the scripts you can add additional CMake flags and any other commands that will be executed in command line/terminal environment. All metapackage tools and dependencies required to build the LimeSuiteNG project and it's components must be specified in meta.yaml top ``requirements:`` section using ``build:``, ``host:`` and ``run:`` directives. ``output:`` section is used to organize built components into separate sub-packages with their own independent requirements. A new sub-package description starts with ``- name: sub_package_name`` directive in the metapackage  ``output:`` section. The built sub-packages are pinned using the ``run:`` directive in the metapackage ``requirements:`` section with ``- {{ pin_subpackage('liblimesuiteng', exact=True) }}`` commands. Any new LimeSuiteNG sub-packages must also be pinned using the metapackage ``run:`` directive. If a sub-package is dependent on another sub-package (for example, liblimesuiteng package libraries), then the required sub-package must be pinned using following command ``- {{ pin_subpackage('liblimesuiteng', exact=True) }}`` in the dependent sub-package ``requirements:`` section ``run:`` directive. Built files are organized into sub-packages with the help of ``files:`` directive. Each sub-package contains a list of files that will be moved to the sub-package after LimeSuiteNG project and it's components are built. Conda will move all files from current build environment into respective sub-packages, if the necessary files were installed during build procedure (bld.bat/bld.sh script). Therefore, it is necessary to make sure that any new LimeSuiteNG components are present in the cmake file install script. It is also a good practice to specify sub-package requirements in ``requirements:`` section using ``build:`` and ``host:`` directives, since sub-packages do not inherit the metapackage build, link and runtime requirements. Each sub-package contents must be tested individually in the ``test:`` section using ``commands:`` directive. In the ``about:`` section of a sub-package, only the ``summary:`` directive must have a unique sub-package description, other directives, such as ``license:`` and etc., can be copied from other sub-packages ``about:`` sections.

Since the metapackage recipe is used to build against different versions of dependencies, the conda_build_config.yaml file allows to specify multiple dependency versions. Dependency version specification will allow to build different packages for all version of dependencies in a single run. Be aware that this will increase build time significantly (builds can last hours).