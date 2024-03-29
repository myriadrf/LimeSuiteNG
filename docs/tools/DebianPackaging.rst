Debian package building
=======================

This file describes how ``.deb`` packages work, how to build them, and how to test them working.

Quick introduction
------------------

``.deb`` packages are a way software is packaged for distribution for Debian
and distributions based on Debian (such as Ubuntu, Linux Mint and so on).

The package contains all the files that the package provides,
some metainformation about the package (name, version, maintainer, etc.),
information about package dependencies (required, optional, conflicts, etc.),
the installation and/or removal scripts,
as well as some additional information such as changelogs and manual files.

Internally the packages are managed with ``dpkg``,
but end users should be using front-ends such as ``apt`` or ``aptitude``.

Required files
--------------

To build a package, a number of files is required. Those files are:

- ``debian/control``
- ``debian/copyright``
- ``debian/changelog``
- ``debian/rules``

``debian/control``
^^^^^^^^^^^^^^^^^^

The file that has the metainformation about what packages are built from this source.

The first block of fields contains information about the source of the packages.
It contains information, such as the source's name, section, priority,
the required dependencies to build the package,
as well as some information about the maintainers and the original author's pages.

All the subsequent blocks describe each package that is being built by this source.
It contains such information as the package's name and description,
architecture, dependencies, other recommended packages.

``debian/copyright``
^^^^^^^^^^^^^^^^^^^^

This file contains information about the copyright the packages in this source are subject to.

``debian/changelog``
^^^^^^^^^^^^^^^^^^^^

This file contains information about the changelog of the package.

This file is used for a multitude of purposes, such as determining the version of the package,
the distribution and urgency of the package.

``debian/rules``
^^^^^^^^^^^^^^^^

The Makefile according to which the package is built.
This file usually includes calls to ``dh_*`` commands, which come from the ``debhelper`` package.

Building the package
--------------------

The packages in a given source are built by running the ``dpkg-buildpackage`` command,
provided by the ``dpkg-dev`` package, in the root folder of the source directory.
The ``debian`` directory with the required files must exist in that directory.
The command prepares and compiles the packages, then packages them into ``.deb`` package files.
Those files are then placed in the directory *above* the current working directory.

It is recommended to build the packages on the oldest still compatible version of the OS
to ensure maximum compatibility. This is due to different compiler versions having
different ``glibc`` requirements and newer versions of ``glibc`` being compatible with
programs built for older versions.

Packages built in Debian will still most likely work on Ubuntu and vice-versa.

Generated files
---------------

The files generated by building the packages are:

- Debian package files
- Debian package files with debug symbols for the binaries
- ``.buildinfo`` file
- ``.changes`` file

Debian package files
^^^^^^^^^^^^^^^^^^^^

The build generates a ``.deb`` file for each of the packages in the source.
The files are named ``${package_name}_${version}_{architecture}.deb``.

The package, which is just an archive, contains 3 files:

- ``debian-binary`` (currently always contains ``2.0``)
- ``control.tar.${compression}``
- ``data.tar.${compression}``

``control.tar``
"""""""""""""""

This archive contains, but is not limited to, these files:

- ``control``
- ``md5sums``
- ``preinst`` (optional)
- ``prerm`` (optional)
- ``postinst`` (optional)
- ``preinst`` (optional)

``control``
***********

This file contains information about the package itself, such as:

- Package name
- Package version
- Package architecture
- Package maintainer
- The size of the package when installed
- Package dependency information (dependencies, conflicts, etc.)
- Description of the package

``md5sums``
***********

The file contains MD5 checksums for every file in the ``data.tar`` archive.

``preinst``, ``prerm``, ``postinst``, ``preinst``
*************************************************

The scripts to run during the pre-installation (``preinst``), post-installation (``postinst``),
pre-removal (``prerm``) and post-removal (``postrm``) stages.

``data.tar``
""""""""""""

This archive contains all the files that would end up installed in the system when installing these packages,
in the locations that they would end up in, relative to the root directory.

Debian package files with debug symbols for the binaries
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The build generates a .ddeb file for each of the packages in the source containing binary files.
The files are named ``${package_name}-dbgsym_${version}_{architecture}.ddeb``.
The structure of these files is the same as normal ``.deb`` files, except that the ``data.tar`` file
contains only the debug symbols for the binaries in the non-debug symbol package.

``.buildinfo`` file
^^^^^^^^^^^^^^^^^^^

The file is named ``${source_name}_${version}_{architecture}.buildinfo``.
It contains information about the built packages, as well as information about the source.

The information includes:

- Name of the source
- Packages built
- Architecture of the packages
- Version of the packages
- MD5, SHA1 and SHA256 checksums of the files
- Status of the system when the package was built

``.changes`` file
^^^^^^^^^^^^^^^^^

The file is named ``${source_name}_${version}_{architecture}.changes``.
It contains information about the built packages.

The information includes:

- Name of the source
- Packages built
- Architecture of the packages
- Version of the packages
- Distribution information of the packages
- Maintainer information of the packages
- Brief description of the packages
- The latest entry in the changelog
- MD5, SHA1 and SHA256 checksums of the files

Installing Debian package files
-------------------------------

Installing the built ``.deb`` packages is done the same way as installing normal packages,
except instead of the package name you give the direct path to the file.
They will function exactly the same as any normal packages installed from repositories.

Example:

.. code-block:: bash

    apt install ./limesuite_24.0.0-1_amd64.deb ./liblimesuite24.0-1_24.0.0-1_amd64.deb

Testing packages installation
-----------------------------

One of the recommended ways to test if the packages are made correctly
would be to test them in a Virtual Machine (VM). This allows for incorrectly made packages
to not break the main system and to easily restore the system state
back to a known good working configuration. This also allows one to mass-test different
OS versions without needing to reinstall the OS or use an entirely separate physical computer
and/or storage device every time one would want to test a different one.

To test whether the devices themselves function properly with the built packages in a VM
requires passing through the devices into the VM. The exact way of doing that differs
for each virtualisation platform.

Docker
------

There exists a Docker file in this repository to generate the ``.deb`` packages.
To run the Docker script, use this command:

.. code-block:: bash

    docker build -o <output_folder> -f DebPackage<version>.Dockerfile .

More information
----------------

Some more expanded information from each section:

- `Required files under the 'debian' directory <https://www.debian.org/doc/manuals/maint-guide/dreq.en.html>`__
- `Building the package <https://www.debian.org/doc/manuals/maint-guide/build.en.html>`__
- `What is the format of a Debian binary package? <https://www.debian.org/doc/manuals/debian-faq/pkg-basics.en.html#deb-format>`__

More reading
------------

Helpful sources for starting out:

- `Basics of the Debian package management system <https://www.debian.org/doc/manuals/debian-faq/pkg-basics.en.html>`__
- `Debian New Maintainers' Guide <https://www.debian.org/doc/manuals/maint-guide/index.en.html>`__
