Updating Docs
=============

This page describes how to generate the Lime Suite NG documentation
(the very thing you're reading right now) on Linux or Windows platforms.

Prerequisites
-------------

Components needed to generate the Lime Suite NG documentation:

- `GCC`_
- `Ninja`_ (For Windows)
- `CMake`_
- `Python (>=3.8)`_
- `Doxygen`_
- `Graphviz`_

Setup
-----

To set up the environment for the documentation generation, you will need to set up a `Python virtual environment (venv)`_.

To set up the environment on Linux, run these commands:

.. code-block:: bash

    python -m venv venv # Create the virtual environment directory
    source venv/bin/activate # Activate the virtual environment
    pip install -r requirements.txt # Install all the required dependencies for the generation

To set up the environment on Windows, run these commands:

.. code-block:: doscon

    python -m venv venv # Create the virtual environment directory
    venv\Scripts\activate # Activate the virtual environment
    pip install -r requirements.txt # Install all the required dependencies for the generation

.. note::
    One should run these commands in the ``docs`` directory of cloned repository.

Generation
----------

In the ``docs`` directory, located in the root directory of the repository, while in the venv, run these commands:

.. code-block:: bash

    cmake -S .. -B ../build # 1. Generate the configuration files for the suite. For Windows include: -G Ninja
    cmake --build ../build -- doxygen # 2. Run only the Doxygen target to build doxygen

    # 3. Generate reference pages for API
    breathe-apidoc --generate class --members --force --output-dir developer/limesuiteng_api/api_member_list ../build/docs/developer/xml/
    breathe-apidoc --generate file --force --output-dir developer/limesuiteng_api/api_member_list ../build/docs/developer/xml/
    breathe-apidoc --generate struct --members --force --output-dir developer/limesuiteng_api/api_member_list ../build/docs/developer/xml/

    # 4. Remove redundant copies of the actual manual pages
    rm developer/limesuiteng_api/api_member_list/file/*dox.rst    # use `del` instead of `rm` on Windows
    
    # 5. Regenerate .rst manual pages from .dox manual pages
    python dox_converter.py

    # 6. Generate the final documentation
    make html 

.. tip::
    
    When running documentation build commands on Windows, do not forget to swap all instances of forward slash ``/`` with backslash ``\``.

.. important::
    These commands or the script must be run in the venv, otherwise it will fail.

For ease of convenience, ``docs`` directory contains scripts that automate the documentation build process for each platform.
On Linux, run ``generate_docs.sh`` script. On Windows, run ``generate_docs.bat`` script. Scripts support additional command line arguments:

- ``clean`` - Deletes all documentation build artifacts.
- ``rebuild`` - Runs the documentation build steps.

To simultaneously clean and rebuild the documentation, pass the arguments in the following order - ``generate_docs.sh clean rebuild``. 
Running script without any command line arguments, will trigger only the sphinx documentation build tool.

.. note::

    Scripts ``generate_docs.sh`` and ``generate_docs.bat`` must be run from ``docs`` directory.

After a successful generation the resulting documentation pages will be located in
``docs/_build/html`` directory.

Link checking
-------------

.. note::

    For Linux only.

To check whether all URLs in the documentation are valid, there exists a ``make`` target to automatically check the links:

.. code-block:: bash

    make checklinks

.. note::
    Run this in the ``docs`` directory while in the venv.

This will check whether all the URLs in the documentation are still valid and open a working webpage.
Any broken links will be shown in red and also listed in the ``_build/linkcheck/output.txt`` file.

Docker
------

.. note::

    For Linux only.

There also exists a Docker file to generate the documentation.
To generate this documentation using Docker, in the root directory of the repository run:

.. code-block:: bash

    docker build -o <output_location> -f GenerateDocumentation.Dockerfile .

This will run the whole documentation generation script and place the generated HTML in the specified directory.

More information
----------------

For more information about how to set up and write the documentation,
check out the `MyriadRF Handbook`_.

.. _`GCC`: https://gcc.gnu.org/
.. _`Ninja`: https://ninja-build.org/
.. _`CMake`: https://cmake.org/
.. _`Python (>=3.8)`: https://www.python.org/downloads/release/python-3818/
.. _`Doxygen`: https://www.doxygen.nl/
.. _`Graphviz`: https://graphviz.org/
.. _`Python virtual environment (venv)`: https://docs.python.org/3.8/library/venv.html
.. _`MyriadRF Handbook`: https://handbook.myriadrf.org/
