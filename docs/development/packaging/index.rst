Packaging
=========

This section provides necessary information for maintaining conda packages and it's recipes. Before publishing packages it is recommended to test build and run them on a local machine. To set up package build and test environment, please check out :ref:`radioconda-setup-ref` set up page which guides through radioconda prompt and conda environment set up. Currently LimeSuiteNG project contains 3 conda recipes: recipe to build gnuradio-limesuiteng package, recipe to build basic LimeSuiteNG components (libraries, utilities and binaries) and a LimeSuiteNG metapackage recipe. The gnuradio-limesuiteng and LimeSuiteNG component conda recipes are made for experimental purposes to quickly build and test individual LimeSuiteNG component packages with different dependency versions. LimeSuiteNG metapackage conda recipe is the main recipe that should be used for public package updates and releases. Metapackage recipe builds entire LimeSuiteNG project with all possible components enabled and organizes built components into separate sub-packages. Recipes are stored in the follwoing directories relative to the project root directory:

#. ``.conda\recipe`` - Minimal LimeSuiteNG package recipe (libraries, utilities and binaries) .
#. ``.conda\metapkg`` - LimeSuiteNG metapackage recipe.
#. ``plugins\gr-limesuiteng\.conda\recipe`` - gnuradio-limesuiteng package recipe.

More about each recipe here:

.. toctree::

   Minimal LimeSuiteNG recipe <condaSuiteRecipe.rst>
   Gnuradio-limesuiteng recipe <condaPluginRecipe.rst>
   Metapackage recipe <condaMetaPkgRecipe.rst>
