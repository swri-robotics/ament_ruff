ament_ruff
==================

Lint and format python files using the `ruff <https://github.com/astral-sh/ruff>`_ tool.

How to run the check from the command line?
-------------------------------------------

.. code:: sh

    usage: ament_ruff [-h] [--config path] [--reformat] [--xunit-file XUNIT_FILE] [paths ...]

    Check or format python code using ruff

    positional arguments:
      paths                 The files or directories to check

    options:
      -h, --help            show this help message and exit
      --config path         The path to a pyproject.toml or ruff.toml config file.
      --reformat            Reformat the files in place
      --xunit-file XUNIT_FILE
                            Generate a xunit compliant XML file

When using the option ``--reformat`` the proposed changes are applied in place.


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

The CMake integration is provided by the package `ament_cmake_ruff
<https://github.com/swri-robotics/ament_ruff/tree/main/ament_cmake_ruff>`_.