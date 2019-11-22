.. _doc_dev:

.. _panda_autograsp: https://github.com/rickstaa/panda_autograsp

Release documentation
===================================

Install requirements
--------------------------
Building `panda_autograsp`_'s documentation requires `sphinx`_,
the panda_autograsp package and several plugins.
After `sphinx`_  is installed the other required dependencies can be installed
by going into the ``panda_autograsp`` directory and running the
following commands:

.. code-block:: bash

    pip install .
    pip install .[docs]


Build the documentation
--------------------------
To build the documentation go into the ``docs`` directory and run the
``make html`` command. This command will generate the html documentation
inside the ``docs/build`` directory.

.. note::

    Make sure the catkin ws has been build and sourced.

Deploying
---------------------------
To deploy documentation to the Github Pages site for the repository,
push the documentation to the ``melodic-devel`` branch and run the
``make gh-pages`` command inside the ``panda_autograsp/docsrc``
directory.

.. _sphinx: http://www.sphinx-doc.org/en/master

.. warning::

    Please make sure you are on the ``melodic-devel`` branch while building the documentation. Otherwise,
    you will be greeted by errors.
