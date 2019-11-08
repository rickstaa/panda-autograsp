.. _doc_dev:

.. _panda_autograsp: https://github.com/rickstaa/panda_autograsp

Release documentation
===================================

Install requirements
--------------------------
Building `panda_autograsp`_'s documentation requires `sphinx <http://www.sphinx-doc.org/en/master>`_,
and `Doxygen <http://www.doxygen.nl/download.html>`_ the panda_autograsp package and several plugins.

The easiest way of installing Doxygen is by installing the binary ``sudo apt-get install doxygen``.
For the other available installation methods see the `Doxygen documentation <http://www.doxygen.nl/download.html>`_).
After Doxygen is installed the other required dependencies can be installed
by going into the ``panda_autograsp`` directory and running the following commands::

 $ pip install .
 $ pip install .[docs]

.. note::

    If you get a ``Could NOT find FLEX (missing: FLEX_EXECUTABLE)`` error while trying to install
    Doxygen please install the flex and bison packages using the following commands:

    .. code-block:: bash

        $ sudo apt-get install flex
        $ sudo apt-get install bison

Build the documentation
--------------------------
To build the documentation go into the ``docs`` directory and run the
``make html`` command. This command will generate the html documentation
inside the ``docs/build`` directory.

Deploying
---------------------------
To deploy documentation to the Github Pages site for the repository,
push the documentation to the ``master`` branch and run the
``make gh-pages`` command inside the ``panda_autograsp/docsrc``
directory.
