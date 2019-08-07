.. _doc_dev:

.. _panda_autograsp: https://github.com/rickstaa/panda_autograsp

Release documentation
===================================

Building
--------------------------
Building `panda_autograsp`_'s documentation requires `sphinx <http://www.sphinx-doc.org/en/master>`_
and a few plugins.

To install the required dependencies, go to the ``panda_autograsp``
directory and run ::

    $ pip install .[docs]

Then, go to the ``docsrc`` directory and run ``make html``
to build the documentation. This command will generate
a set of web pages inside the ``docsrc/_build`` directory.

Deploying
---------------------------
To deploy documentation to the Github Pages site for the repository,
simply push the documentation to the ``master`` branch and run the ``make gh-pages`` command inside the ``panda_autograsp/docsrc``
directory.
