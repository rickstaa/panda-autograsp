.. _doc_dev:

.. _panda-autograsp: https://github.com/rickstaa/panda-autograsp

Release documentation
===================================

Install requirements
--------------------------

Building `panda-autograsp`_'s documentation requires `sphinx`_,
the panda-autograsp package and several plugins. To prevent conflicts with system
installed python packages please install them inside a virtual environment. You can
create such an environment containing the required dependencies with the following
command:

.. code-block:: bash

    virtualenv --system-site-packages ~/venvs/panda_autograsp_docs
    source ~/venvs/panda_autograsp_docs/bin/activate
    pip install -r ./requirements/requirements_docs.txt


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
``make gh-pages`` command inside the ``panda-autograsp/docs``
directory.

.. _sphinx: http://www.sphinx-doc.org/en/master

.. warning::

    Please make sure you are on the ``melodic-devel`` branch while building the documentation. Otherwise,
    you will be greeted by errors.
