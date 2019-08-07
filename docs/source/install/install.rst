.. _install:

.. _panda_autograsp: https://github.com/rickstaa/panda_autograsp

Prerequisites
==============================

ROS
-----------
The `panda_autograsp`_ package has only been tested with ``ROS Kinetic``.
The ROS kinetic installation instructions can be found `here <https://wiki.ros.org/kinetic>`_.
You further also need the following ROS packages:

.. code-block:: bash

    sudo apt-get install ros-kinetic-moveit-ros-move-group ros-kinetic-controller-manager* ros-kinetic-moveit* ros-kinetic-effort-controllers ros-kinetic-joint-trajectory-controller ros-kinetic-gazebo-ros* ros-kinetic-rviz* libboost-filesystem-dev libjsoncpp-dev

Python
-----------

The `panda_autograsp`_ package has only been tested with ``Python 3.5``
, ``Python 3.6``, and ``Python 3.7``.


Ubuntu
-----------------

The `panda_autograsp`_ package has only been tested with
``Ubuntu 12.04``, ``Ubuntu 14.04`` and ``Ubuntu 16.04``.


Libfreenect2 library
----------------------

To use the package together with the Kinect camera, we need to install the
`libfreenect2 <https://github.com/OpenKinect/libfreenect2.git>`_ library. The documentation
for installing the `libfreenect2 <https://github.com/OpenKinect/libfreenect2.git>`_ can be
found in the `readme of the repository <https://github.com/OpenKinect/libfreenect2>`_.

Virtualenv
-------------------

We highly recommend using a Python environment management system like `Virtualenv <https://virtualenv.pypa.io/en/stable/>`_ or `conda <https://conda.io/en/latest/>`_ with the Pip and ROS installations.

Package build instructions
========================================

Clone the repository and build the package
--------------------------------------------------------

Clone or download the `panda_autograsp`_ package from Github
and build the catkin_package
using the following command.

.. code-block:: bash

    bash -c "mkdir -p /panda_autograsp_ws \
            && cd /panda_autograsp_ws \
            && source /opt/ros/kinetic/setup.sh \
            && git clone --recursive https://github.com/rickstaa/panda_autograsp_ws.git src \
            && rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka \
            && catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build -Dfreenect2_DIR=/opt/freenect2/lib/cmake/freenect2"

Install python package using pip
----------------------------------------

Since the package is still under development, you need to install the
`panda_autograsp`_ python package using the pip install ``-e`` flag:

.. code-block:: bash

    cd ~/panda_autograsp
    pip install -e .

Singularity Container installation instructions
==================================================

We currently provide  several Nvidia compatible singularity
container for you to use with this package.
These containers can be build using the recipe files found in the
``panda_autograsp/containers/singularity`` folder or by
pulling directly from the `singularity-hub.org <https://www.singularity-hub.org>`_
container registry.

1. Build the container
-------------------------------------------
The containers in this repository can be pulled directly from
the `singularity-hub <https://www.singularity-hub.org>`_ container
registry as follows:

.. code-block:: bash

    build <CONTAINER_NAME>.simg shub://rickstaa/panda_autograsp:ros-kinetic-cuda10-xenial

It can also be built from the recipe file using the following command:

.. code-block:: bash

    sudo singularity <CONTAINER_NAME>.simg shub://rickstaa/panda_autograsp:ros-kinetic-cuda10-xenial

You can also add the ``--sandbox`` argument to build the container as
a writeable folder.

.. warning:: You need root access to build from a recipe file.

2. Run the container
-------------------------------------------

After te container has been build run it using the ``singularity run --writable <YOUR_CONTAINER_NAME>`` command.

3. Clone the repository and build the package
------------------------------------------------

After you are inside the singularity container, you have to build
the `panda_autograsp`_
`as explained above <#Build-the-panda-autograsp-package>`_.

.. warning::
    As explained in `issue <https://answers.ros.org/question/256886/conflict-anaconda-vs-ros-catking_pkg-not-found/>`_
    there exist some conflicts between anaconda3 and ROS kinetic. As the singularity image provided above automatically starts the ``autograsp``
    conda environment you first need to disable this anaconda environment before you can build the catkin package. After the
    catkin package is build you can enable the anaconda environment again and install the ``autograsp`` package.

4. Add additional permissions
-------------------------------

If you did build the singularity container as a writable folder
you can give your user write and read access from outside the singularity
container by:

#. Changing the group owner to your user group.

    .. code-block:: bash

        sudo chgrp -R <YOUR_USER_NAME> ./<YOUR_CONTAINER_NAME>

#. Giving your user group _read and write\_ access to the ``<YOUR_CONTAINER_NAME`` folder.

    .. code-block:: bash

        sudo chmod -R g+rwx  ./<YOUR_CONTAINER_NAME>

Docker container installation instructions
===========================================

We do not yet provide a docker containers for this package.
