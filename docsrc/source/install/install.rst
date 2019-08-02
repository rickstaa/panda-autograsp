.. pre_prerequisites:

.. _panda_autograsp: https://github.com/BerkeleyAutomation/gqcnn

Prerequisites
==============================

ROS
-----------
The `panda_autograsp`_ package has only been tested with ``ROS Kinetic``. If you experience problems running this package in ``ROS Melodic`` please `create an issue <https://github.com/rickstaa/panda_autograsp/issues>`_.

Python
-----------

The `panda_autograsp`_ package has only been tested with ``Python 3.5``
, ``Python 3.6``, and ``Python 3.7``.


Ubuntu
-----------------

The `panda_autograsp`_ package has only been tested with
``Ubuntu 12.04``, ``Ubuntu 14.04`` and ``Ubuntu 16.04``.

Virtualenv
-------------------

We highly recommend using a Python environment management system like `Virtualenv <https://virtualenv.pypa.io/en/stable/>`_ or `conda <https://conda.io/en/latest/>`_ with the Pip and ROS installations.

Essential dependencies
------------------------------

The following packages are essential for using the `panda_autograsp`_ package.

- `ROS kinetic <https://wiki.ros.org/kinetic>`_
- `A number of additional ROS packages <#ROS-packages>`_
- `The libfreenect2 library <https://github.com/OpenKinect/libfreenect2>`_

ROS Packages
^^^^^^^^^^^^^^^^^^^^^^^

For the package to work, you need the following dependencies can be
installed by running the following command:

.. code-block:: bash

    sudo apt-get install ros-kinetic-moveit-ros-move-group ros-kinetic-controller-manager* ros-kinetic-moveit* ros-kinetic-effort-controllers ros-kinetic-joint-trajectory-controller ros-kinetic-gazebo-ros* ros-kinetic-rviz* libboost-filesystem-dev libjsoncpp-dev

Libfreenect2
^^^^^^^^^^^^^^^^^^^^^^^^

To use the package together with the Kinect camera, we need to install the
`libfreenect2 <https://github.com/OpenKinect/libfreenect2.git>`_ library. The documentation
for installing the `libfreenect2 <https://github.com/OpenKinect/libfreenect2.git>`_ can be
found in the `readme of the repository <https://github.com/OpenKinect/libfreenect2>`_.

Build the panda-autograsp package
========================================

After all the dependencies are installed, you can build the `panda_autograsp`_
package as follows:

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
            && catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build"

Install python package using pip
----------------------------------------

Since the package is still under development, you need to install the
`panda_autograsp`_ python package using the pip install ``-e`` flag:

.. code-block:: bash

    cd ~/panda_autograsp
    pip install -e .

Singularity Container
============================

We currently provide the following Nvidia compatible singularity
container for you to use with this package:

- **tensorflow-gpu-miniconda3-ros_kinetic-xenial.def**

The recipe for this containers can be found in the
`deep_robotics_singularity_containers repository <https://www.deep-robotics.dev>`_ or you can pull
the repository directly from the from the `singularity-hub.org <https://www.singularity-hub.org>`_ website.

1. Pull and build the container
-------------------------------------------
The containers in this repository can be pulled directly from
the `singularity-hub <https://www.singularity-hub.org`_.

.. code-block:: bash

    sudo singularity build --sandbox <CONTAINER_NAME> shub://rickstaa/panda_autograsp/ros-kinetic-cuda10-xenial:latest

2. Setup the right permissions
-------------------------------------------

If you also want to access the files in the container folder without
having to use the root user you can change the ``<YOUR_CONTAINER_NAME>``
folder permissions as follows:

#. Change the group owner to your user group.

    .. code-block:: bash

        sudo chgrp -R <YOUR_USER_NAME> ./<YOUR_CONTAINER_NAME>

#. Give your user group _read and write\_ access to the ``<YOUR_CONTAINER_NAME`` folder.

    .. code-block:: bash

        sudo chmod -R g+rwx  ./<YOUR_CONTAINER_NAME>

#. If successful the lock above the folder now disappeared.

3. Run the container
-------------------------------------------

After the container is built, you can use the singularity ``shell``,
``start`` and ``run`` commands to interact with the container.
You are advised to use the `run` command since this also sources
a ``.singularity_bashrc`` file that is present in each of the containers.
This file can be used as a ``.bashrc`` file. You can run the singularity
container using one of the following `run` commands:

- **With Nvidia GPU:** ``$ singularity run --nv <YOUR_CONTAINER_NAME>``
- **Without Nvidia GPU:** ``$ singularity run <YOUR_CONTAINER_NAME>``

.. note:: Additionally, you can also add the ``--writable`` parameter to the ``run command`` to receive write permissions.

4. Clone the repository and build the package
------------------------------------------------

After you are inside the singularity container, you have to build
the `panda_autograsp`_
`as explained above <#Build-the-panda-autograsp-package>`_.

Docker container
==============================

We do not yet provide a docker containers for this package.
