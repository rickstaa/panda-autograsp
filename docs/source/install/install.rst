.. _install:

.. _panda_autograsp: https://github.com/rickstaa/panda_autograsp

Prerequisites
==============================

ROS
-----------
The `panda_autograsp`_ package has only been tested with ``ROS Melodic``.
The ROS melodic installation instructions can be found `here <https://wiki.ros.org/melodic>`_.
You further also need the following ROS packages:

.. code-block:: bash

    $ sudo apt-get install ros-melodic-moveit-ros-move-group ros-melodic-controller-manager* ros-melodic-moveit* ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller ros-melodic-gazebo-ros* ros-melodic-rviz* libboost-filesystem-dev libjsoncpp-dev

Python
-----------

As ROS melodic does not yet fully support python 3, the `panda_autograsp`_
package has only been tested with ``Python 2.7``.


Ubuntu
-----------------

The `panda_autograsp`_ package has only been tested with ubuntu ``18.04``.

Libfreenect2 library
----------------------

To use the package together with the Kinect camera, we need to install the
`libfreenect2 <https://github.com/OpenKinect/libfreenect2.git>`_ library. The documentation
for installing the `libfreenect2 <https://github.com/OpenKinect/libfreenect2.git>`_ can be
found in the `readme of the repository <https://github.com/OpenKinect/libfreenect2>`_.

Virtualenv
-------------------

Although the package can be installed on the system python you are advised
to use a python environment management system like `Virtualenv <https://virtualenv.pypa.io/en/stable/>`_
or `conda <https://conda.io/en/latest/>`_.

Package build from source instructions
========================================

Clone the repository and build the package
--------------------------------------------------------

Clone or download the `panda_autograsp`_ catkin package from Github:

.. code-block:: bash

    $ bash -c "mkdir -p /panda_autograsp_ws \
    && cd /panda_autograsp_ws \
    && source /opt/ros/melodic/setup.sh \
    && git clone --recursive https://github.com/rickstaa/panda_autograsp_ws.git src \
    && rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka \
    && catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build -Dfreenect2_DIR=/opt/freenect2/lib/cmake/freenect2"

Install the ros system and python dependencies
----------------------------------------

Install the ros package dependencies using the following command:

.. code-block:: bash

    rosdep install --from-paths src --ignore-src --rosdistro melodic -y


Install python package using pip
----------------------------------------

As ``rosdep`` does not yet support specifying specific versions for
python packages, we need to install some additional packages using
the `pip install command`. To ease this process a ``setup.py`` file
was created. This file can be invoked using the following commands:

Build the package
-------------------------

The catkin package can be build by executing one of the following commands:

```
catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build -Dfreenect2_DIR=/opt/freenect2/lib/cmake/freenect2"
```

    $ cd ~/panda_autograsp
    $ pip install .

Singularity Container installation instructions
==================================================

Nvidia container
---------------------------------------------
A ready to run Nividia compatible singularity
container is provided. This container can be build using the
recipe files found in the ``panda_autograsp/containers/singularity``
folder or by pulling directly from the `singularity-hub.org <https://www.singularity-hub.org>`_
container registry.

1. Build the container
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The containers in this repository can be pulled directly from
the `singularity-hub <https://www.singularity-hub.org>`_ container
registry as follows:

.. code-block:: bash

    $ build <CONTAINER_NAME>.simg shub://rickstaa/panda_autograsp:ros-melodic-cuda10-xenial

Go to the ``panda_autograsp/containers/singularity`` folder and
built the container using the recipe file. This is done by running the
following command:

.. code-block:: bash

    $ sudo singularity <CONTAINER_NAME>.simg shub://rickstaa/panda_autograsp:ros-melodic-cuda10-xenial

You can also add the ``--sandbox`` argument to build the container as
a writeable folder.

.. warning:: You need root access to build from a recipe file.

2. Run the container
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After te container has been build run it using the
``singularity run --writable <YOUR_CONTAINER_NAME>`` command.

3. Clone the repository and build the package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After you are inside the singularity container, you have to build
the `panda_autograsp`_
`as explained above <#Build-the-panda-autograsp-package>`_.

.. warning::
 As explained in `issue <https://answers.ros.org/question/256886/conflict-anaconda-vs-ros-catking_pkg-not-found/>`_
 there exist some conflicts between anaconda3 and ROS melodic. As the singularity image provided above automatically starts the ``autograsp``
 conda environment you first need to disable this anaconda environment before you can build the catkin package. After the
 catkin package is built you can enable the anaconda environment again and install the ``autograsp`` package.

4. Add additional permissions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you did build the singularity container as a writeable folder,
you could give your user write and read access from outside the singularity
container by:

#. Changing the group owner to your user group.

.. code-block:: bash

    $ sudo chgrp -R <YOUR_USER_NAME> ./<YOUR_CONTAINER_NAME>

#. Giving your user group read and write access to the ``<YOUR_CONTAINER_NAME`` folder.

.. code-block:: bash

    $ sudo chmod -R g+rwx ./<YOUR_CONTAINER_NAME>

AMD compatible container
----------------------------

Currently, singularity does not yet support AMD graphics. This option is planned to be included in the next
release `see this announcement <https://sylabs.io/2019/06/towards-generalized-gpu-support-in-the-singularity-container-runtime-an-isc-preview-involving-amd-radeon-instinct-accelerators-and-the-rocm-open-software-platform/>`_.

Docker container installation instructions
===========================================

We do not yet provide a docker container for this package.
