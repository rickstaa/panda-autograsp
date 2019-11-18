Singularity container instructions
===========================================

Container aliases
--------------------------------

To increase convenience a number of bash aliases were added to the containers:

    - **agrasp**: activates the autograsp conda environment
    - **dgrasp**: deactivates the autograsp conda environment
    - **dconda**: deactivates the conda enviroment
    - **rsource**: sources the ROS setup.bash file
    - **rossu** sources the setup.bash file at ``./devel/setup.bash``
    - **pbuild**: `panda_autograsp`_ package build command. Make sure your inside the panda_autograsp catkin workspace.

You can unset these aliases or add additional aliases by
creating a ``~/.singularity_bash_aliases`` file in your user home directory.
If such a ``~/.singularity_bash_aliases``
file is present in your home folder, it will be sourced after the container its
``~/.singularity_bash_aliases`` file.

Container .bashrc
---------------------------------

A ``.singularity_bashrc`` file is present inside the containers. This file is
sourced when you run the container. You can overload all of the commands,
aliases and settings contained in this ``.singularity_bashrc`` file by creating
your own ``~/.singularity_bashrc`` file in your home folder. If
such a ``~/.singularity_bashrc`` file is present in your home folder,
it will be sourced after the container its
``~/.singularity_bashrc`` file.

Container run instructions
--------------------------------------

You can use the singularity ``shell``,
``start`` and ``run`` commands to interact with the container.
You are advised to use the ``run`` command since this also sources
a ``.singularity_bashrc`` file that is present in each of the containers.
This file can be used as a ``.bashrc`` file. You can run the singularity
container using one of the following ``run`` commands:

- **With Nvidia GPU:** ``singularity run --nv <YOUR_CONTAINER_NAME>``
- **Without Nvidia GPU:** ``singularity run <YOUR_CONTAINER_NAME>``

.. note:: Additionally, you can also add the ``--writable`` parameter to the ``run command`` to receive write permissions.

.. warning::

    Please not that singularity currently only links to the NVIDIA drivers when you are using the container in read mode.
    As a result when you start the container using the ``--writable`` tag you can not leverage the GPU computation capabilities
    of your NVIDIA graphics card.

Container permissions
--------------------------------

By default, your user can not write to the container folder from outside
of the container. If you did build the singularity container as a writeable
folder, you could give your user write and read access from outside the
singularity container by:

#. Changing the group owner to your user group.

.. code-block:: bash

    sudo chgrp -R <YOUR_USER_NAME> ./<YOUR_CONTAINER_NAME>

#. Giving your user group _read and write\_ access to the ``<YOUR_CONTAINER_NAME`` folder.

.. code-block:: bash

    sudo chmod -R g+rwx ./<YOUR_CONTAINER_NAME>

Add a visual code IDE to the singularity container
------------------------------------------------------------

Visual studio code can be added to the singularity container in order to enable
easy code debugging. This is done as follows:

    #. Run your container using the ``sudo singularity run --writable <YOUR_CONTAINER_NAME>``
    #. Install visual code or visual code-insiders using the following bash commands:

.. code-block:: bash

    curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
    sudo install -o root -g root -m 644 microsoft.gpg /etc/apt/trusted.gpg.d/
    sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
    sudo apt-get install apt-transport-https
    sudo apt-get update
    sudo apt-get install code # or code-insiders

.. note::

    Since visual code requires the /run folder you need to add the ``-B /run`` argument when running a singularity container.
    For more information, see `this issue <https://github.com/sylabs/singularity/issues/3609>`_.

.. _panda_autograsp: https://github.com/rickstaa/panda_autograsp
