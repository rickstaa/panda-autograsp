.. _panda_autograsp: https://github.com/rickstaa/panda_autograsp

Singularity container instructions
===========================================

Container aliases
--------------------------------

To increase convenience a number of bash aliases were added to the containers:

- ``agrasp``: activates the autograsp conda environment
- ``dgrasp``: deactivates the autograsp conda environment
- ``dconda``: deactivates the conda enviroment
- ``rsource``: sources the ROS setup.bash file
- ``rossu``: sources the setup.bash file at ``./devel/setup.bash``
- ``pbuild``: `panda_autograsp`_ package build command. Make sure your inside the panda_autograsp catkin workspace.

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
your own ``~/.singularity_bashrc`` file in your home folder. If such a ``~/.singularity_bashrc``
file is present in your home folder, it will be sourced after the container its
``~/.singularity_bashrc`` file.

Container permissions
--------------------------------

By default your user can not write to the container folder when begin outside
of the container. If you did build the singularity container as a writeable folder
you can give your user write and read access from outside the singularity
container by:

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
