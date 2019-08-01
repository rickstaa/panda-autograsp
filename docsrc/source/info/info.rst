.. info:

.. _panda_autograsp: https://github.com/BerkeleyAutomation/gqcnn

Package information
=========================
`Panda_autograsp`_ is an autonomous ROS based grasping solution that works with the
`Panda Emika Franka robot <https://www.franka.de/panda>`_. In this grasping solution
a number of opensource grasping solutions are implemented on the
`Panda Emika Franka robots <https://www.franka.de/panda>`_ robot.The `panda_autograsp`_ package
currently contains the following grasping algorithms:


-   `BerkleyAutomation/gqcnn <https://github.com/BerkeleyAutomation/gqcnn>`_

.. note:: All of these grasping algorithms can be used with both a real Franka Emika Panda robot as well as a as a (included) simulated version of the franka panda robot.

Grasping solutions
---------------------------

GQCNN
^^^^^^^^^^^^

GQ-CNNs are neural network architectures that take as input a depth image
and grasp,and output the predicted probability that the grasp will successfully
hold the object while lifting, transporting, and shaking the object.

.. figure:: https://berkeleyautomation.github.io/gqcnn/_images/gqcnn1.png
   :width: 100%
   :align: center

   Original GQ-CNN architecture from `Dex-Net 2.0`_.

.. figure:: https://berkeleyautomation.github.io/gqcnn/_images/fcgqcnn_arch_diagram.png
   :width: 100%
   :align: center

   Alternate faster GQ-CNN architecture from `FC-GQ-CNN`_.

The GQ-CNN weights are trained on datasets of synthetic point clouds, parallel
jaw grasps,and grasp metrics generated from physics-based models with domain
randomization for sim-to-real transfer. See the ongoing `Dexterity Network (Dex-Net)`_
project for more information.

.. _Dexterity Network (Dex-Net): https://berkeleyautomation.github.io/dex-net
.. _Dex-Net 2.0: https://berkeleyautomation.github.io/dex-net/#dexnet_2
.. _FC-GQ-CNN: https://berkeleyautomation.github.io/fcgqcnn


.. note:: The text and the images above are taken from the original `GQCNN documentation <https://berkeleyautomation.github.io/gqcnn/>`_.

Other submodules contained in this package
-------------------------------------------------

- `deep_robotics_singularity_recipes <https://github.com/rickstaa/deep_robotics_singularity_recipes>`_
- `franka_ros <https://github.com/rickstaa/franka_ros>`_
- `movit_tutorials <https://github.com/ros-planning/moveit_tutorials>`_
- `panda_movit_config <https://github.com/rickstaa/panda_moveit_config>`_
- `panda_simulation <https://github.com/rickstaa/panda_simulation>`_
- `iai_kinect2 <https://github.com/code-iai/iai_kinect2>`_
