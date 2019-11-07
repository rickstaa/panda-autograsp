.. _info:

.. _panda_autograsp: https://github.com/rickstaa/panda_autograsp

Package information
=========================
`Panda_autograsp`_ is an autonomous ROS based grasping solution that works with the
`Panda Emika Franka robot <https://www.franka.de/panda>`_. In this grasping solution,
several opensource grasping solutions are implemented on the
`Panda Emika Franka robot <https://www.franka.de/panda>`_ robot. The `panda_autograsp`_ package
currently contains the following grasping algorithm:

- `BerkleyAutomation/gqcnn <https://github.com/BerkeleyAutomation/gqcnn>`_

.. note::

    These solutions work both on a physical as well as a simulated version of the panda robot.
    A simulated version of the panda robot is shipped with this package.

Package overview
---------------------------

.. figure:: ../_images/algorithm_overview.svg
    :width: 100%
    :scale: 90%
    :align: center

    Flow diagram of the `panda_autograsp`_ system.

Grasping solutions
---------------------------

GQ-CNN & F-GQ-CNN
^^^^^^^^^^^^^^^^^^^^^^^^^^^

GQ-CNNs are neural network architectures that take as input a depth image
and grasp, and output the predicted probability that the grasp will
successfully hold the object while lifting, transporting, and shaking
the object.

.. figure:: https://berkeleyautomation.github.io/gqcnn/_images/gqcnn1.png
    :width: 100%
    :align: center

    Original GQ-CNN architecture from `Dex-Net 2.0`_.

.. figure:: https://berkeleyautomation.github.io/gqcnn/_images/fcgqcnn_arch_diagram.png
    :width: 100%
    :align: center

    Alternate faster GQ-CNN architecture from `FC-GQ-CNN`_.

The GQ-CNN weights were trained on datasets of synthetic point clouds, parallel
jaw grasps, and grasp metrics generated from physics-based models with domain
randomization for sim-to-real transfer. See the ongoing
`Dexterity Network (Dex-Net)`_ project for more information.

.. note::

    Currently, only the parallel jaw variants of the GQ-CNN and FC-GQ-CNN networks are supported by the `panda_autograsp`_ package. As a result, for the GQ-CNN's and FC-GQ-CNN, the following
    network models can be chosen:

        - **GQCNN-2.0**: Deep Learning to Plan Robust Grasps with Synthetic Point Clouds and Analytic Grasp Metrics. Trained on `Dex-Net 2.0`_.
        - **GQCNN-2.1**: Extension off GQ-CNN 2.0. The network that was trained on `Dex-Net 2.0`_ is improved using RL in simulations.
        - **GQCNN-4.0-PJ**: Improvement of the GQCNN-2.0 which computes more accurate grasps. This network module does additionally state whether it is better to grasp an object using suction or a parallel jaw gripper.
        - **FC-GQCNN-4.0-PJ**: Modification of GQCNN-4.0-PJ in which a fully connected grasp quality CNN (`FC-GQ-CNN`_) is used. This model has a faster grasp computation time and a more accurate grasp.

    You can switch between these networks by supplying the `panda_autograsp`_ launch file with the ``model_type:=<MODEL_NAME>`` argument.

.. _Dexterity Network (Dex-Net): https://berkeleyautomation.github.io/dex-net
.. _Dex-Net 2.0: https://berkeleyautomation.github.io/dex-net/#dexnet_2
.. _FC-GQ-CNN: https://berkeleyautomation.github.io/fcgqcnn
