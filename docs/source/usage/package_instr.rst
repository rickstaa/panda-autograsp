
Package use instructions
===================================

The panda_autograsp solution can be started both with a real physical system
alternatively, with a simulated system.

Physical robot
--------------------

You can start the `panda_autograsp`_ solution with the real robot using
the following command:

.. code-block:: bash

    roslaunch panda_autograsp autograsp.launch solution:=<GRASPING_SOLUTION>

Simulated robot
-------------------

To start the `panda_autograsp`_ solution with the simulation use this command:

.. code-block:: bash

    roslaunch panda_autograsp autograsp.launch real:=True solution:=<GRASPING_SOLUTIOn>

Available grasping solutions
--------------------------------

Currently, the following grasping solutions are supported:

#. **GQCNN-4.0-PJ**: For `Dex-Net 4.0`_, trained on images of objects in clutter with parameters for a PhotoNeo PhoXi S.
#. **FC-GQCNN-4.0-PJ**: For `FC-GQ-CNN`_, trained on images of objects in clutter with parameters for a PhotoNeo PhoXi S.

.. _Dex-Net 4.0: https://berkeleyautomation.github.io/dex-net/#dexnet_4
.. _FC-GQ-CNN: https://berkeleyautomation.github.io/fcgqcnn
