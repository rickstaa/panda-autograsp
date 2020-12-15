# panda-autograsp

[![Codacy Badge](https://app.codacy.com/project/badge/Grade/087fda2f0f4c423cb561745ab7afdba7)](https://www.codacy.com/gh/rickstaa/panda-autograsp/dashboard?utm_source=github.com&utm_medium=referral&utm_content=rickstaa/panda-autograsp&utm_campaign=Badge_Grade)
[![Contributions](https://img.shields.io/badge/contributions-welcome-orange.svg)](contributing.md)
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/rickstaa/panda-autograsp)](https://github.com/rickstaa/panda-autograsp/releases)
[![Python 3](https://img.shields.io/badge/python%203-3.7%20%7C%203.6%20%7C%203.5-yellow.svg)](https://www.python.org/)
[![Python 2](https://img.shields.io/badge/python%202-2.7%20%7C%202.6%20%7C%202.5-brightgreen.svg)](https://www.python.org/)
[![ROS versions](https://img.shields.io/badge/ROS%20versions-Melodic%20%7C%20Kinectic-brightgreen)](https://wiki.ros.org)

:bulb: You are on the default (Kinect2) branch. This branch is optimized to work with the Kinect2 camera. To use the package with the RealSense cameras, see the [melodic-devel-realsense branch](https://github.com/rickstaa/panda-autograsp/tree/melodic-devel-realsense).

## Package Overview

The panda-autograsp is an autonomous ROS based grasping solution that works with the [Panda Emika Franka robot](https://www.franka.de/panda/). In this grasping solution, several opensource grasping solutions are implemented on the [Panda Emika Franka robots](https://www.franka.de/panda/) robot. These solutions work both on a physical as well as a simulated version of the panda robot. A simulated version of the panda robot is already shipped with this package.

-   [BerkleyAutomation/gqcnn](https://github.com/BerkeleyAutomation/gqcnn)

## Installation and Usage

Please see the [docs](https://rickstaa.github.io/panda-autograsp/) for installation and usage instructions.

## Known limitations

-   There package is currently not working with the simulated camera (see [#158](https://github.com/rickstaa/panda-autograsp/issues/158).

-   Since the package is written in python2.7 and this version already reached EOL, the dependencies are quite fragile. The `setup.py` install method might, therefore fail. If this is the case, please install the dependencies using the `./requirements/requirements.txt` file. This can be solved by porting the package to ROS Noetic (see [#163](<https://github.com/rickstaa/panda-autograsp/issues/163>)).

## LICENSE

The main code if this repository is licensed under an **MIT license**. If a LICENCE file is present in a submodule this licence has to be respected but ONLY for the files contained in this submodule.

## References

-   GQ-CNN and FC-GQ-CNN created by [@berkeleyautomation](https://berkeleyautomation.github.io/gqcnn)
-   Icon created with svgs made by [@freepik](https://www.freepik.com/) from [www.flaticon.com](https://www.flaticon.com/authors/eucalyp)
