# GQCNN documentation
This file contains the documentation I made while setting up the [GQCNN](https://berkeleyautomation.github.io/gqcnn) of BerkeleyAutomation.

## Depencencies
To use the `panda_autograsp` package you need the following packages:

- [BerkeleyAutomation/gqcnn (catkin)](https://github.com/BerkeleyAutomation/gqcnn)
- [BerkeleyAutomation/autolab_core (catkin)](https://github.com/BerkeleyAutomation/autolab_core)
- [BerkeleyAutomation/perception](https://github.com/BerkeleyAutomation/perception)
- OPTIONAL: [BerkleyAutomation/pyrender](https://github.com/mmatl/pyrender)

### Python policy


#### Python policy run command
To test out the pcakge we can use the python policy. The package can be evoked using the following command:

`python examples/policy.py <model_name> --depth_image <depth_image_filename> --segmask <segmask_filename> --camera_intr <camera_intr_filename>`

**Example:**

`python examples/policy.py GQCNN-2.0 --depth_image '/home/ricks/singularity/containers/panda_autograsp/panda_autograsp_ws/src/gqcnn/data/examples/single_object/primesense/depth_0.npy' --segmask '/home/ricks/singularity/containers/panda_autograsp/panda_autograsp_ws/src/gqcnn/data/examples/single_object/primesense/segmask_0.png' --camera_intr '/home/ricks/singularity/containers/panda_autograsp/panda_autograsp_ws/src/gqcnn/data/calib/primesense/primesense.intr'`


##### Possible errors you can encounter

###### Pip error
- remove pip -> `apt-get remove python-pip`
- install pip -> `curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python get-pip.py && rm get-pip.py`
- If pip wrapper error use `python -m pip install .` to install gqcnn