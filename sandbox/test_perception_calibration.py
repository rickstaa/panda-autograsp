"""
Script to test autolab calibration capabilities.

"""
import os

from perception import CameraChessboardRegistration, ChessboardRegistrationResult
from perception import Kinect2Sensor
from autolab_core import Point, PointCloud, RigidTransform, YamlConfig

## Create sensor object ##
sensor = Kinect2Sensor()

## Get regestration config file ##
CAMERA_CALIB_CFG = os.path.join(os.path.dirname(
    os.path.realpath(__file__)), "../cfg/calib/register_camera.yaml")
config = YamlConfig(CAMERA_CALIB_CFG)


## Create camera calibration object ##
calibrator = CameraChessboardRegistration()
calibrator.register(sensor, config['sensors']['kinect2'])
print("test")

