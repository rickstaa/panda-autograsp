"""
Script to test autolab calibration capabilities.

"""
## TODO: Working here!

import os

from perception import CameraChessboardRegistration
from perception import Kinect2Sensor
from autolab_core import Point, PointCloud, YamlConfig

## Create sensor object ##
sensor = Kinect2Sensor()
sensor.start()

## Get regestration config file ##
CAMERA_CALIB_CFG = os.path.abspath(os.path.join(os.path.dirname(
    os.path.realpath(__file__)), "../cfg/calib/register_sensor.yaml"))
config = YamlConfig(CAMERA_CALIB_CFG)

## Restructure config
new_config = config["chessboard_registration"]
new_config.update(config['sensors']['kinect2']["registration_config"])

## Create camera calibration object ##
calibrator = CameraChessboardRegistration()
calibration_result = calibrator.register(sensor, new_config)
print("test")

