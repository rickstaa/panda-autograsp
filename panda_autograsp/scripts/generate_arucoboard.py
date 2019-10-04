"""ArucoBoard generator
This script can be used to generate a aruco marker board. This board is used in the camera pose estimation.

Printing instructions:
    - Make sure Scale to fit is selected.
    - Verify the size of the markers after the board is printed. If
      size does not match change the settings below or in the detection alogrithm.
"""

## Main imports ##
import os
import pickle
import numpy as np
import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import pickle

## Get required paths ##
save_dir_path = os.path.abspath(os.path.dirname(
    os.path.realpath(__file__)))

## Script settings ##
MARKERS_X = 4
MARKERS_Y = 6
MARKER_LENGTH = 0.032                 # [M]
MARKER_SEPARATION = 0.009             # [M]
ARUCO_DICT_TYPE = aruco.DICT_6X6_50   # [ROWSxCOLUMNS_LIBRARY_SIZE]
IM_OUT_SIZE = (1100, 1681)            # Pixel size == Pattern size
MARGIN_SIZE = 0

## Save config settings ##
config_dict = {
    "MARKERS_X": MARKERS_X,
    "MARKERS_Y": MARKERS_Y,
    "MARKER_LENGTH": 0.032,
    "MARKER_SEPARATION": 0.009,
    "ARUCO_DICT_TYPE": ARUCO_DICT_TYPE,
    "IM_OUT_SIZE": IM_OUT_SIZE,
    "MARGIN_SIZE": MARGIN_SIZE
}
config_save_str = os.path.abspath(os.path.join(save_dir_path, "../cfg/_cfg/aruco_config.dict"))
with open(config_save_str, 'wb') as config_dict_file:
  pickle.dump(config_dict, config_dict_file)

## Create aruco gridboard ##
ARUCO_DICT = aruco.Dictionary_get(ARUCO_DICT_TYPE)
gridboard = aruco.GridBoard_create(
    markersX=MARKERS_X,
    markersY=MARKERS_Y,
    markerLength=MARKER_LENGTH,
    markerSeparation=MARKER_SEPARATION,
    dictionary=ARUCO_DICT)

## Create an image from the gridboard ##
gridboard_img = gridboard.draw(outSize=IM_OUT_SIZE, marginSize=MARGIN_SIZE, borderBits=1)

## Display and save gridboard image ##
cv2.imshow('Gridboard_jpg', gridboard_img)
img_save_pth = os.path.abspath(os.path.join(
    save_dir_path, "../data/arucoboard.jpg"))
cv2.imwrite(img_save_pth, gridboard_img)

## Exit on any key ##
cv2.waitKey(0)
cv2.destroyAllWindows()
