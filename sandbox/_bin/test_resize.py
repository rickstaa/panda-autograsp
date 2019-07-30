import perception
import cv2
import matplotlib.pyplot as plt
from visualization import Visualizer2D as vis

## Create camera object ##
kinect_camera = perception.Kinect2Sensor()
kinect_camera.start()

## Get frames ##
color_im, depth_im, _ = kinect_camera.frames()

## Visualize resized frame
vis.figure()
vis.imshow(color_im)
vis.show()

## Resize color frame ##
color_im_small = color_im.resize(0.5)

## Visualize resized frame
vis.figure()
vis.imshow(color_im_small.data)
vis.show()

## Use the impaint function
color_im_inpaint = color_im.inpaint(
    rescale_factor=0.5)

## Visualize resized frame
vis.figure()
vis.imshow(color_im_inpaint)
vis.show()

# Close camera connection
kinect_camera.stop()