import perception
import cv2
import matplotlib.pyplot as plt

kinect_camera = perception.Kinect2Sensor()
kinect_camera.start()

while True:
    # Get frames
    color_im, depth_im, _ = kinect_camera.frames()

    # Display images
    #fig, axes = plt.subplots(1, 2)
    #for ax, im in zip(axes, [color_im.data, depth_im.data]):
    #    ax.imshow(im)
    #    ax.axis('off')
    #plt.show()

    ## Create segmask ##
    bin_im = color_im.to_binary()

    # Create segmask

    # Create mask
    #bin_img = color_im.to_binary()
    #imgplot = plt.imshow(bin_img)
    cv2.imshow("color", color_im.data)
    cv2.imshow("depth", depth_im.data)
    key = cv2.waitKey(delay=1)
    if key == ord('q'):
        break

# Close camera connection
kinect_camera.stop()