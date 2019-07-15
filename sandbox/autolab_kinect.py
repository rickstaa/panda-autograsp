import perception

## Create kinect sensor object ##
camera = perception.Kinect2Sensor()
camera.start()
camera.color_intrinsics
print("test")