import cv2
import cv2.aruco as aruco
import os

save_path = os.path.abspath(os.path.join(os.path.dirname(
    os.path.realpath(__file__)), "../data/charuco_board.jpg"))

# Create ChArUco board, which is a set of Aruco markers in a chessboard setting
sqWidth = 12 #number of squares width
sqHeight = 8 #number of squares height
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
gridboard = aruco.CharucoBoard_create(
        squaresX=sqWidth, 
        squaresY=sqHeight, 
        squareLength=0.035, 
        markerLength=0.0175, 
        dictionary=dictionary)

# Create an image from the gridboard
img = gridboard.draw(outSize=(700, 500))
cv2.imwrite(save_path, img)

# Display the image to us
cv2.imshow('Gridboard', img)
# Exit on any key
cv2.waitKey(0)
cv2.destroyAllWindows()


## 
## Board Parameters ##
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
SQUARE_LENGTH = 40
MARKER_LENGTH = 30
CHARUCO_BOARD = aruco.CharucoBoard_create(7, 5, SQUARE_LENGTH, MARKER_LENGTH, ARUCO_DICT)
aruco_parameters = aruco.DetectorParameters_create()
img = CHARUCO_BOARD.draw(outSize=(1400,988))
# fig = plt.figure()
# ax = fig.add_subplot(1,1,1)
# plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
cv2.imwrite("test_gridboard.jpg", img)
# ax.axis("off")
# plt.savefig("charuco_board.pdf")