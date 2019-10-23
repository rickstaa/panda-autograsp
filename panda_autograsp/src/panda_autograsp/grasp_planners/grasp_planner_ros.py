# Make script both python2 and python3 compatible
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

try:
    input = raw_input
except NameError:
    pass

# Main python packages
import math
import time
import matplotlib.pyplot as plt
import numpy as np

from perception import CameraIntrinsics, ColorImage, DepthImage, BinaryImage, RgbdImage
from visualization import Visualizer2D as vis
from gqcnn.utils import NoValidGraspsException
from gqcnn.grasping import Grasp2D, SuctionPoint2D, RgbdImageState

# ROS python packages
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image
import rospy

# ROS messages and services
from tf2_geometry_msgs import PoseStamped  # Needed because we use tf2
from std_msgs.msg import Header
from panda_autograsp.msg import GQCNNGrasp

# Set right matplotlib backend
# Needed in order to show images inside imported modules
plt.switch_backend("TkAgg")


#################################################
# Grasp planner class ###########################
#################################################
class GraspPlannerROS(object):
    """Class used to compute the grasp pose out of the RGB-D data.

    Attributes
    -----------

        cfg : :py:obj:`YamlConfig`
            The grasp ``yaml`` configuration file.
        grasping_policy : :py:obj:`CrossEntropyRobustGraspingPolicy`
            The grasp policy.
        min_width: :py:obj:`int`
            The minimum allowed image width.
        min_height : :py:obj:`int`
            The minimum allowed image height.
    """

    def __init__(self, cfg, cv_bridge, grasping_policy, grasp_pose_publisher):
        """

        Parameters
        ----------
        cfg : dict
            Dictionary of configuration parameters.
        cv_bridge: :py:obj:`CvBridge`
            ROS `CvBridge`.
        grasping_policy: :py:obj:`GraspingPolicy`
            Grasping policy to use.
        grasp_pose_publisher: :py:obj:`Publisher`
        """
        self.cfg = cfg
        self._cv_bridge = cv_bridge
        self.grasping_policy = grasping_policy
        self._grasp_pose_publisher = grasp_pose_publisher

        # Set minimum input dimensions.
        pad = max(
            math.ceil(
                np.sqrt(2) * (float(self.cfg["policy"]["metric"]["crop_width"]) / 2)
            ),
            math.ceil(
                np.sqrt(2) * (float(self.cfg["policy"]["metric"]["crop_height"]) / 2)
            ),
        )
        self.min_width = 2 * pad + self.cfg["policy"]["metric"]["crop_width"]
        self.min_height = 2 * pad + self.cfg["policy"]["metric"]["crop_height"]

        # Initialize image publisher
        self._image_pub = rospy.Publisher("grasp_image", Image, queue_size=10)

    def read_images(self, req):
        """Retrieves the input images from a ROS service request.

        Parameters
        ---------
        req: :py:obj:`ROS ServiceRequest`
            ROS ServiceRequest for grasp planner service.
        """

        # Get the raw depth and color images as ROS `Image` objects
        raw_color = req.color_image
        raw_depth = req.depth_image

        # Get the raw camera info as ROS `CameraInfo`
        raw_camera_info = req.camera_info

        # Wrap the camera info in a BerkeleyAutomation/perception
        # `CameraIntrinsics` object.
        camera_intr = CameraIntrinsics(
            raw_camera_info.header.frame_id,
            raw_camera_info.K[0],
            raw_camera_info.K[4],
            raw_camera_info.K[2],
            raw_camera_info.K[5],
            raw_camera_info.K[1],
            raw_camera_info.height,
            raw_camera_info.width,
        )

        # Create wrapped BerkeleyAutomation/perception RGB and depth images
        try:
            color_im = ColorImage(
                self._cv_bridge.imgmsg_to_cv2(raw_color, "rgb8"),
                frame=camera_intr.frame,
            )
            depth_im = DepthImage(
                self._cv_bridge.imgmsg_to_cv2(
                    raw_depth, desired_encoding="passthrough"
                ),
                frame=camera_intr.frame,
            )
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)

        # Check image sizes
        if color_im.height != depth_im.height or color_im.width != depth_im.width:
            msg = (
                "Color image and depth image must be the same shape! Color"
                " is %d x %d but depth is %d x %d"
            ) % (color_im.height, color_im.width, depth_im.height, depth_im.width)
            rospy.logerr(msg)
            raise rospy.ServiceException(msg)
        if color_im.height < self.min_height or color_im.width < self.min_width:
            msg = (
                "Color image is too small! Must be at least %d x %d"
                " resolution but the requested image is only %d x %d"
            ) % (self.min_height, self.min_width, color_im.height, color_im.width)
            rospy.logerr(msg)
            raise rospy.ServiceException(msg)

        # Return sensor data
        return color_im, depth_im, camera_intr

    def plan_grasp(self, req):
        """Grasp planner request handler.

        Parameters
        ---------
        req: :py:obj:`ROS ServiceRequest`
            ROS `ServiceRequest` for grasp planner service.
        """

        # Retrieve sensor data from the request
        color_im, depth_im, camera_intr = self.read_images(req)

        # Call main grasp computation function
        return self._plan_grasp(color_im, depth_im, camera_intr)

    def plan_grasp_bb(self, req):
        """Grasp planner request handler.

        Parameters
        ---------
        req: :py:obj:`ROS ServiceRequest`
            `ROS ServiceRequest` for grasp planner service.
        """

        # Retrieve sensor data from the request
        color_im, depth_im, camera_intr = self.read_images(req)

        # Call main grasp computation function
        return self._plan_grasp(
            color_im, depth_im, camera_intr, bounding_box=req.bounding_box
        )

    def plan_grasp_segmask(self, req):
        """Grasp planner request handler.

        Parameters
        ---------
        req: :py:obj:`ROS ServiceRequest`
            ROS `ServiceRequest` for grasp planner service.
        """
        color_im, depth_im, camera_intr = self.read_images(req)
        raw_segmask = req.segmask

        # Create segmask
        try:
            segmask = BinaryImage(
                self._cv_bridge.imgmsg_to_cv2(
                    raw_segmask, desired_encoding="passthrough"
                ),
                frame=camera_intr.frame,
            )
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)

        # Validate image size
        if color_im.height != segmask.height or color_im.width != segmask.width:
            msg = (
                "Images and segmask must be the same shape! Color image is"
                " %d x %d but segmask is %d x %d"
            ) % (color_im.height, color_im.width, segmask.height, segmask.width)
            rospy.logerr(msg)
            raise rospy.ServiceException(msg)

        # Call main grasp computation function
        return self._plan_grasp(color_im, depth_im, camera_intr, segmask=segmask)

    def _plan_grasp(
        self, color_im, depth_im, camera_intr, bounding_box=None, segmask=None
    ):
        """Grasp planner request handler.

        Parameters
        ---------
        req: :py:obj:`ROS ServiceRequest`
            ROS `ServiceRequest` for grasp planner service.
        """
        rospy.loginfo("Planning Grasp")

        # Inpaint images
        color_im = color_im.inpaint(rescale_factor=self.cfg["inpaint_rescale_factor"])
        depth_im = depth_im.inpaint(rescale_factor=self.cfg["inpaint_rescale_factor"])

        # Init segmask
        if segmask is None:
            segmask = BinaryImage(
                255 * np.ones(depth_im.shape).astype(np.uint8), frame=color_im.frame
            )

        # Visualize
        if self.cfg["vis"]["figs"]["color_image"]:
            vis.imshow(color_im)
            vis.show()
        if self.cfg["vis"]["figs"]["depth_image"]:
            vis.imshow(depth_im)
            vis.show()
        if self.cfg["vis"]["figs"]["segmask"] and segmask is not None:
            vis.imshow(segmask)
            vis.show()

        # Aggregate color and depth images into a single
        # BerkeleyAutomation/perception `RgbdImage`.
        rgbd_im = RgbdImage.from_color_and_depth(color_im, depth_im)

        # Mask bounding box.
        if bounding_box is not None:

            # Calc bb parameters.
            min_x = bounding_box.minX
            min_y = bounding_box.minY
            max_x = bounding_box.maxX
            max_y = bounding_box.maxY

            # Contain box to image->don't let it exceed image height/width
            # bounds.
            if min_x < 0:
                min_x = 0
            if min_y < 0:
                min_y = 0
            if max_x > rgbd_im.width:
                max_x = rgbd_im.width
            if max_y > rgbd_im.height:
                max_y = rgbd_im.height

            # Mask.
            bb_segmask_arr = np.zeros([rgbd_im.height, rgbd_im.width])
            bb_segmask_arr[min_y:max_y, min_x:max_x] = 255
            bb_segmask = BinaryImage(bb_segmask_arr.astype(np.uint8), segmask.frame)
            segmask = segmask.mask_binary(bb_segmask)

        # Visualize.
        if self.cfg["vis"]["figs"]["rgbd_state"]:
            masked_rgbd_im = rgbd_im.mask_binary(segmask)
            vis.figure()
            vis.subplot(1, 2, 1)
            vis.imshow(masked_rgbd_im.color)
            vis.subplot(1, 2, 2)
            vis.imshow(masked_rgbd_im.depth)
            vis.show()

        # Create an `RgbdImageState` with the cropped `RgbdImage` and
        # `CameraIntrinsics`.
        rgbd_state = RgbdImageState(rgbd_im, camera_intr, segmask=segmask)

        # Execute policy.
        try:
            return self.execute_policy(
                rgbd_state,
                self.grasping_policy,
                self._grasp_pose_publisher,
                camera_intr.frame,
            )
        except NoValidGraspsException:
            rospy.logerr(
                (
                    "While executing policy found no valid grasps from sampled"
                    " antipodal point pairs. Aborting Policy!"
                )
            )
            raise rospy.ServiceException(
                (
                    "While executing policy found no valid grasps from sampled"
                    " antipodal point pairs. Aborting Policy!"
                )
            )

    def execute_policy(
        self, rgbd_image_state, grasping_policy, grasp_pose_publisher, pose_frame
    ):
        """Executes a grasping policy on an `RgbdImageState`.

        Parameters
        ----------
        rgbd_image_state: :py:obj:`RgbdImageState`
            `RgbdImageState` from BerkeleyAutomation/perception to encapsulate
            depth and color image along with camera intrinsics.
        grasping_policy: :py:obj:`GraspingPolicy`
            Grasping policy to use.
        grasp_pose_publisher: :py:obj:`Publisher`
            ROS publisher to publish pose of planned grasp for visualization.
        pose_frame: :py:obj:`str`
            Frame of reference to publish pose in.
        """
        # Execute the policy"s action.
        grasp_planning_start_time = time.time()
        grasp = grasping_policy(rgbd_image_state)

        # Create `GQCNNGrasp` return msg and populate it.
        gqcnn_grasp = GQCNNGrasp()
        gqcnn_grasp.q_value = grasp.q_value
        gqcnn_grasp.pose = grasp.grasp.pose().pose_msg
        if isinstance(grasp.grasp, Grasp2D):
            gqcnn_grasp.grasp_type = GQCNNGrasp.PARALLEL_JAW
        elif isinstance(grasp.grasp, SuctionPoint2D):
            gqcnn_grasp.grasp_type = GQCNNGrasp.SUCTION
        else:
            rospy.logerr("Grasp type not supported!")
            raise rospy.ServiceException("Grasp type not supported!")

        # Store grasp representation in image space.
        gqcnn_grasp.center_px[0] = grasp.grasp.center[0]
        gqcnn_grasp.center_px[1] = grasp.grasp.center[1]
        gqcnn_grasp.angle = grasp.grasp.angle
        gqcnn_grasp.depth = grasp.grasp.depth
        gqcnn_grasp.gripper_width = grasp.grasp.width
        gqcnn_grasp.thumbnail = grasp.image.rosmsg

        # Create and publish the pose alone for easy visualization of grasp
        # pose in Rviz.
        pose_stamped = PoseStamped()
        pose_stamped.pose = grasp.grasp.pose().pose_msg
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = pose_frame
        pose_stamped.header = header
        grasp_pose_publisher.publish(pose_stamped)

        # Return `GQCNNGrasp` msg.
        rospy.loginfo(
            "Total grasp planning time: "
            + str(time.time() - grasp_planning_start_time)
            + " secs."
        )

        # Visualize result
        fig = vis.figure(size=(8, 8), dpi=100)
        vis.imshow(
            rgbd_image_state.rgbd_im.color,
            vmin=self.cfg["policy"]["vis"]["vmin"],
            vmax=self.cfg["policy"]["vis"]["vmax"],
        )
        vis.grasp(grasp.grasp, scale=2.5, show_center=False, show_axis=True)
        vis.title(
            "Planned grasp at depth {0:.3f}m with Q={1:.3f}".format(
                grasp.grasp.depth, grasp.q_value
            ),
            fontsize=20,
        )
        fig.tight_layout()
        if self.cfg["vis"]["figs"]["final_grasp"]:
            vis.show()
        else:
            fig.canvas.draw()

        # Convert grasp image to ROS IMAGE message
        grasp_image = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep="")
        grasp_image = grasp_image.reshape(fig.canvas.get_width_height()[::-1] + (3,))[
            ..., ::-1
        ].copy()

        # Publish final grasp image
        try:
            self._image_pub.publish(self._cv_bridge.cv2_to_imgmsg(grasp_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

        # Return grasp
        return gqcnn_grasp

