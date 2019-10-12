
## Import python packages ##
import math
import time

## Import ros packages ##
import rospy

## Import Tf2 packages ##
import tf_conversions
import tf2_ros
import tf2_geometry_msgs

## Import messages and services ##
import geometry_msgs.msg

## Initialize ros node ##
rospy.loginfo("Initializing panda_autograsp_server")
rospy.init_node('pose_test')

## Generate pose ##
tf2_buffer = tf2_ros.Buffer()
tf2_listener = tf2_ros.TransformListener(tf2_buffer)
time.sleep(1)

## Generate pose Publisher ##
pose_pub_start = rospy.Publisher("start/pose",
                                geometry_msgs.msg.PoseStamped,
                                queue_size=10)
pose_pub_trans = rospy.Publisher("trans/pose",
                                geometry_msgs.msg.PoseStamped,
                                queue_size=10)

## Generate pose ##
pose_p8 = geometry_msgs.msg.PoseStamped()
pose_p8.header.stamp = rospy.Time()
pose_p8.header.frame_id = "panda_link8"
pose_p8.pose.position.x = 0.5
pose_p8.pose.position.y = 0
pose_p8.pose.position.z = 0
pose_p8_quat = tf_conversions.transformations.quaternion_from_euler(math.radians(90), math.radians(0), math.radians(0))
pose_p8.pose.orientation.x = pose_p8_quat[0]
pose_p8.pose.orientation.y = pose_p8_quat[1]
pose_p8.pose.orientation.z = pose_p8_quat[2]
pose_p8.pose.orientation.w = pose_p8_quat[3]
pose_msg= geometry_msgs.msg.PoseStamped()

rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    ## Get transform between panda_link8 and gripper center ##
    trans_grip_center_p8 = tf2_buffer.lookup_transform("panda_link8", "panda_gripper_center", rospy.Time(0))
    pose_msg = tf2_geometry_msgs.do_transform_pose(pose_p8, trans_grip_center_p8)

    ## Publish pose ##
    pose_pub_start.publish(pose_p8)
    pose_pub_trans.publish(pose_msg)
    rate.sleep()
