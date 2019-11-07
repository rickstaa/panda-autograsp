import rospy
from std_srvs.srv import Empty, Trigger
from std_msgs.msg import Empty
from panda_autograsp.srv import ResetOctomap
import sys

if __name__ == "__main__":

    ###############################################
    # Initialize octomap server services ##########
    ###############################################
    rospy.loginfo("Connecting octomap_server 'octomap_server/reset' service.")
    rospy.wait_for_service("reset_octomap")
    try:
        _moveit_octomap_reset_srv = rospy.ServiceProxy("reset_octomap", Empty)
        rospy.loginfo("Moveit 'reset_octomap' service found!")
    except rospy.ServiceException as e:
        rospy.logerr("Moveit 'reset_octomap' service initialization failed: %s" % e)
        shutdown_msg = "Shutting down %s node because %s service connection failed." % (
            rospy.get_name(),
            _moveit_octomap_reset_srv.resolved_name,
        )
        rospy.logerr(shutdown_msg)
        sys.exit(0)

    print("kan")
