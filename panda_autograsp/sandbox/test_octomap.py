import rospy
from std_srvs.srv import Empty, Trigger
from std_msgs.msg import Empty
import sys

if __name__ == "__main__":

    ###############################################
    # Initialize octomap server services ##########
    ###############################################
    rospy.loginfo("Connecting octomap_server 'octomap_server/reset' service.")
    rospy.wait_for_service("octomap_server/reset")
    try:
        _moveit_octomap_reset_srv = rospy.ServiceProxy("octomap_server/reset", Empty)
        rospy.loginfo("Moveit 'octomap_server/reset' service found!")
    except rospy.ServiceException as e:
        rospy.logerr(
            "Moveit 'octomap_server/reset' service initialization failed: %s" % e
        )
        shutdown_msg = "Shutting down %s node because %s service connection failed." % (
            rospy.get_name(),
            _moveit_octomap_reset_srv.resolved_name,
        )
        rospy.logerr(shutdown_msg)
        sys.exit(0)

    print("kan")
