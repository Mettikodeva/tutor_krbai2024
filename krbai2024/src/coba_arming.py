import rospy
from mavros_msgs.srv import CommandBool

if __name__ == "__main__":
    rospy.init_node('coba_arming', log_level=rospy.DEBUG)
    rospy.wait_for_service('/mavros/cmd/arming')
    arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    response = arming_srv(True)
    rospy.logdebug(f"response:{response}")
    rospy.loginfo("Arming")
    # rospy.spin()