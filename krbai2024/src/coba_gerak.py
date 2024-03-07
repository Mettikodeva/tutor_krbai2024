import rospy
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import OverrideRCIn

if __name__ == "__main__":
    rospy.init_node("coba_gerak", log_level=rospy.DEBUG)
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
    arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    response = arm_service(True)
    rospy.loginfo(f"response:{response}")
    rospy.loginfo("Arming")
    if response.success:
        rospy.loginfo("Arming success")
        

    rospy.loginfo("Maju")
    msg = OverrideRCIn()

    start_time = rospy.Time().now()
    while rospy.Time().now() - start_time < rospy.Duration(5):
        msg.channels[4] = 1900
        pub.publish(msg)
    rospy.loginfo("Mundur")
    start_time = rospy.Time().now()
    while rospy.Time().now() - start_time < rospy.Duration(5):
        msg.channels[4] = 1300
        pub.publish(msg)
    
    