from tutorial.srv import activate
import rospy

if __name__ == "__main__":
    rospy.init_node('activate_client')
    rospy.wait_for_service('activate')

    activate_client = rospy.ServiceProxy('activate', activate)
    response = activate_client(0)
    rospy.loginfo(f"response from server: {response}")
        

