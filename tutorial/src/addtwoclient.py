import rospy
from tutorial.srv import addTwo

if __name__ == "__main__":
    rospy.init_node("add_two_client")
    
    while not rospy.is_shutdown():
        a = rospy.get_param("a")
        b = rospy.get_param("b")
        service_client = rospy.ServiceProxy("add_two", addTwo)
        response = service_client(a,b)
        print(f"a = {a} + b = {b}")
        print(f"result = {response.sum}")

        rospy.sleep(3)