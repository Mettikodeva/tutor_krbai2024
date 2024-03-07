import rospy
from tutorial.srv import addTwo, addTwoResponse

def handle_add_two(req):

    hasil = req.a + req.b
    response = addTwoResponse()
    response.sum = hasil
    print(f"Returning {req.a} + {req.b} = {hasil}")
    return response

if __name__ == "__main__":
    rospy.init_node("add_two_server")
    s = rospy.Service("add_two", addTwo, handle_add_two)
    rospy.spin()