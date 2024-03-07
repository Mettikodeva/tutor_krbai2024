from tutorial.srv import activate, activateResponse
import rospy

status = False

def handle_activate(req):
    global status
    request = req.activate
    status = request
    rospy.loginfo(f"request from client: {request}")
    response = activateResponse(True)
    rospy.loginfo(f"response from server: {response}")    
    return response    

if __name__ == "__main__":
    rospy.init_node('activate_server', log_level=rospy.DEBUG)

    s = rospy.Service('activate', activate  , handle_activate)
    print("Ready to activate.")
    r = rospy.Rate(3)
    while not rospy.is_shutdown():
        if status:
            rospy.loginfo("saya aktif")
        else:
            rospy.logdebug("saya tidak aktif")
        r.sleep()
        