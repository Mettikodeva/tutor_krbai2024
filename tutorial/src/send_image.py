import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def send_image():
    rospy.init_node('send_image', anonymous=True)
    pub = rospy.Publisher('image', Image, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        rate.sleep()

if __name__ == '__main__':
    try:
        send_image()
    except rospy.ROSInterruptException:
        pass