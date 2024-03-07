import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('receive_image', anonymous=True)
    rospy.Subscriber("image", Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()