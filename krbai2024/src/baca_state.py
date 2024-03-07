import rospy
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
#       Subscribe("topic", tipe_data, fungsi_callback)

def callback_baca_state(msg):
    # rospy.logdebug(f"state: {msg}")
    rospy.logdebug(f"connected: {msg.connected}")

def callback_baca_imu(msg):
    rospy.logdebug(f"imu: {msg}")
    rospy.logdebug(f"orientation: {msg.orientation}")
    rospy.logdebug(f"angular_velocity: {msg.angular_velocity}")
    rospy.logdebug(f"linear_acceleration: {msg.linear_acceleration}")


if __name__ == "__main__":
    rospy.init_node('baca_state', log_level=rospy.DEBUG)
    # sub  = rospy.Subscriber('/mavros/state', State, callback_baca_state)
    sub_imu = rospy.Subscriber('/mavros/imu/data', Imu, callback_baca_imu)
    rospy.spin()

