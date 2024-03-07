import rospy
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import SetMode,SetModeRequest, CommandBool, CommandBoolRequest
from sensor_msgs.msg import Imu 
from geometry_msgs.msg import Quaternion
from math import atan2, asin

class AUV:
    STABILIZE = "STABILIZE"
    GUIDED = "GUIDED"
    AUTO = "AUTO"
    LOITER = "LOITER"
    RTL = "RTL"
    DEPTH_HOLD = "ALT_HOLD"
    MANUAL = "MANUAL"

    def __init__(self):
        self.state = State()
        self.connected = False
        self.armed = False
        self.mode = self.STABILIZE
        # Set state
        state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.current_state = State()

        # self.manual_pub = rospy.Publisher("/mavros/manual_control/send", ManualControl, queue_size=10)
        self.rc_override_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)



        self.imu_sub = rospy.Subscriber("/mavros/imu/data",Imu, self.callback_imu)
        
    def quat2euler(self, data:Quaternion):
        x = data.x
        y = data.y
        z = data.z
        w = data.w
        roll = atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        pitch = asin(2*(w*y - z*x))
        yaw = atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        return roll, pitch, yaw
    
    def callback_imu(self, msg):
        rospy.logdebug_throttle(1,f"IMU DATA:{msg}")
        rospy.loginfo_throttle(0.1,self.quat2euler(msg.orientation))



    
    def rc_override(self, pitch=0,roll=0,throttle=0,yaw=0,forward=0,lateral=0):
        # 1 pitch
        # 2 roll
        # 3 throttle
        # 4 yaw
        # 5 forward
        # 6 lateral

        ch = OverrideRCIn()
        print(f"len : {len(ch.channels)}")
        channels = [pitch,roll,throttle,yaw,forward,lateral,0,0]
        for i in range(len(ch.channels)):

            try:
                ch.channels[i] = channels[i]
            except:
                ch.channels[i] = 0

        self.rc_override_pub.publish(ch)
    def callback_attitude(self, msg):
        rospy.logdebug_throttle(1,f"ATTITUDE DATA:{msg}")


    def state_cb(self, msg):
        """
        A function for state's subscriber callback
        Will set self.current_state to the message received
        """
        self.current_state = msg

    def set_mode(self, mode:str = "GUIDED"):
        """
        A function to set mode

        Args:
            mode (str): mode to set. Default to GUIDED
        """

        # Get client
        rospy.wait_for_service("/mavros/set_mode")
        client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # Set mode
        client(SetModeRequest(0, mode))

        # Check if mode is set
        if self.current_state.mode == mode:
            # Print success message
            rospy.loginfo(f"Mode is set to {mode}")
        else:
            # Print failed message
            rospy.loginfo(f"Failed to set mode to {mode}")
            rospy.loginfo(f"Current mode is {self.current_state.mode}")

    def arm(self, status: bool = True):
        """
        A function to arm or disarm the drone

        Args:
            status (bool): True to arm, False to disarm
        """
        # Get client
        rospy.wait_for_service("/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        
        # Arm
        while not rospy.is_shutdown() and not self.current_state.armed:
            arming_client(CommandBoolRequest(status))
        else:
            if status == True:
                rospy.loginfo("Drone is armed and ready to fly")
            else:
                rospy.loginfo("Drone is disarmed")
        
    def move_manual(self):

        pass

    def teleop(self):
        pass
if __name__ == "__main__":
    rospy.init_node("krbai2024_main", log_level=rospy.INFO)
    vehicle = AUV()
    vehicle.set_mode("STABILIZE")
    vehicle.arm()
    rospy.logwarn("MAJU")
    vehicle.rc_override(forward=1600)
    rospy.sleep(10)
    rospy.logwarn("MUNDUR")
    vehicle.rc_override(forward=1400)
    rospy.sleep(10)
    vehicle.arm(status=False)
    # vehicle.set_mode("GUIDED")
    # vehicle.arm()
    # rospy.sleep(5)
    # vehicle.arm(status=False)