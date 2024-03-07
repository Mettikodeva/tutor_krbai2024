import pymavlink
from pymavlink import mavutil
import time

# simple test to move ardusub vehicle forward


# connect to the vehicle
master = mavutil.mavlink_connection('udpin:127.0.0.1:14550')


# Make sure the connection is valid
master.wait_heartbeat()

def wait_conn():
    """
    Sends a ping to stabilish the UDP communication and awaits for a response
    """
    msg = None
    while not msg:
        master.mav.ping_send(
            int(time.time() * 1e6), # Unix time in microseconds
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
        msg = master.recv_match()
        time.sleep(0.5)

def arm():
    
    # Arm
    # master.arducopter_arm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')

def rc_override(roll:int, pitch:int, throttle:int, yaw:int):
    """
    roll, pitch, throttle, yaw : -1000 to 1000
    """
    master.mav.rc_channels_override_send(
        master.target_system, # target_system
        master.target_component, # target_component
        roll, # chan1_raw
        pitch, # chan2_raw
        throttle, # chan3_raw
        yaw, # chan4_raw
        0, # chan5_raw
        0, # chan6_raw
        0, # chan7_raw
        0, # chan8_raw
        0  # chan9_raw
    )
def manual_control(x,y,z,r):
    master.mav.manual_control_send(
    master.target_system,
    x,y,z,r,
    0)

def disarm():
    # Disarm
    # master.arducopter_disarm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)

    # wait until disarming confirmed
    master.motors_disarmed_wait()

def set_servo(servo:int, value:int):
    """
    
    value : 1000-2000
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo, # servo number
        value, # servo value
        0, 0, 0, 0, 0
    )

if __name__ == "__main__":
    wait_conn()
    arm()
    while True:
        manual_control(0,500,0,0)
