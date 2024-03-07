
import rospy
from tutorial.msg import add


def callback(data: add):
    hasil = data.a + data.b
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    rospy.loginfo(f"Hasilnya adalah {data.a + data.b}")
    rospy.loginfo(f"{data.a} + {data.b} = {hasil}")


def listener():
    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber(<topic>, <datatypes>, <function callback>)
    rospy.Subscriber('add', add, callback)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        r.sleep()

if __name__ == '__main__':
    
    listener()
