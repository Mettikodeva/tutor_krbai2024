import rospy
import random
from tutorial.msg import add



if __name__ == "__main__":
    rospy.init_node("kirim")
    pub1 = rospy.Publisher('add', add, queue_size=10)
    r = rospy.Rate(1)
    
    while not rospy.is_shutdown():

        data = add()
        data.data = f"halo"
        data.a = random.randint(1,10)
        data.b = random.randint(1,10)
        pub1.publish(data)

        # ----
        # bentuk lain
        # pub1.publish(add("halo",random.randint(1,10),random.randint(1,10)))
        # ----

        r.sleep()
