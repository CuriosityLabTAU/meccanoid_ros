#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import time


class meccaoind_test():

    def talker(self):
        pub = rospy.Publisher('to_meccanoid', String, queue_size=10)
        rospy.init_node('meccanoid_test')

        for p in range(20,200,5):
            time.sleep(0.2)
            pub.publish("0," + str(p) + ",50,0,25,0,50,0,25,0,50,0,25,0,50,0,25")
#        time.sleep(0.2)
#        pub.publish("100,0,5,5,100,0,5,5,100,0,5,5,100,0,5,5")



if __name__ == '__main__':
    try:
        m = meccaoind_test()
        m.talker()
    except rospy.ROSInterruptException:
        pass
