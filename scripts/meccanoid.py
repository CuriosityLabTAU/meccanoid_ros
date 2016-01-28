#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import serial
import os


class meccanoid():
    ser = None

    def start_serial(self):
        port = '/dev/ttyACM0'
        os.system("sudo chmod 666 " + port)
        self.ser = serial.Serial(port, 9600)  # open serial port
        print(self.ser.name)         # check which port was really used

    def close_serial(self):
        self.ser.close()             # close port

    def talker(self):
        #pub = rospy.Publisher('chatter', String, queue_size=10)
        #rospy.init_node('talker', anonymous=True)
        pass

    def callback(self, data):
        print(data.data)
        self.ser.write(str.encode(data.data))     # write a string
        print("--- done ---")

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('meccanoid_listener')
        rospy.on_shutdown(self.close_serial)

        rospy.Subscriber("to_meccanoid", String, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    try:
        m = meccanoid()
        m.start_serial()
        #m.talker()
        m.listener()
    except rospy.ROSInterruptException:
        pass