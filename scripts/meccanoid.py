#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import serial
import os
import time
import io


class meccanoid():
    ser = None
    port = ""

    def start_serial(self):
        self.port = '/dev/ttyACM0'
        print("Waiting for meccanoid to be connected via arduino ...")
        while not os.path.exists(self.port):
            pass
        os.system("sudo chmod 666 " + self.port)
        time.sleep(1)

        self.ser = serial.Serial(self.port, 9600)  # open serial port
        time.sleep(1)
        print("Arduino connected: ", self.ser.name)# check which port was really used

        rospy.init_node('meccanoid_listener')
        rospy.on_shutdown(self.close_serial)

    def close_serial(self):
        self.ser.close()             # close port

    def talker(self):
        pub = rospy.Publisher('from_meccanoid', String, queue_size=10)
        # rospy.spin()
        while True:
            data = self.ser.readline()
            pub.publish(data)
            #print(data)

    def callback(self, data):
        self.ser = serial.Serial(self.port, 9600)  # open serial port
        # num = str.split(data.data, ',')
        # check_sum = 0
        # for i in num:
        #     check_sum += int(i)
        # msg = '400,' + data.data + ',' + str(check_sum)
        # print(msg)
        # # ==============
        msg = data.data;
        self.ser.write(str.encode(msg))     # write a string
        #time.sleep(0.5)

    def listener(self):
        rospy.Subscriber("to_meccanoid", String, self.callback)


if __name__ == '__main__':
    try:
        m = meccanoid()
        m.start_serial()
        m.listener()
        m.talker()
    except rospy.ROSInterruptException:
        pass