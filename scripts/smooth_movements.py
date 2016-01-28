#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy
from std_msgs.msg import String


class smooth_movements():
    # subscribes to "smooth_movements" topic
    # message: "dur, pos [16 comma-separated values]"
    #   dur = duration in seconds for the movements
    #   pos = position of the motors at the end of the movement
    #       no_motion string - for not moving the motor

    current_position = []
    next_position = []
    is_running = False
    robot_platform = "to_meccanoid"
    pub = None
    delay = 0.2 # basic delay for meccanoid
    no_motion = "-1"

    def __init__(self):
        rospy.init_node('smooth_movements')
        self.current_position =

    def to_string(self, pos):
        s = ""
        for p in pos:
            if type(p) is int:
                s += str(p)
            else:
                s += self.no_motion
            s += ','
        s = s[:-2]
        return s

    def listener(self):
        rospy.Subscriber("next_position", String, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def callback(self, data):
        while self.is_running:
            pass
        # parsing the data, including no_motion
        input = []
        for p in str.split(data.data, ','):
            if p != self.no_motion:
                input.append(None)
            else:
                input.append(int(p))

        dur = input[0]  # duration
        pos = input[1:] # next position

        # got over motors

        # if first input, set it to current
        if self.current_position == []:
            self.current_position = pos
            self.publish(self.to_string(pos))
        else:
            # set smooth trajectory
            self.next_position = pos
            self.publish_trajectory(dur)
        self.is_running = False

    def set_trajectory(self, dur):
        num_steps = dur / self.delay
        dp = []
        for k in range(0, len(self.current_position)):
            if k < len(self.next_position):
                if self.current_position

        for t in range(0, num_steps):


    def publish(self, data):
        self.is_running = True
        self.pub.publish(data)

    def talker(self):
        self.pub = rospy.Publisher(self.robot_platform, String, queue_size=10)


if __name__ == '__main__':
    try:
        s = smooth_movements()
        s.talker()
        s.listener()
    except rospy.ROSInterruptException:
        pass
