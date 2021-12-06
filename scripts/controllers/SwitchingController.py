#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class SwitchingController:
    def __init__(self):
        rospy.init_node('switching_controller')
        self.kbd_flg = False
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub1 = rospy.Subscriber('/kbd_cmd_vel', Twist, self.process_kybd_vel)
        self.sub2 = rospy.Subscriber('/nav_cmd_vel', Twist, self.process_controller_vel)
        self.sub3 = rospy.Subscriber('/kbd_flg', Bool, self.process_kybd_flag)

    def process_kybd_flag(self, msg):
        self.kbd_flg = msg.data
        print('Switching controllers. msg: ', msg.data)
        
    def process_kybd_vel(self, msg):
        if self.kbd_flg:
            print('Publishing kybd vel: ', msg)
            self.pub.publish(msg)

    def process_controller_vel(self, msg):
        if not(self.kbd_flg):
            print('Publishing controller vel: ', msg)
            self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    gr = SwitchingController()
    gr.run()
