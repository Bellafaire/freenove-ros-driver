#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class VelCmdController:
    """Simple node to map the /cmd_vel Twist message to the car inputs 
    """
    def __init__(self):
        rospy.loginfo("Starting vel_cmd_controller.py")

        #publishers to the car inputs
        self.motor_pub = rospy.Publisher("motor_drive", Float32, queue_size=10)
        self.steering_pub = rospy.Publisher("servo_1", Float32, queue_size=10)

        #subscriber to Twist message
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback)
    
    def twist_callback(self, msg): 
        motor = msg.linear.x
        turn = msg.angular.z

        motor_msg = Float32()
        motor_msg.data = motor
        self.motor_pub.publish(motor_msg)

        turn_msg = Float32()
        turn_msg.data = turn
        self.steering_pub.publish(turn_msg)

# initialize node when script is called
if __name__ == '__main__':
    rospy.loginfo('starting vel_cmd_controller.py')
    rospy.init_node('vel_cmd_controller', log_level=rospy.INFO)

    try:
        node = VelCmdController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logfatal('caught exception')
    
    rospy.loginfo('exiting')
