#!/usr/bin/env python3

import rospy
import numpy as np
import smbus
from std_msgs.msg import Float32
import tf2_ros
import geometry_msgs.msg
import tf_conversions

#i2c Addresses 
MOTOR_ADDR = 0x18  #motor driver

#servo sub addresses 
SERVO_0 = 0x00 #servo 1 is 0x01 and so on and so fourth

#motor subaddresses 
MOTOR_A_PWM = 0x04
MOTOR_B_PWM = 0x05
MOTOR_A_DIR = 0x06
MOTOR_B_DIR = 0x07

#servo pulse width range
SERVO_MAX = 2500
SERVO_MIN = 500

class FreenoveThreeWheeledControl:
    """Handles all I2C interfacing from the raspberry pi to actuators. 
    """
    def __init__(self):
        #init
        rospy.loginfo("Starting freenove_three_wheel_control_interface.py")
        
        try:
            self.bus = smbus.SMBus(1)
            rospy.loginfo("freenove_three_wheel_control_interface initialized connection to driver board")
        except OSError: 
            rospy.logwarn("freenove_three_wheel_control_interface.py: failed to init driver board.... is it connected????")
            raise RuntimeError("Freenove board not found, ensure hat is connected by I2C to the raspberry pi")

        # self.test_servos_and_motors()

        #list of transforms we're going to publish       
        #frame, x,y,z,yaw
        # self.tf_info = [["fl_wheel", 0.09, 0.08, 0, 0], 
        #                 ["fr_wheel", 0.09, -0.08, 0, 0], 
        #                 ["camera_platform", 0.09, 0, 0.03, 0]] 
        self.tf_info = []

        self.add_tf("fl_wheel", 0.09, 0.08, 0)
        self.add_tf("fr_wheel", 0.09, -0.08, 0)
        self.add_tf("horizontal_camera_platform", 0.09, 0, 0.03)
        self.add_tf("vertical_camera_platform", 0, 0, 0.05, parent_frame="horizontal_camera_platform")


        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.update_tf()

        rospy.Subscriber("motor_drive", Float32, self.drive_callback)
        rospy.Subscriber("servo_1", Float32, self.servo0_callback)
        rospy.Subscriber("servo_2", Float32, self.servo1_callback)
        rospy.Subscriber("servo_3", Float32, self.servo2_callback)
        rospy.Subscriber("servo_4", Float32, self.servo3_callback)
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def add_tf(self, child_frame, x, y, z, roll=0, pitch=0, yaw=0, parent_frame="base_link"): 
        self.tf_info.append(
            { 
             "child_frame" : child_frame, 
             "parent_frame" : parent_frame, 
             "x" : x, 
             "y" : y, 
             "z" : z, 
             "roll" : roll, 
             "pitch" : pitch, 
             "yaw" : yaw
            }
        )

    def timer_callback(self, msg): 
        self.update_tf()
    
    def update_tf(self): 
        for i, a in enumerate(self.tf_info): 
            t = geometry_msgs.msg.TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = a["parent_frame"]
            t.child_frame_id = a["child_frame"]
            t.transform.translation.x = a["x"]
            t.transform.translation.y = a["y"]
            t.transform.translation.z = a["z"]

            #servos max out around 45 degrees
            q = tf_conversions.transformations.quaternion_from_euler(a["roll"], a["pitch"], a["yaw"])
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)

            # rospy.loginfo("Publishing tf %s" % t)
    
    def set_frame_rotation(self, frame, roll=0, pitch=0, yaw=0): 
        for i, a in enumerate(self.tf_info): 
            if a["child_frame"] == frame: 
                self.tf_info[i]["roll"] = roll
                self.tf_info[i]["pitch"] = pitch
                self.tf_info[i]["yaw"] = yaw
                self.update_tf()
                return 
        rospy.logwarn("freenove_three_wheel_control_interface.py: frameid %s is not tracked by this file" % frame)

    
    def servo0_callback(self, msg): 
        self.set_frame_rotation("fl_wheel", yaw = (msg.data * 45.0) * np.pi / 180.0)
        self.set_frame_rotation("fr_wheel", yaw = (msg.data * 45.0) * np.pi / 180.0)
        self.set_servo(0, msg.data)        

    def servo1_callback(self, msg): 
        self.set_frame_rotation("horizontal_camera_platform", yaw = (msg.data * 90) * np.pi / 180.0)
        self.set_servo(1, msg.data)        

    def servo2_callback(self, msg): 
        self.set_frame_rotation("vertical_camera_platform", pitch = - (msg.data * 90) * np.pi / 180.0)
        self.set_servo(2, msg.data)       

    def servo3_callback(self, msg): 
        self.set_servo(3, msg.data)         

    def drive_callback(self, msg): 
        self.drive_motors(msg.data)

    def test_servos_and_motors(self):
        rospy.loginfo("freenove_three_wheel_control_interface.py: Testing actuators")

        #Test code for motor and servo interface         
        for a in np.arange(-1,1, 0.05):
            self.set_servo(0, a)   
            rospy.sleep(0.3)
        self.set_servo(0,0)

        for a in np.arange(-1,1, 0.05): 
            self.drive_motors(a)
            rospy.sleep(0.3)

    def set_servo(self, servoNumber, position):
        """sets a servo on the freenove raspberry pi driver shield to a specified position 
        values must be supplied from -1 to 1 

        Args:
            servoNumber (int): servo to modify the position f
            position (float): value from -1 to 1 with 0 representing 90 degrees and -1 representing 0 degrees. 
        """
        try:
            if abs(position) > 1: 
                raise ValueError("freenove_three_wheel_control_interface.py: servo positions must be between -1 and 1 in set_servo()")
        

            value = nmap(position, -1, 1, SERVO_MIN, SERVO_MAX)
            self.write_register16(MOTOR_ADDR, int(servoNumber), int(value))
            rospy.logdebug("wrote servo " + str(servoNumber) + " with value of " + str(position))

        except OSError as e: 
            rospy.logwarn_throttle(1, "freenove_three_wheel_control_interface.py: error setting servos, is the shield disconnected?")


        #values from -1 to 1
    def drive_motors(self, speed):
        """Drives the motors on the freenove raspberry pi driver shield. Currently untested
        the freenove robot seems to require the 18650 battery power to be supplied in order for the 
        motors to work

        Args:
            speed (float): value from -1 to 1 indicating the speed the motors should be driven. negative values move the motors in reverse
        """

        value = nmap(speed, -1 , 1, -1000, 1000)

        self.write_register16(MOTOR_ADDR, MOTOR_A_DIR,  int(1 if (speed > 0) else 0))
        self.write_register16(MOTOR_ADDR, MOTOR_B_DIR,  int(1 if (speed > 0) else 0))
        self.write_register16(MOTOR_ADDR, MOTOR_A_PWM, int(abs(value)))
        self.write_register16(MOTOR_ADDR, MOTOR_B_PWM, int(abs(value)))


    def write_register16(self, deviceAddr, subAddr, data): 
        """writes to a 16 bit I2C register"""
        try: 
           value = int(data)
           self.bus.write_i2c_block_data(deviceAddr, subAddr, [value >> 8, value & 0xFF])
        except OSError: 
           rospy.logwarn("freenove_three_wheel_control_interface.py: failed to write 16-bit register " + str(hex(subAddr)) + " on device " + str(hex(deviceAddr))) 

    def write_register(self, deviceAddr, subAddr, data):
        """Writes to an 8-bit I2C register"""
        try: 
           self.bus.write_byte_data(deviceAddr, subAddr, data)
        except OSError: 
           rospy.logwarn("freenove_three_wheel_control_interface.py: failed to write 8-bit register " + str(hex(subAddr)) + " on device " + str(hex(deviceAddr)))

    def read_register(self, deviceAddr, subAddr, count): 
        """Reads a given number of 8-Bit I2C registers"""
        try:
           data = self.bus.read_i2c_block_data(deviceAddr, subAddr, count)
        except: 
           data = [-1] * count 
           rospy.logwarn("freenove_three_wheel_control_interface.py: failed to read 8-bit registers " + str(hex(subAddr)) + "-" + str(hex(subAddr + count)) + " on device " + str(hex(deviceAddr)) + " returning -1 for all values ")

        return data

    def close(self):
        """closes the I2C Connection gently (TODO)
        """
        self.drive_motors(0)
        for a in range(0,3): 
           self.set_servo(a, 0)
        rospy.loginfo("Closing I2C Communication")

def nmap(value, inLow, inHigh, outLow, outHigh):
    return (outHigh - outLow) * (value - inLow) / (inHigh - inLow) + outLow

# initialize node when script is called
if __name__ == '__main__':
    rospy.loginfo('starting freenove_three_wheel_control_interface.py')
    rospy.init_node('freenove_three_wheel_control_interface', log_level=rospy.INFO)

    try:
        node = FreenoveThreeWheeledControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logfatal('caught exception')
    
    #close the serial connection so we don't run into any issues
    node.close()

    rospy.loginfo('exiting')
