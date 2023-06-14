#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import math       # import math for math.pi constant

#from HW_DriverFiles import pic24_USB_Driver
from my_py_pkg import pic24_USB_Driver
#from my_py_pkg import vl53l8cx_tof_driver

#import sensor_msgs.point_cloud2 as pc2
import numpy as np
#import pcl
import ctypes
import struct

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from sensor_msgs.msg import Imu

#import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

import numpy as np
import os
import time

import rclpy #use ROS2 python library 
from rclpy.node import Node  #import Node class

#from machine_interfaces.msg import HardwareStatus   #Machine Interface package was accidentally deleted. 
from machine_interfaces.msg import ServoStatus


#Create a class that inherits from the standard ROS2 node class 
class Machine(Node):

    POWER_PIN = "23"
    PULSE_RANGE = [575, 2425]

    def __init__(self, filename):  #filename is the name passed for log data file. Not part of ROS
        
        #Include Class object attributes in this section

        ####
        ####  Class attributes 
        ####

        #super function is used to give access to methods and properties of a parent class
        #functions in python are outside of a class. Methods are inside of a class
        #call the super function. The node name is initialize here, "machine_Interface".
        super().__init__("machine_Interface")  
        self.counter_ = 0
        self.get_logger().info("1. In Machine Control Node.")
        print("1. In Machine Control Node.")
        #self.create_timer(0.5, self.timer_callback)

        self.filename = filename
        program_started = True

        self.present_time = 0
        self.previous_time = 0
        self.counterValue = 0;

        time.sleep(0.1)

        #Connect to the PIC24 board by instantatiating the PIC24 Class   
        self.pic24 = pic24_USB_Driver.Rover_HW_Driver()  

        print("Instantiated PIC24 class") 

        BoardFound = False
        connectCtr = 0
        # PIC24 and ROS setup
        while BoardFound == False:
            #ImuData = self.pic24.get_imu_values()
            BoardID = self.pic24.get_board()
            print("Board Name: ", BoardID) 
            if BoardID == 'PIC24INTER':

                connectCtr += 1
                print("Connection Counter: ", connectCtr) 
                print('')
                print("Hardware Info: ")
                print('Found the PIC24 Driver Board')
                print('')
                print("Board          : ", BoardID)

                BoardFound = True
                print("Board bool: ", BoardFound)
            else:
                connectCtr += 1
                print("Connection Counter: ", connectCtr) 
                BoardID = self.pic24.get_board()
                print("Connection Counter: ", connectCtr) 
                print('')
                print("Board bool: ", BoardFound)
                
                if connectCtr >= 10:
                    connectCtr = 0
                    print("PIC24 Board not Found.")
                    exit()

        time.sleep(1)

        #The following line request the user to press a keyboard key to start logging into a data file.
        #self.open_close_data_file(False)

        #Create a publisher to publish the servo location status.
        self.hw_status_publisher_ = self.create_publisher(ServoStatus, "servo_0_status", 1)  #Queue size of 10      
        self.timer_ = self.create_timer(0.1, self.publish_hw_status)   #1 second interval
        self.get_logger().info("Hardware/Servo Status Publisher has started.")

                #Create a publisher to publish the IMU data.
                #self.imu_data_publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
                #self.timer_ = self.create_timer(0.1, self.publish_imu_data)        #0.1 second interval
                #self.get_logger().info("IMU Data Publisher has started.")
      
    def callback_pcl_data(self, msg):
        self.get_logger().info("Received VL53LC8X PCL Data.")
        #pass

    # def callback_imu_data(self, msg):
    #     #imu_msg = Imu()    
    #     #self.get_logger().info("Imu Data.")
    #     self.get_logger().info(str(msg))

    def open_close_data_file(self, FileStatus):
        
        if FileStatus == False:
            #Open File to Save data
            print("Opened Data File.")
            self.file = open(self.filename, "a")
            self.started_saving_data = False
            print("started_saving_data value: ", self.started_saving_data)
            self.keyboard_input_value = ''
            print("keyboard_input_value: ", self.keyboard_input_value)             
        else:
            #close the file if file is open
            print("Closed Data File.")
            self.file.close() 

    def save_data_file(self, data_to_write):
        #Open File to Save data
        str1 = ""

        #print("Saving Data into log file.")                
        str1 =  str(data_to_write)[1 : -1]
        self.file.write(str1 + "\n") 
     
    def publish_hw_status(self):

        msg = ServoStatus()

        print("Retriving Servo Position Values.")
        Servo_Value = self.pic24.get_servo_motors()

        Servo0_Value = float(Servo_Value[0])
        #print("Servo 1 Pos float = ", Servo0_Value)  
        Servo1_Value = float(Servo_Value[1])
        #print("Servo 2 Pos float = ", Servo1_Value) 

#        try:
        #----------------------------------------------------------------
        #Covert ticks to degrees
        ServoValue_deg = round((0.0225*Servo0_Value), 4)      
        #print("Servo 1 Pos (deg) = ", str(ServoValue_deg))  

        #Covert degrees to radians
        ServoValue_rad  = round((ServoValue_deg * (np.pi / 180.0)), 6)    
        #print("Servo 1 Pos (rad) = ", str(ServoValue_rad))  

        #msg.servo_position = Servo0_Value
        msg.servo0_position_rad = ServoValue_rad 
        #----------------------------------------------------------------
        #----------------------------------------------------------------
        #Covert ticks to degrees
        ServoValue_deg = round((0.02557*Servo1_Value), 4)  
        ServoValue_deg = (ServoValue_deg - 30.1) + 270    #Add the offset angle  
        print("Servo 2 Pos (deg) = ", str(ServoValue_deg))  

        #Covert degrees to radians
        ServoValue_rad  = round((ServoValue_deg * (np.pi / 180.0)), 6)    
        print("Servo 2 Pos (rad) = ", str(ServoValue_rad))  

        #msg.servo_position = Servo0_Value
        msg.servo1_position_rad = ServoValue_rad 
        #----------------------------------------------------------------

        msg.debug_message = "This is the servo position value in radians"

        #self.get_logger().info(str(msg))
        self.hw_status_publisher_.publish(msg) 

        #self.present_time = time.time()
        #tick_value = self.present_time - self.previous_time
        #print("Tick Value: ", tick_value)
        #self.previous_time = self.present_time

        #self.counterValue += 1
        #print("Counter Value: ", self.counterValue)



#       except Exception as exception_error:
#            pass
#            #print("Error: " + str(exception_error))

    def publish_imu_data(self):

        try:
            #self.present_time = time.time()
            #tick_value = self.present_time - self.previous_time
            #print("Tick Value: ", tick_value)
            #self.previous_time = self.present_time

            #self.counterValue += 1
            #print("Counter Value: ", self.counterValue)

            #print("Publisher: Requesting IMU Data.")

            #------------------- LOG FILE CODE ---------------------------------------------
            #print("started_saving_data value: ", self.started_saving_data)
            # if self.started_saving_data == False:
                
            #     self.keyboard_input_value = input("Press 's' to start saving data. \n")

            #     if self.keyboard_input_value == 's':    
            #         self.started_saving_data = True
            #         print("")
            #         #Get user keyboard input    
                        
            # if self.started_saving_data == True:
            #     self.save_data_file(ImuData)
            #------------------- LOG FILE CODE ---------------------------------------------
            
            ImuData = self.pic24.get_imu_values()
            self.get_logger().info(str(ImuData))

            IMU_FRAME = None

            imu_msg = Imu()
            imu_msg.header.frame_id = "vcl53l8cx_sensor"
            #imu_msg.header.frame_id = "world"


            t = self.get_clock().now()
            imu_msg.header.stamp = t.to_msg()

            if ImuData[0] != 0:
                imu_msg.linear_acceleration.x = float(ImuData[0])*9.8
                imu_msg.linear_acceleration.y = float(ImuData[1])*9.8
                imu_msg.linear_acceleration.z = float(ImuData[2])*9.8

                imu_msg.angular_velocity.x = float(ImuData[3])*0.0174
                imu_msg.angular_velocity.y = float(ImuData[4])*0.0174
                imu_msg.angular_velocity.z = float(ImuData[5])*0.0174

            self.imu_data_publisher_.publish(imu_msg)
            print("Publish IMU Data")            

        except KeyboardInterrupt:
            #pass
            rclpy.shutdown()

    def point_cloud(self, points, parent_frame):
        """ Creates a point cloud message.
        Args:
            points: Nx3 array of xyz positions.
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        Code source:
            https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
        """
        
        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes()
        fields = [sensor_msgs.PointField(
            name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]
        header = std_msgs.Header(frame_id=parent_frame)

        return sensor_msgs.PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3),  # Every point consists of three float32s.
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )   

    def timer_callback(self):
        self.counter_ += 1
        #self.get_logger().info("In ROS2 Infinite loop " + str(self.counter_))

        # subscriber
        # rospy.Subscriber("motor/dps/left", Int16, lambda msg: self.g.set_motor_dps(self.ML, msg.data))
        # rospy.Subscriber("motor/dps/right", Int16, lambda msg: self.g.set_motor_dps(self.MR, msg.data))
        # rospy.Subscriber("motor/pwm/left", Int8, lambda msg: self.g.set_motor_power(self.ML, msg.data))
        # rospy.Subscriber("motor/pwm/right", Int8, lambda msg: self.g.set_motor_power(self.MR, msg.data))
        # rospy.Subscriber("motor/position/left", Int16, lambda msg: self.g.set_motor_position(self.ML, msg.data))
        # rospy.Subscriber("motor/position/right", Int16, lambda msg: self.g.set_motor_position(self.MR, msg.data))
        # rospy.Subscriber("servo/pulse_width/1", Int16, lambda msg: self.g.set_servo(self.S1, msg.data))
        # rospy.Subscriber("servo/pulse_width/2", Int16, lambda msg: self.g.set_servo(self.S2, msg.data))
        # rospy.Subscriber("servo/position/1", Float64, lambda msg: self.set_servo_angle(self.S1, msg.data))
        # rospy.Subscriber("servo/position/2", Float64, lambda msg: self.set_servo_angle(self.S2, msg.data))
        # rospy.Subscriber("cmd_vel", Twist, self.on_twist)

        # rospy.Subscriber("led/blinker/left", UInt8, lambda msg: self.g.set_led(self.BL, msg.data))
        # rospy.Subscriber("led/blinker/right", UInt8, lambda msg: self.g.set_led(self.BR, msg.data))
        # rospy.Subscriber("led/eye/left", ColorRGBA, lambda c: self.g.set_led(self.EL, int(c.r*255), int(c.g*255), int(c.b*255)))
        # rospy.Subscriber("led/eye/right", ColorRGBA, lambda c: self.g.set_led(self.ER, int(c.r*255), int(c.g*255), int(c.b*255)))
        # rospy.Subscriber("led/wifi", ColorRGBA, lambda c: self.g.set_led(self.EW, int(c.r * 255), int(c.g * 255), int(c.b * 255)))

        # publisher
        # self.pub_enc_l = rospy.Publisher('motor/encoder/left', Float64, queue_size=10)
        # self.pub_enc_r = rospy.Publisher('motor/encoder/right', Float64, queue_size=10)
        #self.pub_battery = rospy.Publisher('battery_voltage', Float64, queue_size=10)
        #self.pub_imu_values = rospy.Publisher('IMU_Values', IMU_Values, queue_size=10)
        #self.pub_heartbeat = rospy.Publisher('heartbeat', String, queue_size=10)
        
        # self.pub_motor_status = rospy.Publisher('motor/status', MotorStatusLR, queue_size=10)
        # self.pub_odometry = rospy.Publisher("odom", Odometry, queue_size=10)
        # self.pub_joints = rospy.Publisher("joint_state", JointState, queue_size=10)

        # # services
        # self.srv_reset = rospy.Service('reset', Trigger, self.reset)
        # self.srv_spi = rospy.Service('spi', SPI, lambda req: SPIResponse(data_in=self.g.spi_transfer_array(req.data_out)))
        # self.srv_pwr_on = rospy.Service('power/on', Trigger, self.power_on)
        # self.srv_pwr_off = rospy.Service('power/off', Trigger, self.power_off)

def main(args=None):

    try:
        rclpy.init(args=args)  #iniitalize ROS2 communication
        print("Start of machineControlNode.")
        print("")
        # The name of the node is not the name of the file.
        node = Machine(filename = "accel_data.txt")   #Instantiate the node

        rclpy.spin(node)

        print("")
        print("Exiting Application from main.")
        
        node.destroy_node()

        rclpy.shutdown()    #shutdown ROS2 communication by pressing ctrl + c

    except KeyboardInterrupt:
        node.destroy_node()
        print("")

        #node.open_close_data_file(True)

        node.pic24.cleanup_digital_io()
        print("Cleaned Digital I/Os.")

        print("Exiting Application")

        rclpy.shutdown()    #shutdown ROS2 communication by pressing ctrl + c

        print("Exiting Program")

if __name__ == "__main__":
    main()
