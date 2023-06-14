# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

import tf2_ros
import numpy as np
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from machine_interfaces.msg import ServoStatus

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class DynamicFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('door_dynamic_tf2_transform')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        #self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
        
        #Create a subscriber node that subscribes to IMU Data
        #self.subscriber_ = self.create_subscription(Imu, '/imu/data', self.broadcast_timer_callback, 10)
        #self.get_logger().info("Imu Data Subscriber has started.")

        #Create a subscriber node that subscribes to Servo Location Data
        self.subscriber_ = self.create_subscription(ServoStatus, '/servo_0_status', self.broadcast_timer_callback, 1)
        self.get_logger().info("Servo Location Data Subscriber has started.")

    def broadcast_timer_callback(self, msg):
    #def broadcast_timer_callback(self):
        seconds, _ = self.get_clock().now().seconds_nanoseconds()

        try:  
            #self.get_logger().info(str(msg))

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            #t.child_frame_id = 'door_module'
            t.child_frame_id = 'door_link'            
            t.transform.translation.x = 0.0 #0.0   
            t.transform.translation.y = 0.0 #-1.0
            t.transform.translation.z = 0.0 #0.2

            # For the same reason, turtle can only rotate around one axis
            # and this why we set rotation in x and y to 0 and obtain
            # rotation in z axis from the message
            #q = quaternion_from_euler((msg.servo1_position_rad), 0, msg.servo0_position_rad)            
            q = quaternion_from_euler(0, 0, msg.servo0_position_rad)
            #q = quaternion_from_euler(0, 0, msg.servo0_position_rad)

            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            # t.transform.rotation.x = msg.orientation.x
            # t.transform.rotation.y = msg.orientation.y
            # t.transform.rotation.z = msg.orientation.z
            # t.transform.rotation.w = msg.orientation.w

            self.tf_broadcaster.sendTransform(t)

        except Exception as exception_error:
            print("Error: " + str(exception_error))
            self.get_logger().info("Imu Data Error.")

def main():
    rclpy.init()
    node = DynamicFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
