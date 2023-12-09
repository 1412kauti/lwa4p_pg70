#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,Pose
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Quaternion
import numpy as np
from numpy import cos, sin, pi
import time

class Move_Control(Node):
    def __init__(self):
        super().__init__('move_control_draw')
        self.pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.sub = self.create_subscription(Float64MultiArray, '/ik_destination', self.step_pub, 10)
        self.step = 10
        self.start_positions = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.pub.publish(Float64MultiArray(data=self.start_positions))
    def step_pub(self, msg):
        self.destination_positions = np.array(msg.data)
        self.pub.publish(Float64MultiArray(data=self.destination_positions))
        time.sleep(0.07)        

def main(args=None):
    rclpy.init(args=args)
    move_control = Move_Control()
    rclpy.spin(move_control)
    move_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()