#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,Pose
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Quaternion
import numpy as np
import time

class Move_Control(Node):
    def __init__(self):
        super().__init__('move_control')
        self.pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.sub = self.create_subscription(Float64MultiArray, '/ik_destination', self.step_pub, 10)
        self.step = 10
        self.start_positions = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    def step_pub(self, msg):
        self.destination_positions = np.array(msg.data)
        self.step_sizes = [(end - start) / (self.step - 1) for start, end in zip(self.start_positions, self.destination_positions)]
        current_values = self.start_positions.copy()

        for i in range(self.step):
            self.pub.publish(Float64MultiArray(data=current_values))
            current_values += self.step_sizes
            current_values = [current + step for current, step in zip(current_values, self.step_sizes)]
            time.sleep(0.07)

def main(args=None):
    rclpy.init(args=args)
    move_control = Move_Control()
    rclpy.spin_once(move_control)
    move_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()