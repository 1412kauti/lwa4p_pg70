#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,Pose
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Quaternion

import numpy as np
from numpy import cos, sin, pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sympy as sp

from kinematics import Kinematics

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.pos_pub = self.create_publisher(Float64MultiArray, '/ik_destination', 10)
        self.msg_data = Float64MultiArray()
        self.calculate()
        self.publisher_data()
        
    def calculate(self):
        q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
        spi = sp.pi

        DH_params = [[0, q1, 0, spi/2],
                    [0, q2+spi/2, 0.350, spi],
                    [0, q3+spi/2, 0, spi/2],
                    [0.350, q4, 0, -spi/2],
                    [0, q5, 0, spi/2],
                    [0, q6, 0, 0]]

        robot_arm = Kinematics(DH_params)

        joints = np.array([[0.0], [-pi/2], [0.0], [pi/2], [0.0], [0.0]])
        target = np.array([[1, 0, 0, 40],
                        [0, -1, 0, 10],
                        [0, 0, -1, 30],
                        [0, 0, 0, 1]])

        new_j, e_trace = robot_arm.i_kine(joints, target, error_trace=True)
        new_j = new_j.flatten().tolist()
        self.msg_data.data= new_j
    
    def publisher_data(self):
        while True:
            self.pos_pub.publish(self.msg_data)

def main(args=None):
    rclpy.init(args=args)
    move_robot = MoveRobot()
    rclpy.spin_once(move_robot)
    move_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()