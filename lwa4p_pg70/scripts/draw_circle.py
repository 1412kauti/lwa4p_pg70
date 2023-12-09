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
        self.joints = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
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

        self.robot_arm = Kinematics(DH_params)
        self.x, self.y, self.z = self.robot_arm.draw_circle(radius=10, num_points=100, plane_of_rotation='XY')
        print(self.x, self.y, self.z)
    
    def publisher_data(self):
        for i in range(len(self.x)):
            target = np.array([[1, 0, 0, self.x[i]],
                            [0, -1, 0, self.y[i]],
                            [0, 0, -1, self.z[i]],
                            [0, 0, 0, 1]])
            
            print(self.x[i], self.y[i], self.z[i])

            new_j, e_trace = self.robot_arm.i_kine(self.joints, target, error_trace=True)
            new_j = new_j.flatten().tolist()
            self.msg_data.data= new_j
            self.pos_pub.publish(self.msg_data)

def main(args=None):
    rclpy.init(args=args)
    move_robot = MoveRobot()
    rclpy.spin_once(move_robot)
    move_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()