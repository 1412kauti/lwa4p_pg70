import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sympy as sp
import numpy as np
from math import pi,cos,sin

class Kinematics:
    def __init__(self, DH_params):
        self.DOF = 6
        self.q1, self.q2, self.q3, self.q4, self.q5, self.q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
        self.spi = sp.pi
        self.DH_params = DH_params

    def DH_trans_matrix(self, params):
        d, theta, a, alpha = (params[0], params[1], params[2], params[3])
        mat = sp.Matrix([[sp.cos(theta), -1*sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
                        [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -1*sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
                        [0, sp.sin(alpha), sp.cos(alpha), d],
                        [0, 0, 0, 1]])
        return mat

    def joint_transforms(self):
        transforms = []
        transforms.append(sp.eye(4))  # Assuming the first first joint is at the origin
        for el in self.DH_params:
            transforms.append(self.DH_trans_matrix(el))
        return transforms

    def jacobian_expr(self):
        transforms = self.joint_transforms()
        trans_EF = transforms[0]
        for mat in transforms[1:]:
            trans_EF = trans_EF * mat
        pos_EF = trans_EF[0:3, 3]
        J = sp.zeros(6, self.DOF)
        for joint in range(self.DOF):
            trans_joint = transforms[0]
            for mat in transforms[1:joint+1]:
                trans_joint = trans_joint * mat
            z_axis = trans_joint[0:3, 2]
            pos_joint = trans_joint[0:3, 3]
            Jv = z_axis.cross(pos_EF - pos_joint)
            Jw = z_axis
            J[0:3, joint] = Jv
            J[3:6, joint] = Jw
        J = sp.simplify(J)
        return J

    def jacobian_subs(self, joints, jacobian_sym):
        if isinstance(joints, np.ndarray):
            joints = joints.flatten().tolist()
        J_l = jacobian_sym
        J_l = J_l.subs(self.q1, joints[0])
        J_l = J_l.subs(self.q2, joints[1])
        J_l = J_l.subs(self.q3, joints[2])
        J_l = J_l.subs(self.q4, joints[3])
        J_l = J_l.subs(self.q5, joints[4])
        J_l = J_l.subs(self.q6, joints[5])
        return J_l

    def trans_EF_eval(self, joints):
        if isinstance(joints, np.ndarray):
            joints = joints.flatten().tolist()
        transforms = self.joint_transforms()
        trans_EF = transforms[0]
        for mat in transforms[1:]:
            trans_EF = trans_EF * mat
        trans_EF_cur = trans_EF
        trans_EF_cur = trans_EF_cur.subs(self.q1, joints[0])
        trans_EF_cur = trans_EF_cur.subs(self.q2, joints[1])
        trans_EF_cur = trans_EF_cur.subs(self.q3, joints[2])
        trans_EF_cur = trans_EF_cur.subs(self.q4, joints[3])
        trans_EF_cur = trans_EF_cur.subs(self.q5, joints[4])
        trans_EF_cur = trans_EF_cur.subs(self.q6, joints[5])
        return trans_EF_cur

    def plot_pose(self, joints):
        if isinstance(joints, np.ndarray):
            joints = joints.flatten().tolist()
        transforms = self.joint_transforms()
        trans_EF = self.trans_EF_eval(joints)
        pos_EF = trans_EF[0:3, 3]
        xs = []
        ys = []
        zs = []
        J = sp.zeros(6, self.DOF)
        for joint in range(self.DOF):
            trans_joint = transforms[0]
            for mat in transforms[1:joint+1]:
                trans_joint = trans_joint * mat
            pos_joint = trans_joint[0:3, 3]
            pos_joint = pos_joint.subs(self.q1, joints[0])
            pos_joint = pos_joint.subs(self.q2, joints[1])
            pos_joint = pos_joint.subs(self.q3, joints[2])
            pos_joint = pos_joint.subs(self.q4, joints[3])
            pos_joint = pos_joint.subs(self.q5, joints[4])
            pos_joint = pos_joint.subs(self.q6, joints[5])
            xs.append(pos_joint[0])
            ys.append(pos_joint[1])
            zs.append(pos_joint[2])
        xs.append(pos_EF[0])
        ys.append(pos_EF[1])
        zs.append(pos_EF[2])
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim3d(-60, 60)
        ax.set_ylim3d(-60, 60)
        ax.set_zlim3d(0, 120)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.plot(xs, ys, zs)

    def joint_limits(self, joints):
        if joints[0] < -2*self.spi/3:
            # joints[0] = -2*self.spi/3
            pass
        elif joints[0] > 2*self.spi/3:
            # joints[0] = 2*self.spi/3
            pass
        if joints[1] < -0.95*self.spi:
            # joints[1] = -0.95*self.spi
            pass
        elif joints[1] > 0:
            # joints[1] = 0
            pass
        if joints[2] < -0.463*self.spi:
            # joints[2] = -0.463*self.spi
            pass
        elif joints[2] > 0.48*self.spi:
            # joints[2] = 0.48*self.spi
            pass
        if joints[3] < -0.97*self.spi:
            # joints[3] = -0.97*self.spi
            pass
        elif joints[3] > 0.97*self.spi:
            # joints[3] = 0.97*self.spi
            pass
        if joints[4] < -3*self.spi/2:
            # joints[4] = -3*self.spi/2
            pass
        elif joints[4] > 3*self.spi/2:
            # joints[4] = 3*self.spi/2
            pass
        if joints[5] < -0.95*self.spi:
            # joints[5] = -0.95*self.spi
            pass
        elif joints[5] > 0.95*self.spi:
            # joints[5] = 0.95*self.spi
            pass
        return joints

    def i_kine(self, joints_init, target, error_trace=False, no_rotation=False, joint_lims=True):
        joints = joints_init
        xr_desired = target[0:3, 0:3]
        xt_desired = target[0:3, 3]
        x_dot_prev = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        e_trace = []
        iters = 0
        print("Finding symbolic jacobian")
        jacobian_symbolic = self.jacobian_expr()
        print("Starting IK loop")
        final_xt = 0
        while True:
            jac = self.jacobian_subs(joints, jacobian_symbolic)
            jac = np.array(jac).astype(np.float64)
            trans_EF_cur = self.trans_EF_eval(joints)
            trans_EF_cur = np.array(trans_EF_cur).astype(np.float64)
            xr_cur = trans_EF_cur[0:3, 0:3]
            xt_cur = trans_EF_cur[0:3, 3]
            final_xt = xt_cur
            xt_dot = xt_desired - xt_cur
            R = xr_desired @ xr_cur.T
            v = np.arccos((R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2)
            r = (0.5 * sin(v)) * np.array([[R[2, 1]-R[1, 2]],
                                           [R[0, 2]-R[2, 0]],
                                           [R[1, 0]-R[0, 1]]])
            xr_dot = 200 * r * sin(v)
            if no_rotation:
                xr_dot = 0 * r
            xt_dot = xt_dot.reshape((3, 1))
            x_dot = np.vstack((xt_dot, xr_dot))
            x_dot_norm = np.linalg.norm(x_dot)
            if x_dot_norm > 25:
                x_dot /= (x_dot_norm/25)
            x_dot_change = np.linalg.norm(x_dot - x_dot_prev)
            if x_dot_change < 0.005:
                break
            x_dot_prev = x_dot
            e_trace.append(x_dot_norm)
            Lambda = 12
            Alpha = 1
            joint_change = Alpha * np.linalg.inv(jac.T@jac + Lambda**2*np.eye(self.DOF)) @ jac.T @ x_dot
            joints += joint_change
            if joint_lims:
                joints = self.joint_limits(joints)
            iters += 1
        print("Done in {} iterations".format(iters))
        print("Final position is:")
        print(final_xt)


        return (joints, e_trace) if error_trace else joints
    
    def calculate_circle_points(self, radius, num_points, plane_of_rotation='XY'):
        if plane_of_rotation not in {'XY', 'XZ', 'YZ'}:
            raise ValueError("Invalid plane_of_rotation. Choose from 'XY', 'XZ', or 'YZ'.")

        points = []
        for i in range(num_points):
            angle = 2 * pi * i / num_points

            if plane_of_rotation == 'XY':
                x = radius * cos(angle)
                y = radius * sin(angle)
                z = 0
            elif plane_of_rotation == 'XZ':
                x = radius * cos(angle)
                y = 0
                z = radius * sin(angle)
            else:  # plane_of_rotation == 'YZ'
                x = 0
                y = radius * cos(angle)
                z = radius * sin(angle)

            points.append((x, y, z))

        return points

    def draw_circle(self, radius, num_points=100, plane_of_rotation='XY'):
        if plane_of_rotation not in {'XY', 'XZ', 'YZ'}:
            raise ValueError("Invalid plane_of_rotation. Choose from 'XY', 'XZ', or 'YZ'.")

        circle_points = self.calculate_circle_points(radius, num_points, plane_of_rotation)

        # Plotting the robot arm
        joints = np.array([[0.0], [-pi/2], [0.0], [pi/2], [0.0], [0.0]])
        # self.plot_pose(joints)

        # Plotting the circle
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim3d(-60, 60)
        ax.set_ylim3d(-60, 60)
        ax.set_zlim3d(0, 120)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')

        xs, ys, zs = zip(*circle_points)
        # ax.plot(xs, ys, zs, label='Circle', color='r')
        # ax.legend()
        # compound_list = []
        # result = []

        # for i in range(len(xs)):
        #     result = []
        #     result.append(xs[i])
        #     result.append(ys[i])
        #     result.append(zs[i])
        #     compound_list.append(result)
        
        # return compound_list 
        return xs, ys, zs



# q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
# spi = sp.pi

# DH_params = [[0, q1, 0, spi/2],
#              [0, q2+spi/2, 0.350, spi],
#              [0, q3+spi/2, 0, spi/2],
#              [0.350, q4, 0, -spi/2],
#              [0, q5, 0, spi/2],
#              [0, q6, 0, 0]]

# robot_arm = Kinematics(DH_params)

# joints = np.array([[0.0], [-pi/2], [0.0], [pi/2], [0.0], [0.0]])
# target = np.array([[1, 0, 0, 40],
#                    [0, -1, 0, 10],
#                    [0, 0, -1, 30],
#                    [0, 0, 0, 1]])

# new_j, e_trace = robot_arm.i_kine(joints, target, error_trace=True)
# print(new_j)
# robot_arm.plot_pose(new_j)

# plt.figure(figsize=(8, 8))
# plt.plot(e_trace)

def run1():
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
    print(new_j)
    robot_arm.plot_pose(new_j, DH_params)
    plt.figure(figsize=(8,8))
    plt.plot(e_trace)
    plt.title('Error Trace')

def run2():
    q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
    spi = sp.pi

    DH_params = [[0, q1, 0, spi/2],
                 [0, q2+spi/2, 0.350, spi],
                 [0, q3+spi/2, 0, spi/2],
                 [0.350, q4, 0, -spi/2],
                 [0, q5, 0, spi/2],
                 [0, q6, 0, 0]]

    robot_arm = Kinematics(DH_params)

    joints = np.array([[ 0.24997842],[-0.90489976],[ 0.0927195 ],[ 3.04734487],[-2.02265731],[ 0.22542674]])

    result = robot_arm.trans_EF_eval(joints)
    result = sp.Matrix(result)
    robot_arm.plot_pose([ 0.24997842,-0.90489976, 0.0927195,3.04734487,-2.02265731, 0.22542674])
    print(result)
# run()
# run2()
def run_with_circle():
    q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
    spi = sp.pi

    DH_params = [[0, q1, 0, spi/2],
                [0, q2+spi/2, 0.350, spi],
                [0, q3+spi/2, 0, spi/2],
                [0.350, q4, 0, -spi/2],
                [0, q5, 0, spi/2],
                [0, q6, 0, 0]]

    robot_arm = Kinematics(DH_params)
    points = robot_arm.draw_circle(radius=10, num_points=100, plane_of_rotation='XY')

    joints = np.array([[0.0], [-pi/2], [0.0], [pi/2], [0.0], [0.0]])
    for i in range(len(points)):
        target = np.array([[1, 0, 0, points[i][0]],
                        [0, -1, 0, points[i][1]],
                        [0, 0, -1, points[i][2]],
                        [0, 0, 0, 1]])

        new_j, e_trace = robot_arm.i_kine(joints, target, error_trace=True)
        new_j = new_j.flatten().tolist()
        print(new_j)

    # Draw the circle in the XY plane
    

    # plt.figure(figsize=(8, 8))
    # plt.plot(e_trace)
    # plt.title('Error Trace')

# run_with_circle()