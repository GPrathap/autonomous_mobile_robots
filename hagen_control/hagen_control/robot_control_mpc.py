from time import time
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import cv2
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
import std_msgs.msg as std_msg
import rclpy.qos as qos


class MPCDiffDrivePLanner():
    def __init__(self, deltat, horizon) -> None:
        self.T = 0.05
        self.N = 10
        self.v_max = 0.6
        self.v_min = -self.v_max
        self.omega_max = pi/2
        self.omega_min = -self.omega_max
        self.map_dim = [-200, 200, -200, 200]
        self.init_reg = False

        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)
        self.n_status = states.size()[0]

        v = ca.SX.sym('v_x')
        omega = ca.SX.sym('omega')

        controls = ca.vertcat(v, omega)
        self.n_controls = controls.size()[0]
        rhs = ca.vertcat(v*cos(theta), v*sin(theta), omega)

        self.f = ca.Function('f', [states, controls], [rhs])

        U = ca.SX.sym('U', self.n_controls, self.N)
        P = ca.SX.sym('P', self.n_status + self.n_status)
        X = ca.SX.sym('X', self.n_status, self.N+1)

        Q = ca.DM.zeros(3,3)
        Q[0,0] = 1
        Q[1,1] = 1
        Q[2,2] = 0.1
       
        R = ca.DM.zeros(2,2)
        R[0,0] = 1.0
        R[1,1] = 1.0

        obj = 0
        g = ca.SX.sym('g', self.N+1, self.n_status)
        st = X[:,0]

        g[0,:] = st - P[0 : self.n_status]
        ibj = 1
        for k in range(0, self.N):
            st = X[:,k]
            con = U[:,k]
            obj = obj + ca.mtimes((st-P[3:6]).T, ca.mtimes(Q,(st-P[3:6]))) + ca.mtimes((con).T, ca.mtimes(R, (con))) 
            st_next = X[:,k+1]
            f_value = self.f(st, con)
            st_next_euler = st + self.T*f_value
            g[ibj,:] = st_next - st_next_euler
            ibj += 1

        g = ca.reshape(g, self.n_status*(self.N+1), 1)
        OPT_variables = ca.vertcat(ca.reshape(X, self.n_status*(self.N+1), 1), ca.reshape(U, self.n_controls*self.N, 1))

        opts = {}
        opts["expand"] = True
        opts["ipopt.max_iter"] = 2000
        opts["ipopt.tol"] = 1e-4
        opts["ipopt.print_level"] = 0
        opts["print_time"] = 0
        opts["ipopt.acceptable_tol"] = 1e-8

        nlp_prob = {'f':obj, 'x':OPT_variables, 'p':P, 'g':g}
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        lbg = ca.DM(1, self.n_status*(self.N+1))
        ubg = ca.DM(1, self.n_status*(self.N+1))

        lbg[0,0:self.n_status*(self.N+1):1] = 0
        ubg[0,0:self.n_status*(self.N+1):1] = 0

        lbx = ca.DM(self.n_status*(self.N+1)+self.n_controls*self.N,1)
        ubx = ca.DM(self.n_status*(self.N+1)+self.n_controls*self.N,1)

        lbx[0:self.n_status*(self.N+1):self.n_status,0] = self.map_dim[0]
        ubx[0:self.n_status*(self.N+1):self.n_status,0] = self.map_dim[1]
        lbx[1:self.n_status*(self.N+1):self.n_status,0] = self.map_dim[2]
        ubx[1:self.n_status*(self.N+1):self.n_status,0] = self.map_dim[3]
        lbx[2:self.n_status*(self.N+1):self.n_status,0] = -ca.inf
        ubx[2:self.n_status*(self.N+1):self.n_status,0] = ca.inf

        lbx[self.n_status*(self.N+1):self.n_status*(self.N+1) + self.n_controls*self.N:self.n_controls,0] = self.v_min
        ubx[self.n_status*(self.N+1):self.n_status*(self.N+1) + self.n_controls*self.N:self.n_controls,0] = self.v_max
        lbx[self.n_status*(self.N+1)+1:self.n_status*(self.N+1) +self.n_controls*self.N:self.n_controls,0] = self.omega_min
        ubx[self.n_status*(self.N+1)+1:self.n_status*(self.N+1) +self.n_controls*self.N:self.n_controls,0] = self.omega_max
        self.args = {'lbx':lbx, 'ubx':ubx, 'lbg':lbg, 'ubg':ubg, 'p':[], 'x0':[0.5, 1.0, 0.0]}
        
    def shift(self, T, t0, x0, u):
        st = x0
        con = u[0,:].T 
        f_value = self.f(st, con)
        st = st + (T*f_value)
        x0 = st
        t0 = t0 + T
        u_rest = u[1:u.size()[0]:1,:]
        u_last = u[u.size()[0]-1:u.size()[0]:1,:]
        self.u0 = ca.vertcat(u_rest, u_last)
        return t0, x0, self.u0
        
    def init_regulator(self, start_pose, target_pose):
        self.init_reg = True
        self.t0 = 0
        self.x0 = ca.DM([[start_pose[0]], [start_pose[1]], [start_pose[2]]])
        self.xs = ca.DM([[target_pose[0]], [target_pose[1]], [target_pose[2]]])

        xx = ca.DM(self.n_status, 300)
        xx[:,0] = self.x0
        t = ca.DM(1, 300)
        t[0] = self.t0
        self.u0 = ca.DM.zeros(self.N, self.n_controls)
        self.X0 = ca.repmat(self.x0, 1, self.N+1).T
        sim_time = 30
        self.mpciter = 0
        xx1 = []
        u_cl = []
        xx0 = []
        print("x0", self.X0.size())
        self.error_ = ca.norm_2(self.x0-self.xs)
        
    def update(self, odom_pose):
        current_time = self.mpciter*self.T
        self.x0 = ca.DM([[odom_pose[0]], [odom_pose[1]], [odom_pose[2]]]) 
        self.args['p'] = ca.vertcat(self.x0, self.xs)
        self.args['x0'] = ca.vertcat(ca.reshape(self.X0.T, self.n_status*(self.N+1), 1), ca.reshape(self.u0.T, self.n_controls*self.N, 1))
        sol = self.solver(**self.args)
        u = ca.reshape(sol['x'][self.n_status*(self.N+1):sol['x'].size()[0]:1].T, self.n_controls, self.N).T
        
        self.t0, self.x0, self.u0 = self.shift(self.T, self.t0, self.x0, u) 
        self.X0 = ca.reshape(sol['x'][0:self.n_status*(self.N+1)].T, self.n_status, self.N+1).T
        self.X0 = ca.reshape(sol['x'][0:self.n_status*(self.N+1):1].T, self.n_status, self.N+1).T
        x0_rest = self.X0[1:self.X0.size()[0]:1,:]
        x0_last = self.X0[self.X0.size()[0]-1:self.X0.size()[0]:1,:]
        self.X0 = ca.vertcat(x0_rest, x0_last)
        self.mpciter = self.mpciter + 1
        self.error_ =  ca.norm_2(self.x0-self.xs)
        print(self.error_)
        print("states: current: ", self.x0, " desired: ", self.xs)
        # print("control: ", self.u0[0,:])
        return  self.u0[0,:].full().flatten(), self.x0
        
class ControlStrategy(Node):
    def __init__(self, delta_t, ):
        super().__init__('control_strategy')
        self.publisher_ = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.vel_sub = self.create_subscription(Twist, '/hagen/cmd_vel', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/hagen/odom", self.set_pose, 20)
        
        self.img_sub = self.create_subscription(Image
            , '/depth_camera/image_raw', self.img_callback, 3)
        
        self.img_info_sub = self.create_subscription(CameraInfo
            , '/depth_camera/camera_info', self.camera_info_callback, 3)
        
        self.scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 3)
        
        self.i = 0
        self.set_q_init = None
        self.q = None 
        self.r = 0.3 # Wheel radius
        self.L = 1.25 # Axle length
        self.D = 0.07 # Distance between the front front whell and rear axle
        self.Ts = delta_t # Sampling time
        self.t = np.arange(0, 10, self.Ts) # Simulation time
        self.end_controller = False
        self.timer = self.create_timer(self.Ts, self.timer_callback)

    def img_callback(self, m : Image):
        np_img = np.reshape(m.data, (m.height, m.width, 3)).astype(np.uint8)
        self.display(np_img)
        
    def camera_info_callback(self, m : CameraInfo):
        # print(m)
        pass 
 
    def display(self, img : np.ndarray):
        cv2.imshow("camera view", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)
        
    def scan_callback(self, m : LaserScan):
        scan_data = np.array(m.ranges)
              
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def listener_callback(self, msg):
        pass

    def stop_vehicle(self, ):
        self.send_vel(0.0, 0.0)    
    
    def wrap_to_pi(self, x):
        x = np.array([x])
        xwrap = np.remainder(x, 2*np.pi)
        mask = np.abs(xwrap)>np.pi
        xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
        return xwrap[0]
    
    def diff_drive_init(self, duration=10):
        self.duration = duration
        self.wL = 12 # Left wheel velocity
        self.wR = 12 # Right wheel velocity
        self.time_utilized = 0.0
        
    def mpc_planner_init(self, target_pose):
        self.mpc_solver = MPCDiffDrivePLanner(self.Ts, 20)
        self.target_pose = target_pose
        if(self.q is None):
            print("Still robot current pose is not set")
        else:
            self.mpc_solver.init_regulator(self.q, target_pose)
        
    def mpc_planner(self, ):
        if(self.mpc_solver.init_reg):
            u, x = self.mpc_solver.update(self.odom_pose)
            v = u[0]
            w = u[1]
            self.send_vel(v, w)
        elif(self.q is not None):
            self.mpc_solver.init_regulator(self.q, self.target_pose)
            u, x = self.mpc_solver.update(self.odom_pose)
            v = u[0]
            w = u[1]
            self.send_vel(v, w)
        
    def inter_point_diff_drive_init(self, duration=10, r_distance=1.3
                    , refPose=np.array([3,6,0]), k_p=0.5, k_w=0.7, dmin=0.7):
        
        self.duration = duration
        self.r_distance = r_distance
        self.refPose = refPose
        self.k_p = k_p 
        self.k_w = k_w
        self.dmin = dmin
        self.time_utilized = 0.0
        self.xT = self.refPose[0]  - self.r_distance*np.cos(self.refPose[2])
        self.yT = self.refPose[1]  - self.r_distance*np.sin(self.refPose[2])
        self.state = 0
        
    def inter_direction_diff_drive_init(self, duration=10, r_distance=1.3
                    , refPose=np.array([3,6,0]), k_p=0.5, k_w=0.7, dmin=0.7):
        self.duration = duration
        self.r_distance = r_distance
        self.refPose = refPose
        self.k_p = k_p 
        self.k_w = k_w
        self.dmin = dmin
        self.time_utilized = 0.0
    
    def inter_direction_diff_drive(self, ):
        if(self.q is not None):
            if(self.duration < self.time_utilized):
                print("End of simulation")
                self.send_vel(0.0, 0.0)
                self.end_controller = True

            self.D = np.sqrt((self.q[0]-self.refPose[0])**2 
                                    + (self.q[1]-self.refPose[1])**2)

            if(self.D < self.dmin):
                print("Reach to the goal pose")
                self.send_vel(0.0, 0.0)
                self.end_controller = True

            beta = np.arctan(self.r_distance/self.D)
            phiR = np.arctan2(self.refPose[1]-self.q[1], self.refPose[0]-self.q[0])
            alpha = self.wrap_to_pi(phiR-self.refPose[2])
            
            if(alpha <0):
                beta = -beta
            ##Controller
            if(np.abs(alpha) < np.abs(beta)):
                ePhi = self.wrap_to_pi(phiR - self.q[2] + alpha) 
            else:
                ePhi = self.wrap_to_pi(phiR - self.q[2] + beta) 
                
            v = self.k_p*self.D
            w = self.k_w*ePhi
            print("Distance to the goal: ", self.D)
            dq = np.array([v*np.cos(self.q[2]+self.Ts*w/2)
                            , v*np.sin(self.q[2]+self.Ts*w/2), w])
            self.q = self.q + self.Ts*dq # Integration
            self.q[2] = self.wrap_to_pi(self.q[2]) # Map orientation angle to [-pi, pi]
            self.send_vel(v, w)
            self.time_utilized  =  self.time_utilized + self.Ts    
        
    def inter_point_diff_drive(self, ):
        if(self.q is not None):
            if(self.duration < self.time_utilized):
                self.stop_vehicle()
                print("End of simulation")
                self.end_controller = True

            self.D = np.sqrt((self.q[0]-self.refPose[0])**2 + (self.q[1]-self.refPose[1])**2)

            if(self.D < self.dmin):
                self.stop_vehicle()
                print("Reach to the goal pose")
                self.end_controller = True

            if self.state == 0:
                d = np.sqrt((self.yT-self.q[1])**2 + (self.xT-self.q[0])**2)
                if(d < self.dmin):
                    self.state = 1
                self.phiT = np.arctan2(self.yT-self.q[1], self.xT-self.q[0])
                self.ePhi = self.phiT - self.q[2]
            else:
                self.ePhi = self.refPose[2] - self.q[2]
            
            v = self.k_p*self.D
            w = self.k_w*self.ePhi
            print("Distance to the goal: ", self.D)
            dq = np.array([v*np.cos(self.q[2]+self.Ts*w/2), v*np.sin(self.q[2]+self.Ts*w/2), w])
            self.q = self.q + self.Ts*dq # Integration
            self.q[2] = self.wrap_to_pi(self.q[2]) # Map orientation angle to [-pi, pi]
            self.send_vel(v, w)
            self.time_utilized  =  self.time_utilized + self.Ts 

    def perform_action_diff_drive_one_step(self):
        v = self.r/2*(self.wR+self.wL) # Robot velocity
        w = self.r/self.L*(self.wR-self.wL) # Robot angular velocity
        dq = np.array([v*np.cos(self.q[2]+self.Ts*w/2), v*np.sin(self.q[2]+self.Ts*w/2), w])
        self.q = self.q + self.Ts*dq # Integration
        self.q[2] = self.wrap_to_pi(self.q[2]) # Map orientation angle to [-pi, pi]
        self.send_vel(v, w)
        # rate.sleep()
        # time.sleep(self.Ts)
        self.time_utilized  =  self.time_utilized + self.Ts


    def timer_callback(self, ):
        # self.perform_action_diff_drive_one_step()
        # self.inter_direction_diff_drive()
        # self.inter_point_diff_drive()
        self.mpc_planner()
        return 
    
    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        if(self.set_q_init is None):
            self.set_q_init = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
            self.q = self.set_q_init
        self.odom_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    control_strategy = ControlStrategy(delta_t=0.03)
    # control_strategy.diff_drive_init()
    # control_strategy.inter_point_diff_drive_init()
    control_strategy.mpc_planner_init(np.array([5, 5, np.pi/4]))
    while control_strategy.end_controller is False and rclpy.ok():
        try:
            rclpy.spin_once(control_strategy)
        except KeyboardInterrupt:
            break
        
    control_strategy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# To reset ros2 service call /reset_simulation std_srvs/srv/Empty

