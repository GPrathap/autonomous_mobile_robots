import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot

class EKFEstimation():
    def __init__(self, dt=0.1, do_plotting=True):
        self.xk = []
        self.initModel(dt, do_plotting)
        
    def initModel(self, dt=0.05, do_plotting=True):
        self.nx = 4  # state vector [x y yaw v]'
        self.x_esti = np.zeros((self.nx, 1))
        self.xTrue = np.zeros((self.nx, 1))
        self.P_esti = np.eye(self.nx)
        self.xDR = np.zeros((self.nx, 1))  # dead reckoning
        self.dt = dt
        # variance of location on x-axis and y-axis, yaw angle, and velocity
        self.Q = np.matrix([[0.1, 0, 0, 0], 
                [0, 0.1, 0, 0], 
                [0, 0, np.deg2rad(1.0), 0],
                [0, 0, 0, 1.0]])**2
        
        self.R = np.matrix([[1.0,0.0],[0.0,1.0]])**2
        self.do_plotting = do_plotting

    def control(self, ):
        v = 1.0 # in (m/s)
        yaw_rate = 0.1 # in (rad/s)
        u = np.matrix([[v], [yaw_rate]])
        return u

    def observationModel(self, x):
        H = np.matrix([[1.0,0,0,0],[0,1.0,0,0]])
        z = H@x
        return z

    def motionModel(self, x, u):
        F = np.matrix([
                [1.0,0,0,0],
                [0,1.0,0,0],
                [0,0,1.0,0],
                [0,0,0,0]
                ])
        B = np.matrix([
                [self.dt*np.cos(x[2,0]),0],
                [self.dt*np.sin(x[2,0]),0],
                [0, self.dt],
                [1.0, 0]
                ])
        x = F@x+B@u
        return x

    def observation(self, u):
        self.xTrue = self.motionModel(self.xTrue, u)
        # adding noise to true GPS x and y readings along x and y directions 
        z = self.observationModel(self.xTrue) + (np.diag([0.5, 0.5]) ** 2) @ np.random.randn(2, 1)
        # adding noise to true input
        ud = u + (np.diag([1.0, np.deg2rad(30.0)]) ** 2) @ np.random.randn(2, 1)
        self.xDR = self.motionModel(self.xDR, ud)
        return z, ud

    def jacobianOfMotionModel(self, x, u):
        """
        TODO calculate jacobian of motion model J_F
        x_{t+1} = x_t+v*dt*cos(yaw)
        y_{t+1} = y_t+v*dt*sin(yaw)
        yaw_{t+1} = yaw_t+omega*dt
        v_{t+1} = v{t}
        
        dx/dyaw = -v*dt*sin(yaw)
        dx/dv = dt*cos(yaw)
        dy/dyaw = v*dt*cos(yaw)
        dy/dv = dt*sin(yaw)
        """
        J_F  = #
        return J_F

    def jacobianOfObservationModel(self, ):
        # TODO Calculate jacobian of observation model
        J_H = # 
        return J_H


    def applyEFKOneStep(self, z, u):
        self.EKFPredict(u)
        self.EKFUpdate(z)

    def EKFPredict(self, u):
        # ========================
        # project the state ahead
        self.x_pred = self.motionModel(self.x_esti, u)
        # TODO project the error covariance ahead
        J_F = 
        self.P_pred =  
    
    def EKFUpdate(self, z):
        # ===============================
        # TODO calculate jacobian of observation model 
        J_H = 
        # TODO compute the Kalman Gain
        K = 
        # update the estimate via z
        z_pred = self.observationModel(self.x_pred)
        # TODO calculate innovation or residual
        y = 
        # TODO update the error covariance
        self.x_esti = 
        self.P_esti = 
        
    def plotCovarianceEllipse(self, x_esti, P_esti):
        """
            the green point is positioning observation
            the red ellipse is estimated covariance ellipse with EKF
        """
        Pxy = P_esti[0:2, 0:2]
        eigval, eigvec = np.linalg.eig(Pxy)
        if eigval[0] >= eigval[1]:
            bigind = 0
            smallind = 1
        else:
            bigind = 1
            smallind = 0

        t = np.arange(0, 2 * math.pi + 0.1, 0.1)
        a = math.sqrt(eigval[bigind])
        b = math.sqrt(eigval[smallind])
        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]
        angle = math.atan2(eigvec[1, bigind], eigvec[0, bigind])
        rot = Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]
        fx = rot @ (np.array([x, y]))
        px = np.array(fx[0, :] + x_esti[0, 0]).flatten()
        py = np.array(fx[1, :] + x_esti[1, 0]).flatten()
        plt.plot(px, py, "--r")
        
    def executeEKFEstimation(self, duration=50):
        hxTrue = self.xTrue
        hxDR = self.xTrue
        hxEst = self.x_esti
        hz = np.zeros((2, 1))
        self.time = 0.0
        while duration >= self.time:
            self.time += self.dt
            
            u = self.control()
            z, ud = self.observation(u)
            
            self.applyEFKOneStep(z, ud)
            # storing data
            hxEst = np.hstack((hxEst, self.x_esti))
            hxDR = np.hstack((hxDR, self.xDR))
            hxTrue = np.hstack((hxTrue, self.xTrue))
            hz = np.hstack((hz, z))

            if self.do_plotting:
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [
                    exit(0) if event.key == 'escape' else None])
                plt.plot(hz[0, :], hz[1, :], ".g")
                plt.plot(np.array(hxTrue[0, :]).flatten(),
                        np.array(hxTrue[1, :]).flatten(), "-b", label='true trajectory')
                plt.plot(np.array(hxDR[0, :]).flatten(),
                        np.array(hxDR[1, :]).flatten(), "-k", label="dead reckoning trajectory")
                plt.plot(np.array(hxEst[0, :]).flatten(),
                        np.array(hxEst[1, :]).flatten(), "-r", label="estimated trajectory with EKF")
                self.plotCovarianceEllipse(self.x_esti, self.P_esti)
                plt.axis("equal")
                plt.legend(loc='best', prop={'size': 11})
                plt.grid(True)
                plt.pause(0.001)

dt = 0.1
duration = 50
ekf_estimation = EKFEstimation(dt=dt)
ekf_estimation.executeEKFEstimation(duration=duration)
        