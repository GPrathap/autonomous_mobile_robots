from scipy.stats import norm
import matplotlib.pyplot as plt
import numpy as np

class KFEstimation():
    def __init__(self, dt=0.1, v=0.05, w=0.05, vx_mean=30, vy_mean=20):
        self.xk = []
        self.initModel(dt, w, v)
        self.getSensorMeasurements(num=300, vx_mean=vx_mean, vy_mean=vy_mean)
    
    def initModel(self, dt=0.05, w=0.09, v=0.5):
        self.dt = dt 
        self.x = np.matrix([[0.0,0.0,0.0,0.0]]).T 
        self.P = np.diag([1000.0,1000.0,1000.0,1000.0])
        self.Phi = np.matrix([ [1.0, 0.0, dt, 0.0],
                        [0.0, 1.0, 0.0, dt],
                        [0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0]])
        self.H = np.matrix([[0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]])
        self.G = np.matrix([[0.5*dt**2], [0.5*dt**2], [dt], [dt]])
        # TODO try obtain the approximation guess for w by considering
        # self.measurements
        self.R = np.matrix([[w, 0.0],
                            [0.0, w]])
        self.Q = self.G*self.G.T*v**2
        self.I = np.eye(4)
    
    def getSensorMeasurements(self, num=200, vx_mean=30, vy_mean=10, is_plot=True):
        self.vx_mean = vx_mean
        self.vy_mean = vy_mean
        self.v_x = np.array(self.vx_mean+np.random.randn(num))
        self.v_y = np.array(self.vy_mean+np.random.randn(num))
        # np.std(v_x)
        self.measurements = np.vstack((self.v_x, self.v_y))
        if(is_plot):
            self.plotSensorReading()

    def plotSensorReading(self, ):
        fig = plt.figure(figsize=(16, 5))
        m = self.v_x.shape[0]
        plt.step(range(m), self.v_x, label='$\dot x$')
        plt.step(range(m), self.v_y, label='$\dot y$')
        plt.ylabel(r'Velocity $m/s$')
        plt.title('Sensor reading')
        plt.legend(loc='best', prop={'size': 18})
 
    def applyFKEstimation(self, ):
        for n in range(len(self.measurements[0])):
            self.KFPredict()
            Z = self.measurements[:, n].reshape(2, 1)
            self.KFUpdate(Z)
            self.xk.append(self.x)
        self.xk = np.array(self.xk)

    def KFPredict(self, ):
        # ========================
        # project the state ahead
        self.x = self.Phi*self.x
        # project the error covariance ahead
        self.P = self.Phi*self.P*self.Phi.T + self.Q  
    
    def KFUpdate(self, Z):
        # ===============================
        # compute the Kalman Gain
        S = self.H*self.P*self.H.T + self.R
        K = (self.P*self.H.T) * np.linalg.pinv(S)
        # update the estimate via z
        y = Z - (self.H*self.x) # innovation or residual
        self.x = self.x + K*y
        # update the error covariance
        self.P = (self.I - (K*self.H))*self.P
        
    def plotStates(self, ):
        dxt = self.xk[:,2]
        dyt = self.xk[:,3]
        fig = plt.figure(figsize=(16, 9))
        plt.step(range(len(self.measurements[0])), dxt, label='$estimateVx$')
        plt.step(range(len(self.measurements[0])), dyt, label='$estimateVy$')

        plt.step(range(len(self.measurements[0])),
                self.measurements[0], label='$measurementVx$')
        plt.step(range(len(self.measurements[0])),
                self.measurements[1], label='$measurementVy$')

        plt.axhline(self.vx_mean, color='#999999', label='$trueVx$')
        plt.axhline(self.vy_mean, color='#999999', label='$trueVy$')

        plt.xlabel('Filter Step')
        plt.title('Estimate (Elements from State Vector $x$)')
        plt.legend(loc='best', prop={'size': 11})
        plt.ylabel('Velocity')

    def plotEstimatedPath(self, ):
        x_k = self.xk[:,0]
        y_k = self.xk[:,1]
        fig = plt.figure(figsize=(16, 16))
        plt.scatter(self.vx_mean, self.vy_mean, s=20, label='State', c='k')
        plt.scatter(x_k, y_k, s=100, label='Start', c='g')
        plt.scatter(x_k[-1], y_k[-1], s=100, label='Goal', c='r')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Position')
        plt.legend(loc='best')
        plt.axis('equal')

dt = 0.05
v = 0.09
w = 0.06

kf_estimator = KFEstimation(dt=0.1, v=0.09, w=0.3)
kf_estimator.applyFKEstimation()
kf_estimator.plotStates()
kf_estimator.plotEstimatedPath()

plt.show()

    