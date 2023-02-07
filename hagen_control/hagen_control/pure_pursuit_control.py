import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as si


# ref https://stackoverflow.com/questions/34803197/fast-b-spline-algorithm-with-numpy-scipy
def bspline(cv, n=100, degree=3, periodic=False):
    cv = np.asarray(cv)
    count = len(cv)
    if periodic:
        factor, fraction = divmod(count+degree+1, count)
        cv = np.concatenate((cv,) * factor + (cv[:fraction],))
        count = len(cv)
        degree = np.clip(degree,1,degree)
    else:
        degree = np.clip(degree,1,count-1)
    kv = None
    if periodic:
        kv = np.arange(0-degree,count+degree+degree-1)
    else:
        kv = np.clip(np.arange(count+degree+1)-degree,0,count-degree)
    u = np.linspace(periodic,(count-degree),n)
    return np.array(si.splev(u, (kv,cv.T,degree))).T

class HagenRobot:
    def __init__(self, x=0, y=0, theta=0, v=0, L=2.4, Ts=0.01):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.L = L
        self.Ts = Ts

    def step(self, delta, a):
        # TODO Let the state of the robot be position (x,y), orientation theta,
        # and velocity, and control input for the robot be acceleration 
        # and steering angle. Define the state transition  
        self.x += self.v * np.cos(self.theta) * self.Ts
        self.y += self.v * np.sin(self.theta) * self.Ts
        self.theta += self.v * np.tan(delta) / self.L * self.Ts
        self.theta = np.fmod(self.theta, np.pi * 2)
        self.v += a * self.Ts

class PurePurSuitController:
    def __init__(self, robot_model, ref_path_x, ref_path_y, L_d=2.0, k=0.3, kp=1):
        self.hagen_robot = robot_model
        self.L_d = L_d 
        self.ref_path_x = ref_path_x
        self.ref_path_y = ref_path_y 
        self.k = k
        self.kp = 1 
        
    def pure_pursuit_control(self,):
        target_index = self.look_ahead_point_index()
        # TODO get the desired reference position
        t_x, t_y = self.ref_path_x[target_index], self.ref_path_y[target_index]
        # TODO estimate the alpha and delta 
        alpha = np.arctan2(t_y - self.hagen_robot.y, t_x - self.hagen_robot.x) - self.hagen_robot.theta
        delta = np.arctan(2 * self.hagen_robot.L * np.sin(alpha) / (self.k * (self.hagen_robot.v + 1e-5) + self.L_d))
        
        delta_min = -np.pi / 6
        delta_max = np.pi / 6
        if delta > delta_max:
            delta = delta_max
        elif delta < delta_min:
            delta = delta_min
        return delta, target_index

    def look_ahead_point_index(self):
        # TODO Estimate the distance to each control point from the center 
        # position of the back wheel axis and find out the "index" that gives 
        # the minimum distance to the reference position from the current 
        # location of the car.
        dx = [self.hagen_robot.x - t_x for t_x in self.ref_path_x]
        dy = [self.hagen_robot.y - t_y for t_y in self.ref_path_y]
        d = [np.abs(np.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        index = d.index(min(d))
        L = 0
        LF = self.k * self.hagen_robot.v + self.L_d
        while LF > L and (index + 1) < len(self.ref_path_x):
            dx = self.ref_path_x[index + 1] - self.ref_path_x[index]
            dy = self.ref_path_y[index + 1] - self.ref_path_y[index]
            L += np.sqrt(dx ** 2 + dy ** 2)
            index += 1
        return index


    def p_control(self, target, current):
        a = self.kp * (target - current)
        return a


def main():
    L_d = 2.0
    k = 0.1
    kp = 1
    T = 200
    target_velocity = 2.4
    
    start_p = [40, 15.0]
    cv = np.array([[ 50.,  25.],[ 59.,  12.], [ 50.,  10.], [ 57.,   2.], [ 40.,   4.],[ 40.,   14.]])
    p = bspline(cv, n=10000, degree=3,periodic=True)
    ref_path_x, ref_path_y = p.T

    hagen_robot = HagenRobot(x=start_p[0], y=start_p[1], theta=0, v=0, L = 2.9, Ts=0.1)
    pure_puresuit_controller = PurePurSuitController(hagen_robot, ref_path_x, ref_path_y, L_d, k, kp)
    
    x = [hagen_robot.x]
    y = [hagen_robot.y]
    time = 0

    while time <= T:
        a = pure_puresuit_controller.p_control(target_velocity, hagen_robot.v)
        delta, target_index = pure_puresuit_controller.pure_pursuit_control()
        hagen_robot.step(delta, a)

        time += hagen_robot.Ts
        x.append(hagen_robot.x)
        y.append(hagen_robot.y)

        plt.cla()
        plt.scatter(cv[:,0], cv[:,1], label="control points")
        plt.plot(ref_path_x, ref_path_y, ".r", label="reference trajectory")
        plt.plot(x, y, "-b", label="hagen_robot trajectory")
        plt.plot(ref_path_x[target_index], ref_path_y[target_index], "go", label="target")
        plt.axis("equal")
        plt.grid(True)
        plt.title("Pure Pursuit Controller")
        plt.pause(0.001)

if __name__ == "__main__":
    main()
