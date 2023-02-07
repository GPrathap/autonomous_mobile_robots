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
        self.x += self.v * np.cos(self.theta) * self.Ts
        self.y += self.v * np.sin(self.theta) * self.Ts
        self.theta += self.v * np.tan(delta) / self.L * self.Ts
        self.theta = np.fmod(self.theta, np.pi * 2)
        self.v += a * self.Ts

class StanleyController:
    def __init__(self, robot_model, ref_path_x, ref_path_y, ref_path_theta, k=0.3, kp=1, kv=1, L=2.9):
        self.hagen_robot = robot_model
        self.ref_path_x = ref_path_x
        self.ref_path_y = ref_path_y 
        self.ref_path_theta = ref_path_theta 
        self.k = k
        self.kp = kp
        self.kv = kv 
        self.L = L
        
    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    def nearest_pose_index(self,):
        # TODO Estimate the distance to each control point from the center 
        # position of the front wheel axis (dx, dy) and find out the "index" that gives 
        # the minimum distance (d) to the reference position from the current 
        # location of the car (target_index).   
        fx = self.hagen_robot.x + self.L * np.cos(self.hagen_robot.theta)
        fy = self.hagen_robot.y + self.L * np.sin(self.hagen_robot.theta)
        dx = fx - self.ref_path_x 
        dy = fy - self.ref_path_y 
        d = np.hypot(dx, dy)
        target_index = np.argmin(d)
        return target_index, dx[target_index], dy[target_index], d[target_index]
    
    def calculate_crosstrack_term(self, target_velocity, dx, dy, absolute_error):
        front_axle_vector = np.array([np.sin(self.hagen_robot.theta), -np.cos(self.hagen_robot.theta)])
        nearest_path_vector = np.array([dx, dy])
        e = np.sign(nearest_path_vector@front_axle_vector) * absolute_error
        e_delta = np.arctan2((self.k * e), (self.kp*target_velocity))
        return e_delta, e
        
    def stanley_control(self, target_velocity):
        target_index, dx, dy, absolute_error = self.nearest_pose_index()
        t_theta = self.ref_path_theta[target_index]
        # TODO estimate e_phi and e_delta 
        e_phi = self.normalize_angle(t_theta - self.hagen_robot.theta)
        e_delta, e = self.calculate_crosstrack_term(target_velocity, dx, dy, absolute_error)    
        delta = e_phi + e_delta
        
        delta_min = - np.pi / 6
        delta_max = np.pi / 6
        if delta > delta_max:
            delta = delta_max
        elif delta < delta_min:
            delta = delta_min
        return delta, target_index

    def p_control(self, target, current):
        a = self.kv * (target - current)
        return a

def main():
    k = 0.1
    kp = 1
    kv = 1 
    T = 200
    target_velocity = 2.4
   
    start_p = [40, 15.0]
    cv = np.array([[ 50.,  25.],[ 59.,  12.], [ 50.,  10.], [ 57.,   2.], [ 40.,   4.],[ 40.,   14.]])
    p = bspline(cv, n=10000, degree=3,periodic=True)
    ref_path_x, ref_path_y = p.T
    ref_path_theta = [np.arctan2(ref_path_y[i + 1] - ref_path_y[i], ref_path_x[i + 1] - ref_path_x[i]) for i in range(0, len(ref_path_y) - 1, 1)]
    ref_path_theta.append(ref_path_theta[-1])

    hagen_robot = HagenRobot(x=start_p[0], y=start_p[1], theta=0, v=0, L = 2.9, Ts=0.1)
    stanley_controller = StanleyController(hagen_robot, ref_path_x, ref_path_y, ref_path_theta, k, kp, kv)
    
    x = [hagen_robot.x]
    y = [hagen_robot.y]
    time = 0

    while time <= T:
        a = stanley_controller.p_control(target_velocity, hagen_robot.v)
        delta,target_index  = stanley_controller.stanley_control(target_velocity)
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
        plt.title("Stanley Controller")
        plt.pause(0.001)

if __name__ == "__main__":
    main()
