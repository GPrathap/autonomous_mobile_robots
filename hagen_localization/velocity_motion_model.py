import numpy as np 
import matplotlib.pyplot as plt


x0 = np.array([0, 0, -np.pi/10]) #initial pose
xn = np.array([2.45, 2.45, -np.pi/5]) #final pose
dt = 0.1 # time step
v = 2.4 # linear velocity
w = (np.pi/5) # angular velocity
n = 20 # number of samples
u = np.array([v, w]) #the velocity command

def sample_normal_dist(b):
    tot = 0
    for i in range(0, 12):
        tot = tot +  np.random.normal(-b, b)
    sample = 1/2 * tot
    return sample

# ideal motion model
def velocity_motion_model_ideal(xk, u, dt):
    v = u[0]
    w = u[1]
    x = xk[0]
    y = xk[1]
    theta = xk[2]

    xp = x - (v/w)*np.sin(theta) + (v/w)*np.sin(theta + w*dt)
    yp = y + (v/w)*np.cos(theta) - (v/w)*np.cos(theta + w*dt)
    thetap = theta + w*dt

    x = np.array([xp, yp, thetap])
    return x

def velocity_motion_model_noisy(xk, u, dt, noise):
    v = u[0]
    w = u[1]
    x = xk[0]
    y = xk[1]
    theta = xk[2]

    vp = v + noise[0]
    wp = w + noise[1]

    xp = x - (vp/wp)*np.sin(theta) + (vp/wp)*np.sin(theta + wp*dt)
    yp = y + (vp/wp)*np.cos(theta) - (vp/wp)*np.cos(theta + wp*dt)
    thetap = theta + wp*dt + noise(3)*dt

    x = np.array([xp, yp, thetap])
    return x

# plotting noise motion model 
s = []
f = []
for i in  range(0, n):
    noise = [sample_normal_dist(.001), sample_normal_dist(.1), sample_normal_dist(.001)]
    xi = velocity_motion_model_noisy(x0, u, dt, noise)
    s.append(xi) 
    xf = velocity_motion_model_noisy(xf, u, dt, noise)
    f.append(xf)

s = np.array(s)
f = np.array(f)
plt.scatter(f[:,0], f[:,1], 15, 'filled'); #samples of final pose
plt.scatter(s[:,0], s[:,1], 15, 'filled'); #samples of initial pose

# plotting 
xi = velocity_motion_model_ideal(x0, u, dt);
s = xi
x = []
for i in range(0, n):
    xi = velocity_motion_model_ideal(xi, u, dt)
    x.append(x)

x = np.array(x)

plt.figure(figsize=(9, 4))

# plt.subplot(131)
# plt.bar(names, values)
# plt.subplot(132)
# plt.scatter(names, values)
# plt.subplot(133)
# plt.plot(names, values)
# plt.suptitle('Categorical Plotting')
# plt.show()

