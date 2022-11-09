# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.integrate import quad

# # Example 01 
# a = 0
# b = 5
# x = np.random.uniform(low=a, high=b, size=100)
 
# def integrand(x):
#     return  x**2

# # Monte Carlo integration
# y = integrand(x)
# y_bar = ((b-a)/x.shape[0])*np.sum(y)
    
# I = quad(integrand, a, b,)
# print("Actual integration: ", I[0], "Monte Carlo integration: ", y_bar)
# plt.scatter(x, y)
# plt.show()


import numpy as np
import matplotlib.pyplot as plt

n=1e5
radius = 5
u_min = -radius
u_max = radius

x = np.random.uniform(u_min, u_max, int(n))
y = np.random.uniform(u_min, u_max, int(n))

inside_x,  inside_y  = #TODO
outside_x, outside_y = #TODO 

fig, ax = plt.subplots(1)
ax.scatter(inside_x, inside_y, c='b', alpha=0.8, edgecolor=None)
ax.scatter(outside_x, outside_y, c='r', alpha=0.8, edgecolor=None)
area = #TODO 
value_of_pi = area/radius**2
print(value_of_pi)
plt.show()