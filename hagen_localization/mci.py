import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad

# Example 01 
a = 0
b = 5
x = np.random.uniform(low=a, high=b, size=100)
 
def integrand(x):
    return  x**2

# Monte Carlo integration
y = integrand(x)
y_bar = 
    
I = quad(integrand, a, b,)
print("Actual integration: ", I[0], "Monte Carlo integration: ", y_bar)
plt.scatter(x, y)
plt.show()