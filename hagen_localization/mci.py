import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad

# Example 01 
a = 0
b = 5
x = np.random.uniform(low=a, high=b, size=100)
# Monte Carlo integration
y = x**2
y_bar =  

def integrand(x):
    return  x**2
    
I = quad(integrand, a, b,)
print("Actual integration: ", I[0], "Monte Carlo integration: ", y_bar)
plt.scatter(x, y)
plt.show()