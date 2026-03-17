import pandas as pd 
import numpy as np 
import matplotlib.pyplot as plt 
import seaborn as sns 
from sklearn.metrics import r2_score 
import scipy.stats as stats 
  
# dataset = [[2.4,3.13,4.51,6.1,9.4,13.8],[186,215,269,307,362,489]]

# sns.scatterplot(data=dataset, x='PWM',  
#                 y='Force', hue='Force') 
  
# plt.title('') 
# plt.xlabel('Force') 
# plt.ylabel('PWM') 
# # degree 2 polynomial fit or quadratic fit 
# ydata=[2.4,3.13,4.51,6.1,9.4,13.8]
# xdata=[186,215,269,307,362,489]

# model = np.poly1d(np.polyfit(xdata, 
#                              ydata, 2)) 
  


# print("r^2:",r2_score(ydata,  
#                model(xdata))) 

# # polynomial line visualization 
# polyline = np.linspace(0, 10, 100) 
# plt.scatter(xdata, ydata) 
# plt.plot(polyline, model(polyline)) 

# plt.savefig("out.png") 
  
# print(model) 















sns.scatterplot(data=dataset, x='PWM',  
                y='Force', hue='Force') 
  
plt.title('') 
plt.xlabel('Force') 
plt.ylabel('PWM') 
# degree 2 polynomial fit or quadratic fit 
xdata=dataset['PWM'][dataset['PWM']<1500]
ydata=dataset['Force'][dataset['PWM']<1500]

model = np.poly1d(np.polyfit(xdata, 
                             ydata, 2)) 
  


print("r^2:",r2_score(ydata,  
               model(xdata))) 

# polynomial line visualization 
polyline = np.linspace(0, 10, 100) 
plt.scatter(xdata, ydata) 
plt.plot(polyline, model(polyline)) 

plt.savefig("out.png") 
  
print(model) 

modeltest= np.poly1d([-0.02928,6.794,1510])

print("test model",r2_score(ydata,  
               modeltest(xdata))) 