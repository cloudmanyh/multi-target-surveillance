import numpy as np
import matplotlib.pyplot as plt
fig, ax = plt.subplots()
x1 = np.random.uniform(-10, 10, size=20)
x2 = np.random.uniform(-10, 10, size=20)
#print(x1)
#print(x2)
number = []
x11 = []
x12 = []
for i in range(20):
    number.append(i+1)
    x11.append(i+1)
    x12.append(i+1)
plt.figure(1)
# you can specify the marker size two ways directly:
plt.plot(number, x1, 'bo', markersize=20,label='a')  # blue circle with size 20
plt.plot(number, x2, 'ro', ms=10,label='b')  # ms is just an alias for markersize
 
lgnd=plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0,numpoints=1,fontsize=10)
lgnd.legendHandles[0]._legmarker.set_markersize(16)
lgnd.legendHandles[1]._legmarker.set_markersize(10)
 
plt.show()
 
fig.savefig('scatter.eps',dpi=600,format='eps')