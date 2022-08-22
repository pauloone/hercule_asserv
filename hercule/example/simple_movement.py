import numpy
import matplotlib.pyplot as plt
from hercule.unicycle import Unicycle
from hercule.backstepping import Backstepping

step = 0.1

uni = Unicycle(step)
backstepping = Backstepping(1,1,1,1,uni, step)

timerange = [0]
x = [0]
y = [0]
theta = [0]

current_time = 0

cx = 0

while current_time < 10:
    current_time += step
    cx += 0.01
    command = backstepping.next_step(numpy.array([cx, 0]))
    print(command)
    uni.next_step(command[0], command[1])
    x.append(uni.x)
    y.append(uni.y)
    theta.append(uni.theta)
    timerange.append(current_time)

plt.plot(x, y)
plt.show()
