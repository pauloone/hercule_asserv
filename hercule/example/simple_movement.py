import matplotlib.pyplot as plt
from hercule.unicycle import Unicycle

uni = Unicycle(0.1)

step = 0.1

timerange = [0]
x = [0]
y = [0]
theta = [0]

current_time = 0

while current_time < 1:
    current_time += step
    uni.next_step(0.1, 1, step)
    x.append(uni.x)
    y.append(uni.y)
    theta.append(uni.theta)
    timerange.append(current_time)

plt.plot(timerange, x, label="X")
plt.plot(timerange, y, label="Y")
plt.plot(timerange, theta, label="Theta")
plt.legend()
plt.show()
