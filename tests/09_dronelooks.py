from maildelivery.agents import drone, robot
import gtsam
import matplotlib.pyplot as plt
from maildelivery.world import plot_spawnWorld

_, ax = plot_spawnWorld()

pose0 = gtsam.Pose2(0.4,0.4,0.2)
d = drone(pose0,0)
d.plot(ax)


pose0 = gtsam.Pose2(0.7,0.4,0.6)
r = robot(pose0,0)
r.plot(ax)
r.plot_deadcharge(ax)

plt.show()

