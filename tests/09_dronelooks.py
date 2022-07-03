from maildelivery.agents import drone
import gtsam
import matplotlib.pyplot as plt
from maildelivery.world import plot_spawnWorld

_, ax = plot_spawnWorld()

pose0 = gtsam.Pose2(0.4,0.4,0.2)
d = drone(pose0,0)
d.plot(ax)
plt.show()