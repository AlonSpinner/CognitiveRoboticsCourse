from maildelivery.datatypes import beacon, landmark, move

from maildelivery.map import Map
from maildelivery.robot import robot
import maildelivery.plotting as plotting
import numpy as np
import matplotlib.pyplot as plt

import gtsam

def createMap():
    b0 = beacon(0,np.array([7,2]),3)
    b1 = beacon(1,np.array([2,7]),1)
    b2 = beacon(2,np.array([6,7]),2)
    beacons = [b0,b1,b2]


    h0 = landmark(0,np.array([1,1]),'house')
    h1 = landmark(1,np.array([6,0]),'house')
    h2 = landmark(2,np.array([1,5]),'house')
    h3 = landmark(3,np.array([4,7]),'house')
    h4 = landmark(4,np.array([8,4]),'house')
    h5 = landmark(5,np.array([4,4]),'house')
    h6 = landmark(6,np.array([6,4]),'house')
    houses = [h0,h1,h2,h3,h4,h5,h6]

    x0 = landmark(7,np.array([6,2]),'intersection')
    x1 = landmark(8,np.array([4,2]),'intersection')
    x2 = landmark(9,np.array([1,7]),'intersection')
    x3 = landmark(10,np.array([8,7]),'intersection')
    intersections = [x0,x1,x2,x3]

    landmarks = houses + intersections
    connectivityList = [[0,8],[8,1],[8,5],[5,2],[5,3],[2,9],[5,6],[6,4],[4,10],[10,3],[9,3],[4,7],[7,1]]

    packages = []

    m = Map(beacons, landmarks, connectivityList, packages)
    return m

m = createMap()


lm0 = m.landmarks[0]
lm1 = m.landmarks[8]
r = robot(gtsam.Pose2(lm0.xy[0],lm0.xy[1],landmark.angle(lm0,lm1)))

_, ax = plotting.spawnWorld()
m.plot(ax)
graphics_r = r.plot(ax)

odom = gtsam.Pose2(0.4,0,0)
cmd = move(odom)

plt.ion()
for _ in range(5):
    r.move(cmd)

    graphics_r.remove()
    graphics_r = r.plot(ax)
    plt.pause(0.5)



