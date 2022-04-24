from maildelivery.datatypes import beacon, landmark

from maildelivery.map import Map
from maildelivery.robot import robot
import maildelivery.plotting as plotting
import numpy as np
import matplotlib.pyplot as plt

import gtsam

def createMap():
    b0 = beacon(0,np.array([7.0,2.0]),3)
    b1 = beacon(1,np.array([2.0,7.0]),1)
    b2 = beacon(2,np.array([6.0,7.0]),2)
    beacons = [b0,b1,b2]


    h0 = landmark(0,np.array([1,1]),'house')
    h1 = landmark(1,np.array([6,0]),'house')
    h2 = landmark(2,np.array([1,5]),'house')
    h3 = landmark(3,np.array([4,7]),'house')
    h4 = landmark(4,np.array([8,4]),'house')
    h5 = landmark(5,np.array([4,4]),'house')
    houses = [h0,h1,h2,h3,h4,h5]

    x0 = landmark(6,np.array([6,2]),'intersection')
    x1 = landmark(7,np.array([4,2]),'intersection')
    x2 = landmark(8,np.array([1,7]),'intersection')
    x3 = landmark(9,np.array([8,7]),'intersection')
    intersections = [x0,x1,x2,x3]

    landmarks = houses + intersections
    connectivityList = [[0,7],[7,1],[7,5],[5,2],[5,3],[2,8],[5,4],[4,9],[9,3],[8,3],[4,6],[6,1]]

    m = Map(beacons, landmarks, connectivityList)
    return m
m = createMap()

_, ax = plotting.spawnWorld()
m.plot(ax)

lm0 = m.landmarks[0]
lm1 = m.landmarks[m.find_adjacent(lm0)[0]]
r = robot(gtsam.Pose2(lm0.xy[0],lm0.xy[1],landmark.angle(lm0,lm1)))

r.plot(ax)
plt.show()



