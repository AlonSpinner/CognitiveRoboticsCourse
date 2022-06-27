from maildelivery.enviorment import enviorment
from maildelivery.objects import robot, landmark, package
import maildelivery.plotting as plotting
from maildelivery.brains import planner0, ROBOT_INDEX_SHIFT

import numpy as np
import matplotlib.pyplot as plt
import gtsam

def build_env():
    docks = [landmark(0,np.array([0,0]),'dock')]

    x1 = landmark(1,np.array([1,0]),'intersection')
    x2 = landmark(2,np.array([2,0]),'intersection')
    x3 = landmark(3,np.array([1,1]),'intersection')
    x4 = landmark(4,np.array([2,1]),'intersection')
    intersections = [x1,x2,x3,x4]

    h5 = landmark(5,np.array([1,2]),'house')
    h6 = landmark(6,np.array([2,2]),'house')
    houses = [h5,h6]

    landmarks = houses + docks + intersections
    connectivityList = [[0,1],[1,2],[2,4],[1,3],[3,5],[4,6]]

    p0 = package(0,5,6,100)
    p1 = package(1,6,5,100)
    packages = [p0,p1]

    env = enviorment(landmarks, connectivityList, packages)
    return env

#buld enviorment
env = build_env()

#spawn robot
x0 = env.landmarks[0].xy[0]
y0 = env.landmarks[0].xy[1]
theta0 = landmark.angle(env.landmarks[0],env.landmarks[1])
r = robot(gtsam.Pose2(x0,y0,theta0),0 + ROBOT_INDEX_SHIFT)

#ask for plan
planner = planner0()
plan = planner.create_plan(env,[r])
parsed_actions = planner.parse_actions(plan.actions, env)

#plot initial state
_, ax = plotting.spawnWorld()
env.plot(ax)
graphics_r = r.plot(ax)
plt.ion()

#roll simulation
for action in parsed_actions:

    status = False
    while not(status):
        status = r.act(action)
        #update plot
        graphics_r.remove()
        graphics_r = r.plot(ax)
        plt.pause(0.5)

#dont close window in the end
plt.show()



