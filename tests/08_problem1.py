from maildelivery.datatypes import move, pickup, drop

from maildelivery.enviorment import enviorment
from maildelivery.objects import robot, landmark, package
import maildelivery.plotting as plotting
import numpy as np
import matplotlib.pyplot as plt

import gtsam

import unified_planning
from unified_planning.shortcuts import UserType, BoolType,\
        Fluent, InstantaneousAction, Problem, Object, OneshotPlanner, Or, Not
unified_planning.shortcuts.get_env().credits_stream = None #removes the printing planners credits 

def buildEnv():
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

    m = enviorment([], landmarks, connectivityList, packages)
    return m

env = createMap()
r = robot(gtsam.Pose2(env.landmarks[0].xy[0],env.landmarks[0].xy[1],landmark.angle(env.landmarks[0],env.landmarks[1])),0)
plan = createPlan(env,[r])
parsed_actions = parse_actions(plan.actions, env)

_, ax = plotting.spawnWorld()
m.plot(ax)
graphics_r = r.plot(ax)

# odom = gtsam.Pose2(0.4,0,0)
# cmd = move(odom)

plt.ion()

for action in parsed_actions:
    r.act(action)

    graphics_r.remove()
    graphics_r = r.plot(ax)
    plt.pause(0.5)

plt.show()



