from maildelivery.world import enviorment,location, package
from maildelivery.agents import robot
from maildelivery.brains.brains0 import robot_planner

import numpy as np
import matplotlib.pyplot as plt
import gtsam

PLANNER = 'optic'

def build_env():
    docks = [location(0,np.array([0,0]),'dock')]

    x1 = location(1,np.array([1,0]),'intersection')
    x2 = location(2,np.array([2,0]),'intersection')
    x3 = location(3,np.array([1,1]),'intersection')
    x4 = location(4,np.array([2,1]),'intersection')
    intersections = [x1,x2,x3,x4]

    h5 = location(5,np.array([1,2]),'house')
    h6 = location(6,np.array([2,2]),'house')
    houses = [h5,h6]

    landmarks = sorted(houses + docks + intersections)

    connectivityList = [[0,1],[1,2],[2,4],[3,4],[1,3],[3,5],[4,6]]

    p0 = package(0,5,'location',6,100,landmarks[5].xy)
    p1 = package(1,6,'location',5,100,landmarks[6].xy)
    packages = [p0,p1]

    env = enviorment(landmarks, connectivityList, packages)
    return env

#buld enviorment
env = build_env()

#spawn robot
x0 = env.locations[0].xy[0]
y0 = env.locations[0].xy[1]
theta0 = location.angle(env.locations[0],env.locations[1])
r = robot(gtsam.Pose2(x0,y0,theta0),0)
r.last_location = 0
r.goal_location = 0

#ask for plan
planner = robot_planner()
planner.create_problem(env,[r])
plan = planner.solve_plan(planner_name = PLANNER)
if PLANNER == 'tamer':
    parsed_actions = planner.parse_actions(plan.actions, env)
elif PLANNER == 'optic':
    a = 1


#plot initial state
plt.ion()
_, ax = env.plot()
r.plot(ax)
for p in env.packages:
    p.plot(ax)
plt.draw()

#roll simulation
for action in parsed_actions:

    status = False
    while not(status):
        status = r.act(action, env)
        
        #update plot        
        r.plot(ax)
        for p in r.owned_packages:
            p.plot(ax)
        plt.pause(0.1)

#dont close window in the end
ax.set_title('finished!')
plt.ioff()
plt.show()



