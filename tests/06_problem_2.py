from maildelivery.world import enviorment,landmark, package
from maildelivery.agents import robot
from maildelivery.brains import planner0

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
import gtsam
import os

MOVIE = False


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

    landmarks = sorted(houses + docks + intersections)

    connectivityList = [[0,1],[1,2],[2,4],[1,3],[3,5],[4,6],[3,4]]

    p0 = package(0,5,'landmark',6,100,landmarks[5].xy)
    p1 = package(1,6,'landmark',5,100,landmarks[6].xy)
    packages = [p0,p1]

    env = enviorment(landmarks, connectivityList, packages)
    return env

#buld enviorment
env = build_env()

#spawn robots
x0 = env.landmarks[4].xy[0]
y0 = env.landmarks[4].xy[1]
theta0 = landmark.angle(env.landmarks[4],env.landmarks[3])
r0 = robot(gtsam.Pose2(x0,y0,theta0),0)
r0.last_landmark = 4

x0 = env.landmarks[3].xy[0]
y0 = env.landmarks[3].xy[1]
theta0 = landmark.angle(env.landmarks[3],env.landmarks[4])
r1 = robot(gtsam.Pose2(x0,y0,theta0),1)
r1.last_landmark = 3

r = [r0,r1]

#ask for plan
planner = planner0()
plan = planner.create_plan(env,r)
parsed_actions = planner.parse_actions(plan.actions, env)

#plot initial state
plt.ion()
fig, ax = env.plot()
[ri.plot(ax) for ri in r]
for p in env.packages:
    p.plot(ax)
plt.draw()

#ready movie
if MOVIE:
    moviewriter = PillowWriter(fps = 5)
    moviewriter.setup(fig,'06_movie.gif',dpi = 100)

#roll simulation
for action in parsed_actions:

    status = False
    while not(status):
        status = r[action.robot_id].act(action, env)
        
        #update plot        
        for ri in r:
            ri.plot(ax)
            for p in ri.owned_packages:
                p.plot(ax)
        
        if MOVIE:
            moviewriter.grab_frame()
        plt.pause(0.1)

#dont close window in the end
ax.set_title('finished!')
if MOVIE:
    moviewriter.finish()
plt.ioff()
plt.show()



