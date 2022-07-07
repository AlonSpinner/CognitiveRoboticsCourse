from maildelivery.world import enviorment,location, package
from maildelivery.agents import robot, drop, wait
from maildelivery.brains.brains1 import robot_planner

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
import gtsam
import os

DT = 0.001
L = 1.0
T = 1.0
MOVIE = True
dir_path = os.path.dirname(__file__)
MOVIE_FILENAME = os.path.join(dir_path,'06_movie.gif')

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

    locations = sorted(houses + docks + intersections)

    connectivityList = [[0,1],[1,2],[2,4],[1,3],[3,5],[4,6],[3,4]]

    p0 = package(0,5,'location',6,100,locations[5].xy)
    p1 = package(1,6,'location',5,100,locations[6].xy)
    packages = [p0,p1]

    env = enviorment(locations, connectivityList, packages)
    return env

#buld enviorment
env = build_env()

#spawn robots
l0 = 5
x0 = env.locations[l0].xy[0]
y0 = env.locations[l0].xy[1]
theta0 = np.pi/2
r0 = robot(gtsam.Pose2(x0,y0,theta0),0)
r0.last_location = l0
r0.goal_location = l0
r0.max_forward = L/T * DT

l0 = 6
x0 = env.locations[l0].xy[0]
y0 = env.locations[l0].xy[1]
theta0 = np.pi/2
r1 = robot(gtsam.Pose2(x0,y0,theta0),1)
r1.last_location = l0
r1.goal_location = l0
r1.max_forward = L/T * DT

r = [r0,r1]
Nrobots = len(r)

#ask for plan
planner = robot_planner()
planner.create_problem(env,r)
r_execution_times, r_actions, r_durations = planner.solve_and_parse(env)

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
    moviewriter.setup(fig,MOVIE_FILENAME,dpi = 100)

#roll simulation
t = 0
plotCounter = 0

r_current_actions = [wait(i) for i in range(Nrobots)]
r_next_actions_indicies = [0 for _ in range(Nrobots)]
r_done = [False for _ in range(Nrobots)]
while True:

    for i,ri in enumerate(r):
        #go do next action
        if r_done[i] == False and \
            type(r_current_actions[i]) == wait and \
                t > r_execution_times[i][r_next_actions_indicies[i]]:
            r_current_actions[i] = r_actions[i][r_next_actions_indicies[i]]
            r_current_actions[i] #we update index so 
            r_next_actions_indicies[i] += 1
             
        if ri.act(r_current_actions[i], env): #do action, and if its finished, start waiting allowing accepting new actions
            r_current_actions[i] = wait(robot_id = i)

    #update plot        
    if plotCounter % 100 == 0:
        for ri in r:
            ri.plot(ax)
            for p in ri.owned_packages:
                p.plot(ax)
        
        if MOVIE:
            moviewriter.grab_frame()
        plt.pause(0.1)
    plotCounter +=1

    t += DT

    for i in range(Nrobots):
        r_done[i] = r_next_actions_indicies[i] == len(r_actions[i]) and type(r_current_actions[i]) == wait

    if all(r_done):
        for ri in r:
            ri.plot(ax)
        for p in env.packages:
            p.plot(ax)
        if MOVIE:
            moviewriter.grab_frame()
        break
    
#dont close window in the end
ax.set_title('finished!')
if MOVIE:
    moviewriter.finish()

for ri in r:
    print(f"robot has {ri.charge}/{ri.max_charge} charge left")

plt.ioff()
plt.show()



