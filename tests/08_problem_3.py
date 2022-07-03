from maildelivery.world import enviorment,location, package
from maildelivery.agents import robot, wait
from maildelivery.brains.brains0 import brain

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
import gtsam
import os

MOVIE = False

def build_block(base_ind : int, bottomleft_xy : np.ndarray):
    x_d = 1.0
    h_d = 0.4

    x_bl = location(base_ind + 0,bottomleft_xy + np.array([0,0]),'intersection')
    x_br = location(base_ind + 1,bottomleft_xy + np.array([x_d,0]),'intersection')
    x_tr = location(base_ind + 2,bottomleft_xy + np.array([x_d,x_d]),'intersection')
    x_tl = location(base_ind + 3,bottomleft_xy + np.array([0,x_d]),'intersection')
    x = [x_bl,x_br,x_tr,x_tl]

    h_bl = location(base_ind + 4, bottomleft_xy + np.array([h_d,h_d]),'house')
    h_br = location(base_ind + 5, bottomleft_xy + np.array([x_d - h_d, h_d]),'house')
    h_tr = location(base_ind + 6, bottomleft_xy + np.array([x_d - h_d, x_d - h_d]),'house')
    h_tl = location(base_ind + 7, bottomleft_xy + np.array([h_d, x_d - h_d]),'house')
    h = [h_bl,h_br,h_tr,h_tl]

    c = ([
            [x_bl.id, h_bl.id],
            [x_br.id, h_br.id],
            [x_tr.id, h_tr.id],
            [x_tl.id, h_tl.id],
            [x_bl.id, x_br.id],
            [x_br.id, x_tr.id],
            [x_tr.id, x_tl.id],
            [x_tl.id, x_bl.id]
            ])

    return x, h, c

def build_env():
    x_a,h_a, c_a = build_block(0, np.array([0,0]))

    locations = sorted(x_a + h_a)

    p0 = package(0,h_a[2].id,'location',h_a[0].id,100, h_a[2].xy)
    p1 = package(1,h_a[0].id,'location',h_a[2].id,100, h_a[0].xy)
    packages = [p0,p1]

    env = enviorment(locations, c_a, packages)
    return env

#buld enviorment
env = build_env()

#spawn robots
x0 = env.locations[4].xy[0]
y0 = env.locations[4].xy[1]
theta0 = 0.0
r0 = robot(gtsam.Pose2(x0,y0,theta0),0)
r0.last_location = 4

x0 = env.locations[6].xy[0]
y0 = env.locations[6].xy[1]
theta0 = 0
r1 = robot(gtsam.Pose2(x0,y0,theta0),1)
r1.last_location = 6

r = [r0,r1]
Nrobots = len(r)

#ask for plan
planner = brain()
plan = planner.create_plan(env,r)
parsed_actions = planner.parse_actions(plan.actions, env)
actions_per_robot = planner.actions_per_robot(parsed_actions, Nrobots)

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
    moviewriter.setup(fig,'08_movie.gif',dpi = 100)

#roll simulation
current_action_indicies = [0 for _ in range(len(r))]
while True:
    current_actions = [wait for _ in range(len(r))]
    finished_all_actions = True
    
    for ri in range(len(r)):
        if current_action_indicies[ri] < len(actions_per_robot[ri]): #still actions to do
            current_actions[ri] = actions_per_robot[ri][current_action_indicies[ri]]
            current_action_indicies[ri] += 1
            finished_all_actions = False

    if finished_all_actions:
        break

    #perform current actions
    status = False #just to initialize
    while status is False:
        
        status = True
        for action in current_actions:
            status = r[action.robot_id].act(action, env) and status

        #update plot        
        for ri in r:
            ri.plot(ax)
            for p in ri.owned_packages:
                p.plot(ax)
                plt.pause(0.1) 

        if MOVIE:
            moviewriter.grab_frame()
        plt.pause(.1)

#dont close window in the end
ax.set_title('finished!')
if MOVIE:
    moviewriter.finish()
plt.ioff()
plt.show()
