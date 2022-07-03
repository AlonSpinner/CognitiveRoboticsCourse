from maildelivery.world import enviorment,location, package
from maildelivery.agents import robot, wait, drop
from maildelivery.brains.brains0 import robot_planner

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
import gtsam
import os

MOVIE = True
dir_path = os.path.dirname(__file__)
MOVIE_FILENAME = os.path.join(dir_path,'08_movie.gif')
X_D = 1.0
H_D = 0.4

def build_block(base_ind : int, bottomleft_xy : np.ndarray):
    x_bl = location(base_ind + 0,bottomleft_xy + np.array([0,0]),'intersection')
    x_br = location(base_ind + 1,bottomleft_xy + np.array([X_D,0]),'intersection')
    x_tr = location(base_ind + 2,bottomleft_xy + np.array([X_D,X_D]),'intersection')
    x_tl = location(base_ind + 3,bottomleft_xy + np.array([0,X_D]),'intersection')
    x = [x_bl,x_br,x_tr,x_tl]

    h_bl = location(base_ind + 4, bottomleft_xy + np.array([H_D,H_D]),'house')
    h_br = location(base_ind + 5, bottomleft_xy + np.array([X_D - H_D, H_D]),'house')
    h_tr = location(base_ind + 6, bottomleft_xy + np.array([X_D - H_D, X_D - H_D]),'house')
    h_tl = location(base_ind + 7, bottomleft_xy + np.array([H_D, X_D - H_D]),'house')
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

    n = len(x) + len(h)

    return x, h, c, n

def build_leftconnected_block(base_ind : int, bottomleft_xy : np.ndarray):
    x_br = location(base_ind + 0,bottomleft_xy + np.array([X_D,0]),'intersection')
    x_tr = location(base_ind + 1,bottomleft_xy + np.array([X_D,X_D]),'intersection')
    x = [x_br,x_tr]

    h_bl = location(base_ind + 2, bottomleft_xy + np.array([H_D,H_D]),'house')
    h_br = location(base_ind + 3, bottomleft_xy + np.array([X_D - H_D, H_D]),'house')
    h_tr = location(base_ind + 4, bottomleft_xy + np.array([X_D - H_D, X_D - H_D]),'house')
    h_tl = location(base_ind + 5, bottomleft_xy + np.array([H_D, X_D - H_D]),'house')
    h = [h_bl,h_br,h_tr,h_tl]

    c = ([
            [x_br.id, h_br.id],
            [x_tr.id, h_tr.id],
            [x_br.id, x_tr.id],
            ])

    n = len(x) + len(h)

    return x, h, c, n

def connectFromLeft(x_left,x_right,h_right):
    if len(x_left) == 4:
        return [[x_left[1].id,x_right[0].id]] + \
                [[x_left[2].id,x_right[1].id]] + \
                [[x_left[1].id,h_right[0].id]] + \
                [[x_left[2].id,h_right[3].id]]

    elif len(x_left) == 2:
        return [[x_left[0].id,x_right[0].id]] + \
        [[x_left[1].id,x_right[1].id]] + \
        [[x_left[0].id,h_right[0].id]] + \
        [[x_left[1].id,h_right[3].id]]


def build_env():
    x_a, h_a, c_a, n_a = build_block(0, np.array([0,0]))
    x_b,h_b, c_b, n_b = build_leftconnected_block(n_a, np.array([X_D,0]))    
    x_c,h_c, c_c, n_c = build_leftconnected_block(n_a + n_b, np.array([2 * X_D,0]))   

    i_s = n_a + n_b + n_c + np.array([0,1,2])
    station2 = location(int(i_s[0]), x_a[2].xy + np.array([0,0.2*X_D]),'station')
    station9 = location(int(i_s[1]), x_b[1].xy + np.array([0,0.2*X_D]),'station')
    station15 = location(int(i_s[2]), x_c[1].xy + np.array([0,0.2*X_D]),'station')
    stations = [station2, station9, station15]
    n_s = 3

    dock = location(n_a + n_b + n_c + n_s, np.array([1.5*X_D,1.5*X_D]),'dock')

    locations = sorted(x_a + h_a + \
                        x_b + h_b + \
                        x_c + h_c + stations +[dock])
    connectivityList = c_a + \
                        connectFromLeft(x_a,x_b,h_b) + c_b + \
                        connectFromLeft(x_b,x_c,h_c) + c_c + \
                        [[2,station2.id]] + [[9, station9.id]] + [[15, station15.id]]

    p0 = package(0,locations[4].id,'location',locations[6].id,100, locations[4].xy)
    p1 = package(1,locations[6].id,'location',locations[4].id,100, locations[6].xy)
    p2 = package(2,locations[11].id,'location',locations[18].id,100, locations[11].xy)
    p3 = package(3,locations[19].id,'location',locations[10].id,100, locations[19].xy)
    p4 = package(4,locations[16].id,'location',locations[5].id,100, locations[16].xy)
    p5 = package(5,locations[13].id,'location',locations[19].id,100, locations[13].xy)
    packages = [p0,p1,p2,p3,p4,p5]

    env = enviorment(locations,connectivityList , packages)
    return env

#buld enviorment
env = build_env()

#spawn robots
station = 20
x0 = env.locations[station].xy[0]
y0 = env.locations[station].xy[1]
theta0 = np.pi/2
r0 = robot(gtsam.Pose2(x0,y0,theta0),0)
r0.last_location = station
r0.goal_location = station

station = 21
x0 = env.locations[station].xy[0]
y0 = env.locations[station].xy[1]
theta0 = np.pi/2
r1 = robot(gtsam.Pose2(x0,y0,theta0),1)
r1.last_location = station
r1.goal_location = station

station = 22
x0 = env.locations[station].xy[0]
y0 = env.locations[station].xy[1]
theta0 = np.pi/2
r2 = robot(gtsam.Pose2(x0,y0,theta0),2)
r2.last_location = station
r2.goal_location = station

r = [r0,r1,r2]
Nrobots = len(r)

#ask for plan
planner = robot_planner()
print('starting to plan... this may take a mintue')
plan = planner.create_plan(env,r)
print('finished planning')
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
    moviewriter.setup(fig,MOVIE_FILENAME,dpi = 100)

#roll simulation
current_action_indicies = [0 for _ in range(len(r))]
while True:
    current_actions = [wait(id) for id in range(len(r))]
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
    current_actions_status = np.zeros(Nrobots) #just to initialize
    while status is False:
        
        for ri, s in enumerate(current_actions_status):
            if not s:
                action = current_actions[ri]
                current_actions_status[ri] = r[action.robot_id].act(action, env)
        
        status = bool(np.all(current_actions_status))

        #update plot        
        for ri in r:
            ri.plot(ax)
            for p in ri.owned_packages:
                p.plot(ax)
        for a, s in zip(current_actions, current_actions_status):
            if s and type(a) is drop:
                a.p.plot(ax)

        if MOVIE:
            moviewriter.grab_frame()
        plt.pause(0.1)

#dont close window in the end
ax.set_title('finished!')
if MOVIE:
    moviewriter.finish()
plt.ioff()
plt.show()
