from maildelivery.world import enviorment,location, package
from maildelivery.agents import robot, drone, wait
from maildelivery.brains.brains_bots_and_drones import robot_planner
from maildelivery.brains.plan_parser import full_plan_2_per_agent, parse_plan
from maildelivery.geometry import pose2

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
from matplotlib.offsetbox import AnchoredText
import os

DT = 0.001 #[s]
V_ROBOT = 8.0 #[m/s]
V_DRONE = 10.0 #[m/s]
MOVIE = True
dir_path = os.path.dirname(__file__)
MOVIE_FILENAME = os.path.join(dir_path,'16_movie.gif')
X_D = 3.0
H_D = 1.0
f_dist2charge = lambda dist: 2 * dist
f_charge2time = lambda missing_charge: missing_charge/100

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
    n = 0
    x_a, h_a, c_a, n_a = build_block(n, np.array([0,0]))
    n += n_a
    x_b,h_b, c_b, n_b = build_leftconnected_block(n, np.array([X_D,0]))    
    n += n_b
    x_c,h_c, c_c, n_c = build_leftconnected_block(n, np.array([2 * X_D,0]))
    n += n_c

    station0 = location(n, x_a[2].xy + np.array([0,H_D]),'station')
    n += 1
    station1 = location(n, x_b[1].xy + np.array([0,H_D]),'station')
    n += 1
    station2 = location(n, x_c[1].xy + np.array([0,H_D]),'station')
    n += 1
    stations = [station0, station1, station2]

    dock0 = location(n, np.array(x_b[0].xy + np.array([0,-H_D])),'dock')
    n += 1
    dock1 = location(n, np.array(station0.xy + np.array([0,H_D])),'dock')
    n += 1
    docks = [dock0, dock1]


    x_d,h_d, c_d, n_d = build_block(n, dock1.xy + np.array([H_D,0]))
    n += n_d

    locations = sorted(x_a + h_a + \
                        x_b + h_b + \
                        x_c + h_c + stations +docks \
                        + x_d + h_d)
    connectivityList = c_a + \
                        connectFromLeft(x_a,x_b,h_b) + c_b + \
                        connectFromLeft(x_b,x_c,h_c) + c_c + \
                        [[2,station0.id]] + [[9, station1.id]] + [[15, station2.id]] + \
                            [[8,dock0.id]] + [[x_d[0].id, dock1.id]] + c_d

    p0 = package(0,locations[4].id,'location',locations[6].id,100, locations[4].xy)
    p1 = package(1,locations[6].id,'location',locations[4].id,100, locations[6].xy)
    p2 = package(2,locations[11].id,'location',locations[18].id,100, locations[11].xy)
    p3 = package(3,locations[19].id,'location',locations[10].id,100, locations[19].xy)
    p4 = package(4,locations[16].id,'location',locations[5].id,100, locations[16].xy)
    p5 = package(5,locations[13].id,'location',locations[19].id,100, locations[13].xy)
    p6 = package(6,locations[29].id,'location',locations[30].id,100, locations[29].xy)
    p7 = package(7,locations[32].id,'location',locations[31].id,100, locations[32].xy)
    packages = [p0,p1,p2,p3,p4,p5,p6,p7]

    env = enviorment(locations,connectivityList , packages)
    return env

#buld enviorment
env = build_env()

#spawn robots
station = 20
x0 = env.locations[station].xy[0]
y0 = env.locations[station].xy[1]
theta0 = np.pi/2
r0 = robot(0, pose2(x0,y0,theta0), DT)
r0.last_location = station
r0.goal_location = station
r0.velocity = V_ROBOT
r0.f_dist2charge = f_dist2charge
r0.f_charge2time = f_charge2time
r0.charge = 100.0

station = 21
x0 = env.locations[station].xy[0]
y0 = env.locations[station].xy[1]
theta0 = np.pi/2
r1 = robot(1, pose2(x0,y0,theta0), DT)
r1.last_location = station
r1.goal_location = station
r1.velocity = V_ROBOT
r1.f_dist2charge = f_dist2charge
r1.f_charge2time = f_charge2time
r1.charge = 50.0

station = 22
x0 = env.locations[station].xy[0]
y0 = env.locations[station].xy[1]
theta0 = np.pi/2
r2 = robot(2, pose2(x0,y0,theta0), DT)
r2.last_location = station
r2.goal_location = station
r2.velocity = V_ROBOT
r2.f_dist2charge = f_dist2charge
r2.f_charge2time = f_charge2time
r2.charge = 50.0

drone_init_location = 31
x0 = env.locations[drone_init_location].xy[0]
y0 = env.locations[drone_init_location].xy[1]
theta0 = np.pi/2
d0 = drone(3, pose2(x0,y0,theta0), DT)
d0.last_location = drone_init_location
d0.velocity = V_DRONE

r = [r0,r1,r2]
d = [d0]
a = r + d #important that it is r + d as it is assumed all over the code
Nagents = len(a)

#ask for plan
planner = robot_planner()
planner.create_problem(env,r,d)

execution_times, actions, durations = planner.solve(engine_name = 'lpg', minimize_makespan = True)
actions = parse_plan(execution_times, actions, durations,env, a)
a_execution_times, a_actions, a_durations = full_plan_2_per_agent(execution_times, actions, durations, a)


print(f"plan should end at {max([action.time_end for action in actions])}")

#plot initial state
plt.ion()
fig , ax = env.plot()
def animate():
    if 'anchored_text' in locals(): anchored_text.remove()
    anchored_text = AnchoredText(f"t = {t:2.2f}[s]", loc=2)
    ax.add_artist(anchored_text)
    [ri.plot(ax) for ri in r]
    [di.plot(ax) for di in d]
    [p.plot(ax) for p in env.packages]
    plt.pause(0.01)
#ready movie
if MOVIE:
    moviewriter = PillowWriter(fps = 5)
    moviewriter.setup(fig,MOVIE_FILENAME,dpi = 100)

#roll simulation
t = 0
plotCounter = 0

a_next_actions_indicies = [0 for _ in range(Nagents)]
a_done = [False for _ in range(Nagents)]
while True:

    for i,ai in enumerate(a):
        #go do next action
        if a_done[i] == False and \
            type(ai.current_action) == wait and \
                t >= a_execution_times[i][a_next_actions_indicies[i]]:
            
            ai.current_action =  a_actions[i][a_next_actions_indicies[i]]
            print(f"t = {t:2.2f}  :",ai.current_action)
            a_next_actions_indicies[i] += 1

        if ai.act(ai.current_action, env): #do action, and if its finished, start waiting allowing accepting new actions
            ai.current_action = wait(ai)

    #update plot        
    if plotCounter % 100 == 0:
        animate()
        if MOVIE:
            moviewriter.grab_frame()
    plotCounter +=1

    t += DT

    for i in range(Nagents):
        a_done[i] = a_next_actions_indicies[i] == len(a_actions[i]) and type(a[i].current_action) == wait

    if all(a_done):
        animate()
        if MOVIE:
            moviewriter.grab_frame()
        break
    
#dont close window in the end
ax.set_title('finished!')
if MOVIE:
    moviewriter.finish()

for ri in r:
    print(f"robot {ri.id} has {ri.charge:2.2f}/{ri.max_charge} charge left")

plt.ioff()
plt.show()

