from maildelivery.world import enviorment,location, package
from maildelivery.agents import robot, wait
from maildelivery.brains.brains_bots_charge_added import robot_planner
from maildelivery.brains.plan_parser import full_plan_2_per_agent, parse_plan
from maildelivery.geometry import pose2

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
from matplotlib.offsetbox import AnchoredText
import os

DT = 0.001 #[s]
V = 2.0 #[m/s]

MOVIE = True
dir_path = os.path.dirname(__file__)
MOVIE_FILENAME = os.path.join(dir_path,'06_movie.gif')
f_dist2charge = lambda dist: 2 * dist
f_charge2time = lambda missing_charge: missing_charge/100
max_charge = 100.0

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
r0 = robot(0,pose2(x0,y0,theta0),DT)
r0.last_location = l0
r0.goal_location = l0
r0.velocity = V
r0.f_charge2time = f_charge2time
r0.max_charge = max_charge
r0.charge = 100.0

l0 = 6
x0 = env.locations[l0].xy[0]
y0 = env.locations[l0].xy[1]
theta0 = np.pi/2
r1 = robot(1,pose2(x0,y0,theta0),DT)
r1.last_location = l0
r1.goal_location = l0
r1.velocity = V
r1.f_charge2time = f_charge2time
r1.max_charge = max_charge
r1.charge = 100.0

r = [r0, r1]
a = r
Nagents = len(a)

#ask for plan
planner = robot_planner()
planner.create_problem(env,a)
planner.f_dist2charge = f_dist2charge
planner.f_charge2time = f_charge2time
planner.max_charge = max_charge

execution_times, actions, durations = planner.solve(engine_name = 'lpg', minimize_makespan = True, lpg_n = 1)
actions = parse_plan(execution_times, actions, durations,env, a)
a_execution_times, a_actions, a_durations = full_plan_2_per_agent(execution_times, actions, durations, a)


#plot initial state
plt.ion()
fig , ax = env.plot()
def animate():
    if 'anchored_text' in locals(): anchored_text.remove()
    anchored_text = AnchoredText(f"t = {t:2.2f}[s]", loc=2)
    ax.add_artist(anchored_text)
    [ri.plot(ax) for ri in r]
    [p.plot(ax) for p in env.packages]
    plt.pause(0.01)
#ready movie
if MOVIE:
    moviewriter = PillowWriter(fps = 5)
    moviewriter.setup(fig,MOVIE_FILENAME,dpi = 100)

#roll simulation
t = 0
plotCounter = 0

a_current_actions = [wait(ai) for ai in a]
a_next_actions_indicies = [0 for _ in range(Nagents)]
a_done = [False for _ in range(Nagents)]
while True:

    for i,ai in enumerate(a):
        #go do next action
        if a_done[i] == False and \
            type(a_current_actions[i]) == wait and \
                t >= a_execution_times[i][a_next_actions_indicies[i]]:
            a_current_actions[i] = a_actions[i][a_next_actions_indicies[i]]
            print(f"t = {t:2.2f}  :",a_current_actions[i])
            a_next_actions_indicies[i] += 1
             
        if ai.act(a_current_actions[i], env): #do action, and if its finished, start waiting allowing accepting new actions
            a_current_actions[i] = wait(ai)

    #update plot        
    if plotCounter % 50 == 0:
        animate()
        if MOVIE:
            moviewriter.grab_frame()
    plotCounter +=1

    t += DT

    for i in range(Nagents):
        a_done[i] = a_next_actions_indicies[i] == len(a_actions[i]) and type(a_current_actions[i]) == wait

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



