from maildelivery.world import enviorment,location, package
from maildelivery.agents import robot, drop, wait
from maildelivery.brains.brains_bots_simple import robot_planner
from maildelivery.brains.plan_parser import full_plan_2_per_agent, parse_plan
from maildelivery.geometry import pose2

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.offsetbox import AnchoredText


DT = 0.0001 #[s]
V = 2.0 #[m/s]

def build_env():

    h0 = location(0,np.array([0,0]),'house')
    h1 = location(1,np.array([1,0]),'house')
    h2 = location(2,np.array([1,1]),'house')
    h3 = location(3,np.array([0,1]),'house')
    houses = [h0,h1,h2,h3]

    landmarks = houses

    connectivityList = [[0,1],[0,2],[1,2]]#[2,3],[3,0]]

    p0 = package(0,0,'location',2,100,landmarks[0].xy)
    p1 = package(1,2,'location',0,100,landmarks[2].xy)
    packages = [p0,p1]

    env = enviorment(landmarks, connectivityList, packages)
    return env

#buld enviorment
env = build_env()

#spawn robots
l0 = 0
x0 = env.locations[l0].xy[0]
y0 = env.locations[l0].xy[1]
theta0 = np.pi/2
r0 = robot(0,pose2(x0,y0,theta0), DT)
r0.last_location = l0
r0.goal_location = 2
r0.velocity = V

l0 = 2
x0 = env.locations[l0].xy[0]
y0 = env.locations[l0].xy[1]
theta0 = np.pi/2
r1 = robot(1,pose2(x0,y0,theta0), DT)
r1.last_location = l0
r1.goal_location = 0
r1.velocity = V

r = [r0,r1]
a = r
Nagents = len(a)

#ask for plan
planner = robot_planner()
planner.create_problem(env,a)

execution_times, actions, durations = planner.solve(engine_name = 'lpg', minimize_makespan = True, lpg_n = 3)
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
    if plotCounter % 1000 == 0:
        animate()
    plotCounter +=1

    t += DT

    for i in range(Nagents):
        a_done[i] = a_next_actions_indicies[i] == len(a_actions[i]) and type(a_current_actions[i]) == wait

    if all(a_done):
        animate()
        break
    
#dont close window in the end
ax.set_title('finished!')

for ri in r:
    print(f"robot {ri.id} has {ri.charge:2.2f}/{ri.max_charge} charge left")

plt.ioff()
plt.show()