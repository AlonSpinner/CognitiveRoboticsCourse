from maildelivery.world import enviorment,location, package
from maildelivery.agents import robot, wait
from maildelivery.brains.brains_bots_simple import robot_planner
from maildelivery.brains.plan_parser import parse_actions
from maildelivery.geometry import pose2

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.offsetbox import AnchoredText

DT = 0.001 #[s]
V = 2.0 #[m/s]

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
r = robot(0, pose2(x0,y0,theta0), DT)
r.last_location = 0
r.goal_location = 0
r.velocity = V

#ask for plan
planner = robot_planner()
planner.create_problem(env,[r])
execution_times, actions, durations = planner.solve(engine_name = 'lpg')
parsed_actions = parse_actions(actions, env, [r])

#plot initial state
plt.ion()
_, ax = env.plot()
def animate():
    if 'anchored_text' in locals(): anchored_text.remove()
    anchored_text = AnchoredText(f"t = {t:2.2f}[s]", loc=2)
    ax.add_artist(anchored_text)
    r.plot(ax)
    [p.plot(ax) for p in env.packages]
    plt.pause(0.01)
    
#roll simulation
t = 0
next_action_index = 0
plotCounter = 0
action = wait(r)
while True:
    
    #go do next action
    if type(action) == wait and t >= execution_times[next_action_index]:
        action = parsed_actions[next_action_index]
        print(f"t = {t:2.2f}  :",action)
        next_action_index += 1
        
        
    if r.act(action, env): #do action, and if its finished, start waiting allowing accepting new actions
        action = wait(r)
    
    #update plot        
    if plotCounter % 200 == 0:
        animate()
    plotCounter += 1
    
    t += DT

    if next_action_index == len(parsed_actions) and type(action) == wait:
        animate()
        break

#dont close window in the end
ax.set_title('finished!')
print(f"robot has {r.charge}/{r.max_charge} charge left")
plt.ioff()
plt.show()



