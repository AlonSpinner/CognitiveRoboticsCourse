import unified_planning
from unified_planning.shortcuts import UserType, BoolType,\
        Fluent, DurativeAction, Problem, Object, OneshotPlanner,\
        StartTiming, EndTiming, Or, Not
unified_planning.shortcuts.get_env().credits_stream = None #removes the printing planners credits 

# --------------------------------------------- Define problem variables and actions
MOVE_TIME = 1

#problem types ~ objects
location = UserType('location')
robot = UserType('robot')

#problem variables that are changed by actions on objects (no floats please, they cause problems to solvers)
robot_at = Fluent('robot_at', BoolType(), r = robot, l = location)
is_connected = Fluent('is_connected', BoolType(), l_from = location, l_to = location)
is_occupied = Fluent('is_occupied', BoolType(), l = location)

#actions
move = DurativeAction('move',  r = robot, l_from = location, l_to = location)
r = move.parameter('r')
l_from = move.parameter('l_from')
l_to = move.parameter('l_to')
move.set_fixed_duration(MOVE_TIME)
move.add_condition(StartTiming(),is_connected(l_from, l_to))
move.add_condition(StartTiming(),robot_at(r, l_from))
move.add_condition(StartTiming(),Not(is_occupied(l_to))) #at end, l_to is free
move.add_effect(EndTiming(),robot_at(r, l_from), False)
move.add_effect(EndTiming(),is_occupied(l_from), False)
move.add_effect(EndTiming(),robot_at(r, l_to), True)
move.add_effect(EndTiming(),is_occupied(l_to), True)


problem = Problem('maildelivery')
problem.add_action(move)
problem.add_fluent(robot_at, default_initial_value = False)
problem.add_fluent(is_connected, default_initial_value = False)
problem.add_fluent(is_occupied, default_initial_value = False)

# --------------------------------------------- Set specific values
#objects of problem          
locations = [Object(f"l{i}", location) for i in range(25)]                   
deliverybots = [Object(f"r{i}",robot) for i in range(2)]
problem.add_objects(locations + deliverybots)
#initial values of grid locations
def lin2mat(k,n):
        # n - number of cols
        j = k // n
        i = k - j * n
        return i,j

def mat2lin(i,j,m):
        # m - number of rows
        k = i + j * m
        return k
m = 5; n = 5
for i in range(m):
        for j in range(n):
                k = mat2lin(i,j,m)
                if i > 0:
                        ktop = mat2lin(i-1,j,m)
                        problem.set_initial_value(is_connected(locations[k],locations[ktop]),True)
                if i < m - 1:
                        kbot = mat2lin(i+1,j,m)
                        problem.set_initial_value(is_connected(locations[k],locations[kbot]),True)
                if j > 0:
                        kleft = mat2lin(i,j-1,m)
                        problem.set_initial_value(is_connected(locations[k],locations[kleft]),True)
                if j < n - 1:
                        kright = mat2lin(i,j+1,m)
                        problem.set_initial_value(is_connected(locations[k],locations[kright]),True)
# place bots
problem.set_initial_value(robot_at(deliverybots[0],locations[mat2lin(0,0,m)]),True)
problem.set_initial_value(robot_at(deliverybots[1],locations[mat2lin(2,0,m)]),True)
problem.set_initial_value(is_occupied(locations[mat2lin(0,0,m)]),True)
problem.set_initial_value(is_occupied(locations[mat2lin(2,0,m)]),True)

#place obstacles
problem.set_initial_value(is_occupied(locations[mat2lin(0,1,m)]),True)
problem.set_initial_value(is_occupied(locations[mat2lin(1,1,m)]),True)
problem.set_initial_value(is_occupied(locations[mat2lin(1,2,m)]),True)
problem.set_initial_value(is_occupied(locations[mat2lin(2,1,m)]),True)
problem.set_initial_value(is_occupied(locations[mat2lin(2,2,m)]),True)
problem.set_initial_value(is_occupied(locations[mat2lin(3,3,m)]),True)

#goal
problem.add_timed_goal(StartTiming(100),robot_at(deliverybots[0],locations[mat2lin(4,4,m)]))
problem.add_timed_goal(StartTiming(100),robot_at(deliverybots[1],locations[mat2lin(2,4,m)]))
# problem.add_goal(robot_at(deliverybots[0],locations[mat2lin(4,4,m)]))
# problem.add_goal(robot_at(deliverybots[1],locations[mat2lin(2,4,m)]))

print(problem.kind)

import time
s = time.time()
# with OneshotPlanner(name='tamer') as planner:
#     result = planner.solve(problem)
    
with OneshotPlanner(problem_kind = problem.kind) as planner:
    result = planner.solve(problem)

if result.plan is not None:
        for action in result.plan._actions:
                print(action)
else:
        print('unable to produce a plan')
e = time.time()

print(e - s)