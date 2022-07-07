from unified_planning.model.metrics import MaximizeExpressionOnFinalState
from unified_planning.shortcuts import *
from unified_planning.io.pddl_writer import PDDLWriter
from maildelivery import optic_wrapper

location = UserType('location')
robot = UserType('robot')

is_connected = Fluent('is_connected', BoolType(), l_from = location, l_to = location)
distance = Fluent('distance', IntType(), l_from = location, l_to = location)
robot_at = Fluent('robot_at', BoolType(), r = robot, l = location)
charge = Fluent('charge', IntType(0,100), r = robot)


dist2chargeUse  = lambda dist: 2 * dist

move = DurativeAction('move',  r = robot, l_from = location, l_to = location)
r = move.parameter('r')
l_from = move.parameter('l_from')
l_to = move.parameter('l_to')
move.set_fixed_duration(1)
move.add_condition(StartTiming(), is_connected(l_from, l_to))
move.add_condition(StartTiming(), robot_at(r, l_from))
move.add_condition(StartTiming(),GE(charge(r),dist2chargeUse(distance(l_from,l_to))))
move.add_effect(StartTiming(),robot_at(r, l_from), False)
move.add_effect(EndTiming(),robot_at(r, l_to), True)
move.add_decrease_effect(EndTiming(),charge(r),dist2chargeUse(distance(l_from,l_to)))

problem = Problem('rovers2')
problem.add_action(move)
problem.add_fluent(robot_at, default_initial_value = False)
problem.add_fluent(is_connected, default_initial_value = False)
problem.add_fluent(charge, default_initial_value = int(0))
problem.add_fluent(distance, default_initial_value = int(100)) #some absuard number

#objects of problem
locations = [Object(f"l{i}", location) for i in range(4)]
deliverybot = Object("r0",robot)
problem.add_objects(locations + [deliverybot])
#initial values of grid locations
problem.set_initial_value(is_connected(locations[0],locations[1]),True)
problem.set_initial_value(is_connected(locations[1],locations[2]),True)
problem.set_initial_value(is_connected(locations[2],locations[3]),True)
problem.set_initial_value(distance(locations[0],locations[1]),1)
problem.set_initial_value(distance(locations[1],locations[2]),2)
problem.set_initial_value(distance(locations[2],locations[3]),3)
#robot at start
problem.set_initial_value(robot_at(deliverybot,locations[0]),True)
#fuel at start
problem.set_initial_value(charge(deliverybot), (dist2chargeUse(20)))
#goal
problem.add_goal(robot_at(deliverybot,locations[2]))
#minimization
problem.add_quality_metric(MaximizeExpressionOnFinalState(charge(deliverybot)))

w = PDDLWriter(problem)
with open(optic_wrapper.DOMAIN_PATH, 'w') as f:
    print(w.get_domain(), file = f)
with open(optic_wrapper.PROBLEM_PATH, 'w') as f:
    print(w.get_problem(), file = f)
print('copied pddls')

optic_wrapper.remove_problem_lines(1)
optic_wrapper.add_problem_lines(['(:metric maximize(charge r0))'])

returncode = optic_wrapper.run_optic()
if returncode == 0:
    execution_times, actions, durations = optic_wrapper.get_plan()
    print(actions)
