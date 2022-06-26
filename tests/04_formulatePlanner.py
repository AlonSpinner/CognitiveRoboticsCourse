import unified_planning
from unified_planning.shortcuts import UserType, BoolType, RealType, \
        Fluent, DurativeAction, InstantaneousAction, SimulatedEffect, Problem, Object,\
        StartTiming, EndTiming, GlobalStartTiming, GlobalEndTiming, Real, GE, Or, TimeInterval
    
from unified_planning.shortcuts import *
unified_planning.shortcuts.get_env().credits_stream = None #removes the printing planners credits 

        
from unified_planning.io.pddl_writer import PDDLWriter
from unified_planning.io.pddl_reader import PDDLReader
# --------------------------------------------- Define problem variables and actions

#constants
FULLTANK = 100
LOCATION_DISTANCE = 1.0
MOVE_TIME = 1.0
PICKUP_TIME = 1.0
DROP_TIME = 1.0
FILL_TIME = 1.0


#problem types ~ objectsv
location = UserType('location')
robot = UserType('robot')
package = UserType('package')

#problem variables that are changed by actions on objects
robot_at = Fluent('robot_at', BoolType(), r = robot, l = location)
is_connected = Fluent('is_connected', BoolType(), l_from = location, l_to = location)
is_occupied = Fluent('is_occupied', BoolType(), l = location)
distance = Fluent('distance', RealType(), l_from = location, l_to = location)
delivery_time = Fluent('delivery_time', RealType(), p = package)
robot_has_package = Fluent('robot_has_package', BoolType(), p = package, r = robot)
location_has_package = Fluent('location_has_package', BoolType(), p = package, l = location)
location_is_pump = Fluent('location_is_pump', BoolType(), l = location)
fuel = Fluent('fuel', IntType(), r = robot)

#actions
move = DurativeAction('move',  r = robot, l_from = location, l_to = location)
r = move.parameter('r')
l_from = move.parameter('l_from')
l_to = move.parameter('l_to')
move.set_fixed_duration(MOVE_TIME)
move.add_condition(StartTiming(),Or(is_connected(l_from, l_to), \
                                    is_connected(l_to, l_from)))
move.add_condition(StartTiming(),GE(fuel(r),LOCATION_DISTANCE))
move.add_condition(StartTiming(), robot_at(r, l_from))
move.add_condition(EndTiming(), is_occupied(l_to))
move.add_effect(StartTiming(), robot_at(r, l_from), False)
move.add_effect(StartTiming(), is_occupied(l_from), False)
move.add_effect(EndTiming(), robot_at(r, l_to), True)
move.add_effect(StartTiming(), is_occupied(l_to), True)
def decrease_fuel_fun(problem, state, actual_params):
    fuelCurrent = state.get_value(fuel(actual_params.get(r))).constant_value()
    return [Int(fuelCurrent-1)]
move.set_simulated_effect(StartTiming(),SimulatedEffect([fuel(r)], decrease_fuel_fun))
# move.add_decrease_effect(EndTiming(),fuel(r),distance(l_from, l_to))

pickup = DurativeAction('pickup', p = package, r = robot, l = location)
p = pickup.parameter('p')
r = pickup.parameter('r')
l = pickup.parameter('l')
pickup.set_fixed_duration(PICKUP_TIME)
pickup.add_condition(StartTiming(),robot_at(r, l))
pickup.add_condition(StartTiming(),location_has_package(p, l))
pickup.add_effect(StartTiming(),location_has_package(p, l), False)
pickup.add_effect(EndTiming(),robot_has_package(p, r), True)

drop = DurativeAction('drop', p = package, r = robot, l = location)
p = drop.parameter('p')
r = drop.parameter('r')
l = drop.parameter('l')
drop.set_fixed_duration(DROP_TIME)
drop.add_condition(StartTiming(),robot_at(r, l))
drop.add_condition(StartTiming(),robot_has_package(p, r))
drop.add_effect(StartTiming(),robot_has_package(p, r), False)
drop.add_effect(EndTiming(),location_has_package(p, l), True)

fillfuel = DurativeAction('fllfuel', r = robot, l = location)
r = fillfuel.parameter('r')
l = fillfuel.parameter('l')
fillfuel.set_fixed_duration(FILL_TIME)
fillfuel.add_condition(StartTiming(), location_is_pump(l))
fillfuel.add_condition(StartTiming(), robot_at(r, l))
fillfuel.add_effect(EndTiming(), fuel(r), FULLTANK)

problem = Problem('maildelivery')
problem.add_action(move)
problem.add_action(pickup)
problem.add_action(drop)
problem.add_action(fillfuel)
problem.add_fluent(robot_at, default_initial_value = False)
problem.add_fluent(is_connected, default_initial_value = False)
problem.add_fluent(is_occupied, default_initial_value = False)
problem.add_fluent(distance, default_initial_value = 1.0)
problem.add_fluent(delivery_time, default_initial_value = 100000.0)
problem.add_fluent(robot_has_package, default_initial_value = False)
problem.add_fluent(location_has_package, default_initial_value = False)
problem.add_fluent(location_is_pump, default_initial_value = False)
problem.add_fluent(fuel, default_initial_value = 0)

# --------------------------------------------- Set specific values
#objects of problem
locations = [Object(f"l{i}", location) for i in range(4)]
deliverybot = Object("r",robot)
note = Object("p",package)
problem.add_objects(locations + [deliverybot] + [note])
# #initial values of grid locations
problem.set_initial_value(is_connected(locations[0],locations[1]),True)
problem.set_initial_value(is_connected(locations[1],locations[2]),True)
problem.set_initial_value(is_connected(locations[2],locations[3]),True)
problem.set_initial_value(distance(locations[0],locations[1]),1.0)
problem.set_initial_value(distance(locations[1],locations[2]),1.0)
problem.set_initial_value(distance(locations[2],locations[3]),1.0)
problem.set_initial_value(distance(locations[3],locations[2]),1.0)
#connect pump to grid
problem.set_initial_value(location_is_pump(locations[0]),True)
#robot at start
problem.set_initial_value(robot_at(deliverybot,locations[0]),True)
problem.set_initial_value(is_occupied(locations[0]),True)
#connect houses to grid and place package in house 0
problem.set_initial_value(location_has_package(note,locations[3]),True)
#fuel at start
problem.set_initial_value(fuel(deliverybot),1)
#goal
problem.add_goal(location_has_package(note,locations[2]))
# problem.add_timed_goal(StartTiming(150.0), location_has_package(note,houses[1]))

print(problem.kind)
# with OneshotPlanner(names=['tamer', 'tamer'],
#                     params=[{'heuristic': 'hadd'}, {'heuristic': 'hmax'}]) as planner:
#     plan = planner.solve(problem).plan

# print(problem)

with OneshotPlanner(problem_kind=problem.kind) as planner:
    result = planner.solve(problem)

if result.plan is not None:
        for action in result.plan.timed_actions:
                print(action[1])
else:
        print('unable to produce plan')
# t = 5
