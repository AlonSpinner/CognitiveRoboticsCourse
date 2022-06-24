import unified_planning
from unified_planning.shortcuts import UserType, BoolType, RealType, StartTiming, EndTiming, GE
from unified_planning.io.pddl_writer import PDDLWriter
from unified_planning.io.pddl_reader import PDDLReader

location = UserType('location')
robot = UserType('robot')
package = UserType('package')

robot_at = unified_planning.model.Fluent('robot_at', BoolType(), r = robot, l = location)
distance = unified_planning.model.Fluent('connected', RealType(), l_from = location, l_to = location)
robot_has_package = unified_planning.model.Fluent('robot_has_package', BoolType(), p = package, r = robot)
location_has_package = unified_planning.model.Fluent('location_has_package', BoolType(), p = package, l = location)

move = unified_planning.model.DurativeAction('move',  r = robot, l_from = location, l_to = location)
r = move.parameter('r')
l_from = move.parameter('l_from')
l_to = move.parameter('l_to')
move.add_condition(StartTiming(),GE(distance(l_from, l_to),0))
move.add_condition(StartTiming(),robot_at(r, l_from))
move.add_effect(StartTiming(),robot_at(r, l_from), False)
move.add_effect(EndTiming(),robot_at(r, l_to), True)

pickup = unified_planning.model.InstantaneousAction('pickup', p = package, r = robot, l = location)
p = pickup.parameter('p')
r = pickup.parameter('r')
l = pickup.parameter('l')
pickup.add_precondition(robot_at(r, l))
pickup.add_precondition(location_has_package(p, l))
pickup.add_effect(location_has_package(p, l), False)
pickup.add_effect(robot_has_package(p, r), True)

drop = unified_planning.model.InstantaneousAction('drop', p = package, r = robot, l = location)
p = drop.parameter('p')
r = drop.parameter('r')
l = drop.parameter('l')
drop.add_precondition(robot_at(r, l))
drop.add_precondition(location_has_package(p, l))
drop.add_effect(robot_has_package(p, r), False)
drop.add_effect(location_has_package(p, l), True)

problem = unified_planning.model.Problem('maildelivery')
problem.add_action(move)
problem.add_action(pickup)
problem.add_action(drop)

print(problem)

# NLOC = 10
# locations = [unified_planning.model.Object('l%s' % i, Location) for i in range(NLOC)]
# problem.add_objects(locations)