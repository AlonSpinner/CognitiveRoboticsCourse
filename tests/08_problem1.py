from maildelivery.datatypes import beacon, landmark, move, package, pickup, drop

from maildelivery.map import Map
from maildelivery.robot import robot
import maildelivery.plotting as plotting
import numpy as np
import matplotlib.pyplot as plt

import gtsam

import unified_planning
from unified_planning.shortcuts import UserType, BoolType,\
        Fluent, InstantaneousAction, Problem, Object, OneshotPlanner, Or, Not
unified_planning.shortcuts.get_env().credits_stream = None #removes the printing planners credits 

def createMap():
    docks = [landmark(0,np.array([0,0]),'dock')]

    x1 = landmark(1,np.array([1,0]),'intersection')
    x2 = landmark(2,np.array([2,0]),'intersection')
    x3 = landmark(3,np.array([1,1]),'intersection')
    x4 = landmark(4,np.array([2,1]),'intersection')
    intersections = [x1,x2,x3,x4]

    h5 = landmark(5,np.array([1,2]),'house')
    h6 = landmark(6,np.array([2,2]),'house')
    houses = [h5,h6]

    landmarks = houses + docks + intersections
    connectivityList = [[0,1],[1,2],[2,4],[1,3],[3,5],[4,6]]

    p0 = package(0,5,6,100)
    p1 = package(1,6,5,100)
    packages = [p0,p1]

    m = Map([], landmarks, connectivityList, packages)
    return m

def createPlan(env : Map, robots : list[robot]):
    #planner has to use different name as to not to confuse with names of modules
    location = UserType('location') # <-> landmark
    deliverybot = UserType('deliveybot') #<-> robot
    mail = UserType('mail') #<-> package

    #problem variables that are changed by actions on objects (no floats please, they cause problems to solvers)
    deliverybot_at = Fluent('deliverybot_at', BoolType(), r = deliverybot, l = location)
    is_connected = Fluent('is_connected', BoolType(), l_from = location, l_to = location)
    is_occupied = Fluent('is_occupied', BoolType(), l = location)
    deliveybot_has_mail = Fluent('deliveybot_has_mail', BoolType(), p = mail, r = deliverybot)
    location_has_mail = Fluent('location_has_mail', BoolType(), p = mail, l = location)

    _move = InstantaneousAction('move',  r = deliverybot, l_from = location, l_to = location)
    r = _move.parameter('r')
    l_from = _move.parameter('l_from')
    l_to = _move.parameter('l_to')
    _move.add_precondition(Or(is_connected(l_from, l_to), \
                                        is_connected(l_to, l_from)))
    _move.add_precondition(deliverybot_at(r, l_from))
    _move.add_precondition(Not(is_occupied(l_to))) #at end, l_to is free
    _move.add_effect(deliverybot_at(r, l_from), False)
    _move.add_effect(is_occupied(l_from), False)
    _move.add_effect(deliverybot_at(r, l_to), True)
    _move.add_effect(is_occupied(l_to), True)

    _pickup = InstantaneousAction('pickup', m = mail, r = deliverybot, l = location)
    m = _pickup.parameter('m')
    r = _pickup.parameter('r')
    l = _pickup.parameter('l')
    _pickup.add_precondition(deliverybot_at(r, l))
    _pickup.add_precondition(location_has_mail(m, l))
    _pickup.add_effect(location_has_mail(m, l), False)
    _pickup.add_effect(deliveybot_has_mail(m, r), True)

    _drop = InstantaneousAction('drop', m = mail, r = deliverybot, l = location)
    m = _drop.parameter('m')
    r = _drop.parameter('r')
    l = _drop.parameter('l')
    _drop.add_precondition(deliverybot_at(r, l))
    _drop.add_precondition(deliveybot_has_mail(m, r))
    _drop.add_effect(deliveybot_has_mail(m, r), False)
    _drop.add_effect(location_has_mail(m, l), True)

    problem = Problem('maildelivery')
    problem.add_action(_move)
    problem.add_action(_pickup)
    problem.add_action(_drop)
    problem.add_fluent(deliverybot_at, default_initial_value = False)
    problem.add_fluent(is_connected, default_initial_value = False)
    problem.add_fluent(is_occupied, default_initial_value = False)
    problem.add_fluent(deliveybot_has_mail, default_initial_value = False)
    problem.add_fluent(location_has_mail, default_initial_value = False)

    #objects of problem
    locations = [Object(f"l{id}", location) for id in [lm.id for lm in env.landmarks]]
    deliverybots = [Object(f"r{id}", deliverybot) for id in [bot.id for bot in robots]]
    notes = [Object(f"m{id}", mail) for id in [p.id for p in env.packages]]
    problem.add_objects(locations + deliverybots + notes)

    for c in env.connectivityList:
        problem.set_initial_value(is_connected(
                                    locations[c[0]],
                                    locations[c[1]]),
                                    True)
    # robot at start
    for r in robots:
        problem.set_initial_value(deliverybot_at(
                                                deliverybots[r.id],
                                                locations[r.last_landmark]),
                                                True)
        problem.set_initial_value(is_occupied(
                                             locations[r.last_landmark]),
                                             True) 
    #place packages
    for p in env.packages:
        if p.id < 1000:
            problem.set_initial_value(location_has_mail(
                                                        notes[p.id],
                                                        locations[p.owner]),
                                                        True)
        else:
            problem.set_initial_value(deliveybot_has_mail(
                                            notes[p.id],
                                            deliverybots[p.owner-1000]),
                                            True)
    #goal
    for p in env.packages:
        problem.add_goal(location_has_mail(notes[p.id],locations[p.goal]))

    with OneshotPlanner(problem_kind = problem.kind) as planner:
        result = planner.solve(problem)
    
    return result.plan

m = createMap()
r = robot(gtsam.Pose2(m.landmarks[0].xy[0],m.landmarks[0].xy[1],landmark.angle(m.landmarks[0],m.landmarks[1])),0)
plan = createPlan(m,[r])

def parse_actions(actions):
    parsed_actions = []
    for a in actions:
        if a.action.name == 'move':
            parsed_actions.append(move(
                int(str(a.actual_parameters[0])[1:]), #robot id
                int(str(a.actual_parameters[1])[1:]), #landmark_from id
                int(str(a.actual_parameters[2])[1:]) #landmark_to id
                )) 
        elif a.action.name == 'drop':
            parsed_actions.append(drop(
                int(str(a.actual_parameters[1])[1:]), #robot id
                int(str(a.actual_parameters[0])[1:]), #package id
                int(str(a.actual_parameters[2])[1:]) #landmark id
                )) 
        elif a.action.name == 'pickup':
            parsed_actions.append(pickup(
                int(str(a.actual_parameters[1])[1:]), #robot id
                int(str(a.actual_parameters[0])[1:]), #package id
                int(str(a.actual_parameters[2])[1:]) #landmark id
                ))
    return parse_actions

_, ax = plotting.spawnWorld()
m.plot(ax)
# graphics_r = r.plot(ax)

# odom = gtsam.Pose2(0.4,0,0)
# cmd = move(odom)

# plt.ion():
    # for _ in range(len(plan.actions)):
    #     r.move(cmd)

    #     graphics_r.remove()
    #     graphics_r = r.plot(ax)
    #     plt.pause(0.5)

plt.show()



