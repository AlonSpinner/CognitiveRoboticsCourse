from maildelivery.datatypes import beacon, landmark, move, package

from maildelivery.map import Map
from maildelivery.robot import robot
import maildelivery.plotting as plotting
import numpy as np
import matplotlib.pyplot as plt

import gtsam

import unified_planning
from unified_planning.shortcuts import UserType, BoolType, IntType, Int,\
        Fluent, InstantaneousAction, SimulatedEffect, Problem, Object, OneshotPlanner,\
        StartTiming, EndTiming,  GE, Or, Not
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

def createPlan(m : Map, deliverybots : list[robot]):
    location = UserType('location')
    deliveybot = UserType('deliveybot')
    mail = UserType('mail')

    #problem variables that are changed by actions on objects (no floats please, they cause problems to solvers)
    deliveybot_at = Fluent('deliveybot_at', BoolType(), r = deliveybot, l = location)
    is_connected = Fluent('is_connected', BoolType(), l_from = location, l_to = location)
    is_occupied = Fluent('is_occupied', BoolType(), l = location)
    deliveybot_has_mail = Fluent('deliveybot_has_mail', BoolType(), p = mail, r = deliveybot)
    location_has_mail = Fluent('location_has_package', BoolType(), p = mail, l = location)

    move = InstantaneousAction('move',  r = deliveybot, l_from = location, l_to = location)
    r = move.parameter('r')
    l_from = move.parameter('l_from')
    l_to = move.parameter('l_to')
    move.add_precondition(Or(is_connected(l_from, l_to), \
                                        is_connected(l_to, l_from)))
    move.add_precondition(deliveybot_at(r, l_from))
    move.add_precondition(Not(is_occupied(l_to))) #at end, l_to is free
    move.add_effect(deliveybot_at(r, l_from), False)
    move.add_effect(is_occupied(l_from), False)
    move.add_effect(deliveybot_at(r, l_to), True)
    move.add_effect(is_occupied(l_to), True)

    pickup = InstantaneousAction('pickup', p = mail, r = deliveybot, l = location)
    p = pickup.parameter('p')
    r = pickup.parameter('r')
    l = pickup.parameter('l')
    pickup.add_precondition(deliveybot_at(r, l))
    pickup.add_precondition(location_has_mail(p, l))
    pickup.add_effect(location_has_mail(p, l), False)
    pickup.add_effect(deliveybot_has_mail(p, r), True)

    drop = InstantaneousAction('drop', p = mail, r = deliveybot, l = location)
    p = drop.parameter('p')
    r = drop.parameter('r')
    l = drop.parameter('l')
    drop.add_precondition(deliveybot_at(r, l))
    drop.add_precondition(deliveybot_has_mail(p, r))
    drop.add_effect(deliveybot_has_mail(p, r), False)
    drop.add_effect(location_has_mail(p, l), True)

    problem = Problem('maildelivery')
    problem.add_action(move)
    problem.add_action(pickup)
    problem.add_action(drop)
    problem.add_fluent(deliveybot_at, default_initial_value = False)
    problem.add_fluent(is_connected, default_initial_value = False)
    problem.add_fluent(is_occupied, default_initial_value = False)
    problem.add_fluent(deliveybot_has_mail, default_initial_value = False)
    problem.add_fluent(location_has_mail, default_initial_value = False)

    #objects of problem
    locations = [Object(f"l{i}", location) for i in range(4)]
    deliverybot = Object("r",robot)
    note = Object("p",package)
    problem.add_objects(locations + [deliverybot] + [note])

m = createMap()
r = robot(gtsam.Pose2(m.landmarks[0].xy[0],m.landmarks[0].xy[1],landmark.angle(m.landmarks[0],m.landmarks[1])))
createPlan(m,[r])

_, ax = plotting.spawnWorld()
m.plot(ax)
# graphics_r = r.plot(ax)

# odom = gtsam.Pose2(0.4,0,0)
# cmd = move(odom)

# with plt.ion():
#     for _ in range(5):
#         r.move(cmd)

#         graphics_r.remove()
#         graphics_r = r.plot(ax)
#         plt.pause(0.5)

plt.show()



