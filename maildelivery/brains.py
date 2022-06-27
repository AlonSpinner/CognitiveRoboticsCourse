import numpy as np
import gtsam
from maildelivery.datatypes import move, pickup, drop
from maildelivery.objects import robot
from maildelivery.enviorment import enviorment

import unified_planning
from unified_planning.shortcuts import UserType, BoolType,\
        Fluent, InstantaneousAction, Problem, Object, OneshotPlanner, Or, Not
unified_planning.shortcuts.get_env().credits_stream = None #removes the printing planners credits 

CONTROL_THETA_THRESHOLD = np.radians(0.001)
CONTROL_DIST_THRESHOLD = 0.001
ROBOT_INDEX_SHIFT = 1000

def control(r :robot, cmd : move):
    #given r and goto command, produce proper move command
    e_theta = r.pose.bearing(cmd.lm_to_xy).theta()
    if abs(e_theta) > CONTROL_THETA_THRESHOLD:
        u = np.sign(e_theta)*min(abs(e_theta),r.max_rotate)
        return move(gtsam.Pose2(0,0,u))

    e_dist = r.pose.range(cmd.lm_to_xy)
    if e_dist > CONTROL_DIST_THRESHOLD:
        u = min(e_dist,r.max_forward)
        return move(gtsam.Pose2(u, 0, 0))
    
    return False

class planner0:
    '''
    planner0: multirobot, instant actions, no charging
    '''
    def __init__(self) -> None:
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

        #save to self
        self.problem = problem
        #user types
        self.location = location
        self.deliverybot = deliverybot
        self.mail = mail
        #fluents
        self.deliverybot_at = deliverybot_at
        self.is_connected = is_connected
        self.is_occupied = is_occupied
        self.deliveybot_has_mail = deliveybot_has_mail
        self.location_has_mail =location_has_mail

    
    def create_plan(self, env : enviorment, robots : list[robot]):
        locations = [Object(f"l{id}", self.location) for id in [lm.id for lm in env.landmarks]]
        deliverybots = [Object(f"r{id}", self.deliverybot) for id in [bot.id for bot in robots]]
        notes = [Object(f"m{id}", self.mail) for id in [p.id for p in env.packages]]
        self.problem.add_objects(locations + deliverybots + notes)

        #locations connectivity
        for c in env.connectivityList:
            self.problem.set_initial_value(self.is_connected(
                                        locations[c[0]],
                                        locations[c[1]]),
                                        True)
        # robot at start
        for r in robots:
            self.problem.set_initial_value(self.deliverybot_at(
                                                    deliverybots[r.id],
                                                    locations[r.last_landmark]),
                                                    True)
            self.problem.set_initial_value(self.is_occupied(
                                                locations[r.last_landmark]),
                                                True) 
        #place packages
        for p in env.packages:
            if p.id < 1000:
                self.problem.set_initial_value(self.location_has_mail(
                                                            notes[p.id],
                                                            locations[p.owner]),
                                                            True)
            else:
                self.problem.set_initial_value(self.deliveybot_has_mail(
                                                notes[p.id],
                                                deliverybots[p.owner-1000]),
                                                True)
        #goal
        for p in env.packages:
            self.problem.add_goal(self.location_has_mail(notes[p.id],locations[p.goal]))

        with OneshotPlanner(problem_kind = self.problem.kind) as planner:
            result = planner.solve(self.problem)
        
        return result.plan

    def parse_actions(actions : list[unified_planning.plans.plan.ActionInstance], env : enviorment):
        parsed_actions = []
        for a in actions:
            if a.action.name == 'move':
                parsed_actions.append(move(
                    int(str(a.actual_parameters[0])[1:]), #robot id
                    env.landmarks[int(str(a.actual_parameters[1])[1:])].xy, #landmark_from xy
                    env.landmarks[int(str(a.actual_parameters[2])[1:])].xy, #landmark_to xy
                    )) 
            elif a.action.name == 'drop':
                parsed_actions.append(drop(
                    int(str(a.actual_parameters[1])[1:]), #robot id
                    int(str(a.actual_parameters[0])[1:]), #package id
                    env.landmarks[int(str(a.actual_parameters[2])[1:])] #landmark xy
                    )) 
            elif a.action.name == 'pickup':
                parsed_actions.append(pickup(
                    int(str(a.actual_parameters[1])[1:]), #robot id
                    int(str(a.actual_parameters[0])[1:]), #package id
                    env.landmarks[int(str(a.actual_parameters[2])[1:])] #landmark xy
                    ))
        return parsed_actions
        