from maildelivery.agents import move, pickup, drop, robot, action
from maildelivery.world import enviorment, package
from maildelivery.binary_solvers import manipulate_pddls, paths
from maildelivery.binary_solvers.optic import optic_wrapper
from maildelivery.brains.plan_parser import parse_actions,actions_indicies_per_robot, plan_per_robot, parse_up

import numpy as np

import unified_planning as up
from unified_planning.shortcuts import UserType, BoolType,\
        Fluent, InstantaneousAction, DurativeAction, Problem, Object,\
        IntType, Int, StartTiming, EndTiming, GE, GT, Or, Equals, And, Not, Implies, OneshotPlanner, SimulatedEffect
from unified_planning.io.pddl_writer import PDDLWriter
import time
from unified_planning.model.metrics import MinimizeMakespan,\
     MinimizeActionCosts, MinimizeExpressionOnFinalState, MaximizeExpressionOnFinalState
up.shortcuts.get_env().credits_stream = None #removes the printing planners credits 


NOT_CONNECTED_DISTANCE = int(10000)

class robot_planner:
    '''
    multirobot, instant actions, no charging
    '''
    def __init__(self) -> None:
        self.f_dist2charge  = lambda dist: 2 * dist #not used here 
        self.create_domain()

    def create_domain(self) -> None:
        _location = UserType('location')
        _robot = UserType('robot')
        _package = UserType('package')

        #problem variables that are changed by actions on objects (no floats please, they cause problems to solvers)
        robot_at = Fluent('robot_at', BoolType(), r = _robot, l = _location)
        is_connected = Fluent('is_connected', BoolType(), l_from = _location, l_to = _location)
        location_is_free = Fluent('location_is_free', BoolType(), l = _location)
            #robot can wait on a location, occupying it, and no robot will pass over
        location_not_targeted = Fluent('not_targeted', BoolType(), l_to = _location)
            #no robot is targeting the location == moving towards it. We don't want "near misses"
        road_is_free = Fluent('road_is_free',BoolType(), l_from = _location, l_to = _location)
            #don't want two robots to go into head on collision
        robot_has_package = Fluent('robot_has_package', BoolType(), p = _package, r = _robot)
            #robot has a specific package, to allow for drop actions
        robot_not_holding_package = Fluent('robot_not_holding_package', BoolType(), r = _robot)
            #to prevent picking up more than one package
        location_has_package = Fluent('location_has_package', BoolType(), p = _package, l = _location)
        distance = Fluent('distance', IntType(), l_from = _location, l_to = _location)
        
        _move = DurativeAction('move',  r = _robot, l_from = _location, l_to = _location)
        r = _move.parameter('r')
        l_from = _move.parameter('l_from')
        l_to = _move.parameter('l_to')
        _move.set_fixed_duration(distance(l_from,l_to))
        _move.add_condition(StartTiming(), is_connected(l_from, l_to))
        _move.add_condition(StartTiming(), location_not_targeted(l_to))
        _move.add_condition(StartTiming(), road_is_free(l_to,l_from)) #opposite way
        _move.add_condition(StartTiming(), robot_at(r, l_from))
        _move.add_condition(EndTiming(),location_is_free(l_to))
        _move.add_effect(StartTiming(),robot_at(r, l_from), False)
        _move.add_effect(StartTiming(),location_is_free(l_from), True)
        _move.add_effect(StartTiming(), location_not_targeted(l_to), False)
        _move.add_effect(StartTiming(), road_is_free(l_from,l_to), False)
        _move.add_effect(EndTiming(),robot_at(r, l_to), True)
        _move.add_effect(EndTiming(),location_is_free(l_to), False)
        _move.add_effect(EndTiming(), location_not_targeted(l_to), True)
        _move.add_effect(EndTiming(), road_is_free(l_from,l_to), True)

        _pickup = InstantaneousAction('pickup', p = _package, r = _robot, l = _location)
        p = _pickup.parameter('p')
        r = _pickup.parameter('r')
        l = _pickup.parameter('l')
        _pickup.add_precondition(robot_at(r, l))
        _pickup.add_precondition(location_has_package(p, l))
        _pickup.add_precondition(robot_not_holding_package(r))
        _pickup.add_effect(location_has_package(p, l), False)
        _pickup.add_effect(robot_has_package(p, r), True)
        _pickup.add_effect(robot_not_holding_package(r), False)

        _drop = InstantaneousAction('drop', p = _package, r = _robot, l = _location)
        p = _drop.parameter('p')
        r = _drop.parameter('r')
        l = _drop.parameter('l')
        _drop.add_precondition(robot_at(r, l))
        _drop.add_precondition(robot_has_package(p, r))
        _drop.add_effect(robot_has_package(p, r), False)
        _drop.add_effect(location_has_package(p, l), True)
        _drop.add_effect(robot_not_holding_package(r), True)

        problem = Problem('maildelivery')
        problem.add_action(_move)
        problem.add_action(_pickup)
        problem.add_action(_drop)
        problem.add_fluent(robot_at, default_initial_value = False)
        problem.add_fluent(is_connected, default_initial_value = False)
        problem.add_fluent(location_is_free, default_initial_value = True)
        problem.add_fluent(robot_has_package, default_initial_value = False)
        problem.add_fluent(location_has_package, default_initial_value = False)
        problem.add_fluent(robot_not_holding_package, default_initial_value = True)
        problem.add_fluent(location_not_targeted, default_initial_value = True)
        problem.add_fluent(road_is_free, default_initial_value = True)
        problem.add_fluent(distance, default_initial_value = int(NOT_CONNECTED_DISTANCE)) #some absuard number

        #save to self
        self.problem = problem
        #user types
        self._location = _location
        self._robot = _robot
        self._package = _package
        #fluents
        self.robot_at = robot_at
        self.is_connected = is_connected
        self.location_is_free = location_is_free
        self.robot_has_package = robot_has_package
        self.location_has_package = location_has_package
        self.robot_not_holding_package = robot_not_holding_package
        self.location_not_targeted = location_not_targeted
        self.road_is_free = road_is_free
        self.distance = distance
        

    def create_problem(self, env : enviorment, robots : list[robot]):
        _locations = [Object(f"l{id}", self._location) for id in [loc.id for loc in env.locations]]
        _robots = [Object(f"r{id}", self._robot) for id in [bot.id for bot in robots]]
        _packages = [Object(f"p{id}", self._package) for id in [p.id for p in env.packages]]

        self.problem.add_objects(_locations + _robots + _packages)

        #locations connectivity and distance
        for c in env.connectivityList:
            self.problem.set_initial_value(self.is_connected(
                                        _locations[c[0]],
                                        _locations[c[1]]),
                                        True)
            self.problem.set_initial_value(self.is_connected(
                            _locations[c[1]],
                            _locations[c[0]]),
                            True)
            self.problem.set_initial_value(self.distance(
                            _locations[c[0]],
                            _locations[c[1]]),
                            int(env.locations[c[0]].distance(env.locations[c[1]]))
                            )
            self.problem.set_initial_value(self.distance(
                            _locations[c[1]],
                            _locations[c[0]]),
                            int(env.locations[c[1]].distance(env.locations[c[0]]))
                            )
        # robot at start
        for r in robots:
            self.problem.set_initial_value(self.robot_at(
                                                    _robots[r.id],
                                                    _locations[r.last_location]),
                                                    True)
            self.problem.set_initial_value(self.location_is_free(
                                                _locations[r.last_location]),
                                                False)

        #place packages
        for p in env.packages:

            if p.owner_type == 'location':
                self.problem.set_initial_value(self.location_has_package(
                                                            _packages[p.id],
                                                            _locations[p.owner]),
                                                            True)

            elif p.owner_type == 'robot':
                self.problem.set_initial_value(self.robot_has_package(
                                                _packages[p.id],
                                                _robots[p.owner]),
                                                True)
                self.problem.set_initial_value(self.robot_not_holding_package(_robots[p.owner]),False)
        #goal
        for p in env.packages:
            self.problem.add_goal(self.location_has_package(_packages[p.id],_locations[p.goal]))
        for r in robots:
            self.problem.add_goal(self.robot_at(_robots[r.id],_locations[r.goal_location]))

        # REMOVE THIS IF USING BINARY SOLVER
        # self.problem.add_quality_metric(metric =  MinimizeMakespan())
        # self.problem.add_quality_metric(metric = MaximizeExpressionOnFinalState(self.charge(_robots[0])))
        # problem.add_quality_metric(metric = MinimizeActionCosts({
        #                                                         _move: Int(1),
        #                                                         _pickup: Int(0),
        #                                                         _drop: Int(0)
        #                                                         }))

        self._robots = _robots
        self._locations = _locations
        self._packages = _packages
        
    def solve(self, engine_name = 'optic', only_read_plan = False, time_fix = True):        
        
        if engine_name == 'optic':
            w = PDDLWriter(self.problem)
            with open(paths.DOMAIN_PATH, 'w') as f:
                print(w.get_domain(), file = f)
            with open(paths.PROBLEM_PATH, 'w') as f:
                print(w.get_problem(), file = f)
            print('copied pddls')

            # add optimization here via rewriting the pddl files
            if len(self._robots) == 1:
                manipulate_pddls.add_problem_lines([f' (:metric maximize (charge {self._robots[0]}))'])
            else:
                part1 = ' (:metric maximize (+ '
                part2 = ' '.join([f'(charge {rname})' for rname in self._robots])
                part3 = '))'
                newline = part1 + part2 + part3
                manipulate_pddls.add_problem_lines([newline])
        
            if not only_read_plan:
                start = time.time()
                print('started solving domain+problem with optic')
                optic_wrapper.run_optic()
                end = time.time()
                print(f'finished solving in {end-start} seconds')
            execution_times, actions, durations = optic_wrapper.get_plan()
        
        if engine_name == 'tamer':
            with OneshotPlanner(name='tamer') as engine:
                result = engine.solve(self.problem)
                execution_times, actions, durations = parse_up(result.plan.timed_actions)

        if time_fix:
            execution_times = [execution_times[i] - execution_times[0] for i in range(len(execution_times))]

        return execution_times, actions, durations
        
