from maildelivery.agents import move, pickup, drop, robot, action
from maildelivery.world import enviorment
import numpy as np

import unified_planning as up
from unified_planning.shortcuts import UserType, BoolType,\
        Fluent, InstantaneousAction, DurativeAction, Problem, Object,\
        OneshotPlanner, Or, Not, IntType, Int, StartTiming, EndTiming, GE, SimulatedEffect
from unified_planning.io.pddl_writer import PDDLWriter
from unified_planning.engines import PlanGenerationResultStatus
from unified_planning.model.metrics import MinimizeMakespan, MinimizeActionCosts, MinimizeExpressionOnFinalState, MaximizeExpressionOnFinalState


up.shortcuts.get_env().credits_stream = None #removes the printing planners credits 
from maildelivery import optic_wrapper

NOT_CONNECTED_DISTANCE = int(100)

class robot_planner:
    '''
    multirobot, instant actions, no charging
    '''
    def __init__(self) -> None:
        self.planner_name = 'optic' #can also be 'tamer' or 'up-auto'
        self.f_dist2charge  = lambda dist: 2 * dist
        self.create_domain()

    def create_domain(self) -> None:
        _location = UserType('location')
        _robot = UserType('robot')
        _package = UserType('package')

        #problem variables that are changed by actions on objects (no floats please, they cause problems to solvers)
        robot_at = Fluent('robot_at', BoolType(), r = _robot, l = _location)
        is_connected = Fluent('is_connected', BoolType(), l_from = _location, l_to = _location)
        is_free = Fluent('is_free', BoolType(), l = _location)
        robot_has_package = Fluent('robot_has_package', BoolType(), p = _package, r = _robot)
        robot_can_hold_package = Fluent('robot_can_hold_package', BoolType(), r = _robot)
        location_has_package = Fluent('location_has_package', BoolType(), p = _package, l = _location)
        distance = Fluent('distance', IntType(), l_from = _location, l_to = _location)
        charge = Fluent('charge', IntType(0,100), r = _robot)

        _move = DurativeAction('move',  r = _robot, l_from = _location, l_to = _location)
        r = _move.parameter('r')
        l_from = _move.parameter('l_from')
        l_to = _move.parameter('l_to')
        _move.set_fixed_duration(distance(l_from,l_to))
        _move.add_condition(StartTiming(), is_connected(l_from, l_to))
        _move.add_condition(StartTiming(), robot_at(r, l_from))
        _move.add_condition(EndTiming(),is_free(l_to)) #at end, l_to is free
        _move.add_condition(StartTiming(),GE(charge(r),self.f_dist2charge(distance(l_from,l_to))))
        _move.add_effect(StartTiming(),robot_at(r, l_from), False)
        _move.add_effect(StartTiming(),is_free(l_from), True)
        _move.add_effect(EndTiming(),robot_at(r, l_to), True)
        _move.add_effect(EndTiming(),is_free(l_to), False)
        def decrease_charge_fun(problem, state, actual_params):
            dist = state.get_value(distance(actual_params.get(l_from),actual_params.get(l_to))).constant_value()
            requiredCharge = self.f_dist2charge(dist)
            currentCharge = state.get_value(charge(actual_params.get(r))).constant_value()
            return [Int(currentCharge-requiredCharge)]
        _move.set_simulated_effect(StartTiming(),SimulatedEffect([charge(r)], decrease_charge_fun))

        _pickup = InstantaneousAction('pickup', p = _package, r = _robot, l = _location)
        p = _pickup.parameter('p')
        r = _pickup.parameter('r')
        l = _pickup.parameter('l')
        _pickup.add_precondition(robot_at(r, l))
        _pickup.add_precondition(location_has_package(p, l))
        _pickup.add_precondition(robot_can_hold_package(r))
        _pickup.add_effect(location_has_package(p, l), False)
        _pickup.add_effect(robot_has_package(p, r), True)
        _pickup.add_effect(robot_can_hold_package(r), False)

        _drop = InstantaneousAction('drop', p = _package, r = _robot, l = _location)
        p = _drop.parameter('p')
        r = _drop.parameter('r')
        l = _drop.parameter('l')
        _drop.add_precondition(robot_at(r, l))
        _drop.add_precondition(robot_has_package(p, r))
        _drop.add_effect(robot_has_package(p, r), False)
        _drop.add_effect(location_has_package(p, l), True)
        _drop.add_effect(robot_can_hold_package(r), True)

        problem = Problem('maildelivery')
        problem.add_action(_move)
        problem.add_action(_pickup)
        problem.add_action(_drop)
        problem.add_fluent(robot_at, default_initial_value = False)
        problem.add_fluent(is_connected, default_initial_value = False)
        problem.add_fluent(is_free, default_initial_value = True)
        problem.add_fluent(robot_has_package, default_initial_value = False)
        problem.add_fluent(location_has_package, default_initial_value = False)
        problem.add_fluent(robot_can_hold_package, default_initial_value = True)
        problem.add_fluent(charge, default_initial_value = int(0))
        problem.add_fluent(distance, default_initial_value = int(NOT_CONNECTED_DISTANCE)) #some absuard number
        
        # problem.add_quality_metric(metric =  MinimizeMakespan())
        # problem.add_quality_metric(metric = MinimizeActionCosts({
        #                                                         _move: Int(1),
        #                                                         _pickup: Int(0),
        #                                                         _drop: Int(0)
        #                                                         }))

        #save to self
        self.problem = problem
        #user types
        self._location = _location
        self._robot = _robot
        self._package = _package
        #fluents
        self.robot_at = robot_at
        self.is_connected = is_connected
        self.is_free = is_free
        self.robot_has_package = robot_has_package
        self.location_has_package = location_has_package
        self.robot_can_hold_package = robot_can_hold_package
        self.charge = charge
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
            self.problem.set_initial_value(self.is_free(
                                                _locations[r.last_location]),
                                                False)
            self.problem.set_initial_value(self.charge(_robots[r.id]), r.charge)

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
                self.problem.set_initial_value(self.robot_can_hold_package(_robots[p.owner]),False)
        #goal
        for p in env.packages:
            self.problem.add_goal(self.location_has_package(_packages[p.id],_locations[p.goal]))
        for r in robots:
            self.problem.add_goal(self.robot_at(_robots[r.id],_locations[r.goal_location]))

        # self.problem.add_quality_metric(metric = MaximizeExpressionOnFinalState(self.charge(_robots[0])))

        self._locations = _locations
        self._robots = _robots
        self._packages = _packages
        
    def solve(self):
        if self.planner_name == 'up-auto':
            with OneshotPlanner(problem_kind = self.problem.kind) as planner:
                result = planner.solve(self.problem)
        elif self.planner_name == 'tamer':
            with OneshotPlanner(name='tamer',
                    optimality_guarantee=PlanGenerationResultStatus.SOLVED_OPTIMALLY) as planner:
                #this solved optimally seems to do nothing
                result = planner.solve(self.problem)
            return result.plan
        
        elif self.planner_name == 'optic':
            w = PDDLWriter(self.problem)
            with open(optic_wrapper.DOMAIN_PATH, 'w') as f:
                print(w.get_domain(), file = f)
            with open(optic_wrapper.PROBLEM_PATH, 'w') as f:
                print(w.get_problem(), file = f)
            print('copied pddls')
            
            if len(self._robots) == 1:
                optic_wrapper.add_problem_lines([f' (:metric maximize (charge {self._robots[0]}))'])
            else:
                part1 = ' (:metric maximize (+ '
                part2 = ' '.join([f'(charge {rname})' for rname in self._robots])
                part3 = '))'
                newline = part1 + part2 + part3
                optic_wrapper.add_problem_lines([newline])
            

            optic_wrapper.run_optic()
            
            execution_times, actions, durations = optic_wrapper.get_plan()

            return execution_times, actions, durations

    def parse_actions(self, actions, env):
        if self.planner_name == 'optic':
            return self.parse_actions_optic(actions,env)
        else:
            return self.parse_actions_up(actions,env)

    def parse_actions_optic(self, actions : list[tuple], env : enviorment):
        parsed_actions = []
        for a in actions:
            name = a[0]
            params = a[1:]
            if name == 'move':
                parsed_actions.append(move(
                int(params[0][1:]), #robot id
                env.locations[int(params[1][1:])], #locations_from
                env.locations[int(params[2][1:])], #locations_to
                )) 
            elif name == 'drop':
                parsed_actions.append(drop(
                    int(params[1][1:]), #robot id
                    env.packages[int(params[0][1:])], #package
                    env.locations[int(params[2][1:])] #location
                    )) 
            elif name == 'pickup':
                parsed_actions.append(pickup(
                    int(params[1][1:]), #robot id
                    env.packages[int(params[0][1:])], #package
                    env.locations[int(params[2][1:])] #location
                    ))
        return parsed_actions
    
    def parse_actions_up(self, actions : list[up.plans.plan.ActionInstance], env : enviorment):
        parsed_actions = []
        for a in actions:
            if a.action.name == 'move':
                parsed_actions.append(move(
                    int(str(a.actual_parameters[0])[1:]), #robot id
                    env.locations[int(str(a.actual_parameters[1])[1:])], #locations_from
                    env.locations[int(str(a.actual_parameters[2])[1:])], #locations_to
                    )) 
            elif a.action.name == 'drop':
                parsed_actions.append(drop(
                    int(str(a.actual_parameters[1])[1:]), #robot id
                    env.packages[int(str(a.actual_parameters[0])[1:])], #package
                    env.locations[int(str(a.actual_parameters[2])[1:])] #location
                    )) 
            elif a.action.name == 'pickup':
                parsed_actions.append(pickup(
                    int(str(a.actual_parameters[1])[1:]), #robot id
                    env.packages[int(str(a.actual_parameters[0])[1:])], #package
                    env.locations[int(str(a.actual_parameters[2])[1:])] #location
                    ))
        return parsed_actions

    def actions_per_robot(self,parsed_actions : list[action], Nrobots = None):
        if Nrobots is None:
            robots_inds = np.sort(np.unique([a.robot_id for a in parsed_actions]))
        else:
            robots_inds = list(range(Nrobots))
        
        robots_actions = [[] for _ in robots_inds]
        robot_actions_indicies = [[] for _ in robots_inds]
        
        for a,idx in enumerate(parsed_actions):
            robots_actions[a.robot_id].append(a)
            robot_actions_indicies[a.robot_id].append(idx)
        
        return robots_actions, robot_actions_indicies

class drone_planner:
    pass
    # def __init__(self) -> None:
    #     _location = UserType('_location')
    #     _drone = UserType('_drone')
    #     _charge = UserType('charge')

    #     #problem variables that are changed by actions on objects (no floats please, they cause problems to solvers)
    #     drone_at = Fluent('drone_at', BoolType(), d = _drone, l = _location)
    #     location_charge = Fluent('location_charge', IntType(), p = _package, l = _location)
    #     distance = Fluent(distance, )
        
