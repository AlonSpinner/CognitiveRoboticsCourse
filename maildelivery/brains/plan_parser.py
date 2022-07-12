from maildelivery.agents import agent, robot, action, wait, move, pickup, drop, chargeup, drone_fly, drone_fly_robot
from maildelivery.world import enviorment

def parse_plan(execution_times, actions, durations, env : enviorment, agents : list[agent]):
    #from actions ('action_name','param1','param2') to my actions
    Nrobots = 0
    for a in agents:
        Nrobots += type(a) == robot

    parsed_actions = []
    for e, a, d in zip(execution_times, actions, durations):
        name = a[0]
        params = a[1:]
        if name == 'move':
            parsed_actions.append(move(
            agent = agents[(int(params[0][1:]))],
            loc_from = env.locations[int(params[1][1:])], #locations_from
            loc_to = env.locations[int(params[2][1:])], #locations_to
            time_start = e,
            time_end = e + d
            )) 
        elif name == 'drop':
            parsed_actions.append(drop(
            agent = agents[(int(params[1][1:]))],
            p = env.packages[int(params[0][1:])], #package
            loc = env.locations[int(params[2][1:])], #location
            time_start = e,
            time_end = e + d
            )) 
        elif name == 'pickup':
            parsed_actions.append(pickup(
            agent = agents[(int(params[1][1:]))],
            p = env.packages[int(params[0][1:])], #package
            loc = env.locations[int(params[2][1:])], #location
            time_start = e,
            time_end = e + d
                ))

        elif name == 'chargeup':
            parsed_actions.append(chargeup(
                agent = agents[(int(params[0][1:]))],
                loc = env.locations[int(params[1][1:])], #location
                time_start = e,
                time_end = e + d
                ))
        elif name == 'drone_fly':
            parsed_actions.append(drone_fly(
                agent = agents[(int(params[0][1:])) + Nrobots], #drone index
                loc_from = env.locations[int(params[1][1:])], #location_from
                loc_to = env.locations[int(params[2][1:])], #location_to
                time_start = e,
                time_end = e + d
                ))
        elif name == 'drone_fly_robot':
            parsed_actions.append(drone_fly_robot(
                agent = agents[(int(params[0][1:])) + Nrobots], #drone index
                robot = agents[(int(params[1][1:]))], #robot index
                loc_from = env.locations[int(params[2][1:])], #location_from
                loc_to = env.locations[int(params[3][1:])], #location_to
                time_start = e,
                time_end = e + d
                ))

    return parsed_actions

def actions_indicies_per_agent(parsed_actions : list[action], Nagents):
    #split to N lists each holding indicies of actions [indicies for robot0, indicies for robot1...]
    actions_indicies_per_agent = [[] for _ in range(Nagents)]
    
    for idx, a in enumerate(parsed_actions):
        actions_indicies_per_agent[a.agent.id].append(idx)
    
    return actions_indicies_per_agent

def plan_per_agent(actions_indicies_per_agent, execution_times, actions, durations, agents):
    Nagents = len(agents)
    agent_actions = [[] for _ in range(Nagents)]
    agent_durations = [[] for _ in range(Nagents)]
    agent_execution_times = [[] for _ in range(Nagents)]

    #all robots start with waiting
    for i,ai in enumerate(agents):
        agent_execution_times[i] += [0]
        agent_actions[i] += [wait(ai)]
        agent_durations[i] += [0]

    for i in range(Nagents):
        agent_execution_times[i] += [execution_times[k] for k in actions_indicies_per_agent[i]]
        agent_actions[i] += [actions[k] for k in actions_indicies_per_agent[i]]
        agent_durations[i] += [durations[k] for k in actions_indicies_per_agent[i]]
    
    return agent_execution_times, agent_actions, agent_durations

def full_plan_2_per_agent(execution_times, actions, durations, agents):
    Nagents = len(agents)
    a_i_p_r = actions_indicies_per_agent(actions, Nagents)
    a_execution_times, a_actions, a_durations = plan_per_agent(a_i_p_r, execution_times, actions, durations, agents)
    return a_execution_times, a_actions, a_durations

def parse_up(result_plan):
    #from list of up timed_actions to list of tuples [('action_name','param1','param2')]
    #used inside the planners/brains
    
    execution_times = []
    actions = []
    durations = []

    for p in result_plan:
        execution_times += [float(p[0])]
        actions += [up_action_2_str(p[1])]    
        durations += [float(p[2])] if p[2] is not None else [0.01]

    return execution_times, actions, durations

def up_action_2_str(a):
    str_a = str(a)
    action_name = str_a[:str_a.find('(')]
    return tuple((','.join([action_name] + [str(p) for p in a._params])).split(','))
    