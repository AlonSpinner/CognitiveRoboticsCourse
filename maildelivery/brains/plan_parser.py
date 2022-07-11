from maildelivery.agents import action, wait, move, pickup, drop, chargeup
from maildelivery.world import enviorment

def parse_actions(actions : list[tuple], env : enviorment):
    #from actions ('action_name','param1','param2') to my actions
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

        elif name == 'chargeup':
            parsed_actions.append(chargeup(
                int(params[0][1:]), #robot id
                env.locations[int(params[1][1:])], #location
                ))

    return parsed_actions

def actions_indicies_per_agent(parsed_actions : list[action], Nagents):
    #split to N lists each holding indicies of actions [indicies for robot0, indicies for robot1...]
    actions_indicies_per_agent = [[] for _ in range(Nagents)]
    
    for idx, a in enumerate(parsed_actions):
        actions_indicies_per_agent[a.robot_id].append(idx)
    
    return actions_indicies_per_agent

def plan_per_agent(actions_indicies_per_robot, execution_times, actions, durations, Nagents):
    agent_actions = [[] for _ in range(Nagents)]
    agent_durations = [[] for _ in range(Nagents)]
    agent_execution_times = [[] for _ in range(Nagents)]

    #all robots start with waiting
    for i in range(Nagents):
        agent_execution_times[i] += [0]
        agent_actions[i] += [wait(i)]
        agent_durations[i] += [0]

    for i in range(Nagents):
        agent_execution_times[i] += [execution_times[k] for k in actions_indicies_per_robot[i]]
        agent_actions[i] += [actions[k] for k in actions_indicies_per_robot[i]]
        agent_durations[i] += [durations[k] for k in actions_indicies_per_robot[i]]
    
    return agent_execution_times, agent_actions, agent_durations

def full_plan_2_per_agent(execution_times, actions, durations, Nagents):
    a_i_p_r = actions_indicies_per_agent(actions, Nagents)
    r_execution_times, r_actions, r_durations = plan_per_agent(a_i_p_r, execution_times, actions, durations, Nagents)
    return r_execution_times, r_actions, r_durations

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
    