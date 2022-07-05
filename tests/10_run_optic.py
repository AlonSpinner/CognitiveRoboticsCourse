from maildelivery.optic_wrapper import run_optic, get_plan

run_optic()
execution_times, actions, durations = get_plan()
print(actions)
