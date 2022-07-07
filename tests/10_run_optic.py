from maildelivery import optic_wrapper

returncode = optic_wrapper.run_optic()
if returncode == 0:
    execution_times, actions, durations = optic_wrapper.get_plan()
    print(actions)