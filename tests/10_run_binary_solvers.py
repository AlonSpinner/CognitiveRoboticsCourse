from maildelivery.binary_solvers.optic import optic_wrapper
from maildelivery.binary_solvers.lpg import lpg_wrapper

found_solution = optic_wrapper.run()
if found_solution:
    execution_times, actions, durations = optic_wrapper.get_plan()
    print(actions)
else:
    print('no solution')

found_solution = lpg_wrapper.run()
if found_solution:
    execution_times, actions, durations = lpg_wrapper.get_plan()
    print(actions)
else:
    print('no solution')