import os
from maildelivery.optic_wrapper import place_files, run_optic, get_plan

py_dir_path = os.path.dirname(os.path.realpath(__file__))
dir_path = os.path.join(py_dir_path,"07_pddl")

domain_path = os.path.join(dir_path,"domain.pddl")
problem_path = os.path.join(dir_path,"problem.pddl")

place_files(domain_path, problem_path)
run_optic()
t, cmd ,duration = get_plan()

print(cmd)