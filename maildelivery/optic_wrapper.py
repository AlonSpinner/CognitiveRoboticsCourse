#optic:
#https://nms.kcl.ac.uk/planning/software/optic.html

import os
import subprocess

DIR_PATH = os.path.join(os.path.dirname(__file__))
DOCKER_DIR_PATH = os.path.join(DIR_PATH,'docker')
DOMAIN_PATH = os.path.join(DOCKER_DIR_PATH,"domain.pddl")
PROBLEM_PATH = os.path.join(DOCKER_DIR_PATH,"problem.pddl")
PLAN_PATH = os.path.join(DOCKER_DIR_PATH,"plan.txt")

def place_files(domain_old,problem_old):
    subprocess.run(f"cp {domain_old} {DOMAIN_PATH}", shell = True)
    subprocess.run(f"cp {problem_old} {PROBLEM_PATH}", shell = True)

def run_optic():
    # p = subprocess.run(f"./optic-clp {DOMAIN_PATH} {PROBLEM_PATH} > {PLAN_PATH}", \
    #      cwd = DIR_PATH, shell = True)
    p = subprocess.run(f"./optic-rewrite-no-lp {DOMAIN_PATH} {PROBLEM_PATH} > {PLAN_PATH}", \
         cwd = DIR_PATH, shell = True)
    return p.returncode

def run_optic_docker():
    #if this doesnt work.. check this out:
        #https://docs.docker.com/engine/install/linux-postinstall/
    
    p = subprocess.run("bash ./run.sh", cwd = DOCKER_DIR_PATH, shell = True)
    return p.returncode

def get_plan(file = None):
    if file is None:
        file = PLAN_PATH

    with open(file) as f:
        lines = f.readlines()

    #find last ';' and take everything from there
    last = 0
    for i,l in enumerate(lines):
        if l[0] == ';':
            last = i

    lines = lines[last+1:]

    execution_times = []
    actions = []
    durations = []

    for l in lines:
        execution_times += [ float(l[:l.find(':')]) ]
        actions += [ tuple(l[l.find('(')+1:l.find(')')].split(' ')) ]
        durations += [ float(l[l.find('[')+1:l.find(']')]) ]

    return execution_times, actions, durations

def add_problem_lines(added_lines : list[str]):
    with open(PROBLEM_PATH, 'r') as file:
        lines = file.readlines()
    with open(PROBLEM_PATH, 'w') as file:
        for line in lines[:-1]:
            file.write(line)
        for line in added_lines:
            file.write(line + '\n')
        file.write(')')

def remove_problem_lines(n : int):
    with open(PROBLEM_PATH, 'r') as file:
        lines = file.readlines()
    with open(PROBLEM_PATH, 'w') as file:
        lines = lines[:-(n + 2)] #remove the previous ')'
        newlines = lines +  [')']
        for line in newlines:
            file.write(line)
        
if __name__ == '__main__':
    t, cmd ,duration = get_plan()
    print(cmd)

