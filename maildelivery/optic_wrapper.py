#optic:
#https://nms.kcl.ac.uk/planning/software/optic.html

import os
import subprocess

dir_path = os.path.join(os.path.dirname(__file__),'docker')
DOMAIN_PATH = os.path.join(dir_path,"domain.pddl")
PROBLEM_PATH = os.path.join(dir_path,"problem.pddl")

def place_files(domain_old,problem_old):
    subprocess.run(f"cp {domain_old} {DOMAIN_PATH}", shell = True)
    subprocess.run(f"cp {problem_old} {PROBLEM_PATH}", shell = True)

def run_optic():
    #if this doesnt work.. check this out:
        #https://docs.docker.com/engine/install/linux-postinstall/
    
    dir_path = os.path.join(os.path.dirname(__file__),'docker')
    p = subprocess.run("bash ./run.sh", cwd = dir_path, shell = True)
    return p.returncode
    

def get_plan(file = None):
    if file is None:
        dir_path = os.path.dirname(__file__)
        file = os.path.join(dir_path,'docker','plan.txt')

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

if __name__ == '__main__':
    t, cmd ,duration = get_plan()
    print(cmd)

