import os
import subprocess

def place_files(domain_old,problem_old):
    dir_path = os.path.join(os.path.dirname(__file__),'docker')

    domain_new = os.path.join(dir_path,"domain.pddl")
    problem_new = os.path.join(dir_path,"problem.pddl")
    subprocess.run(f"cp {domain_old} {domain_new}", shell = True)
    subprocess.run(f"cp {problem_old} {problem_new}", shell = True)

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

    t = [] #time of execution
    cmd = [] #command to execute
    duration = [] #duration of the command

    for l in lines:
        t.append(float(l[:l.find(':')]))
        cmd.append(tuple(l[l.find('(')+1:l.find(')')].split(' ')))
        duration.append(float(l[l.find('[')+1:l.find(']')]))

    return t, cmd, duration

if __name__ == '__main__':
    t, cmd ,duration = get_plan()
    print(cmd)

