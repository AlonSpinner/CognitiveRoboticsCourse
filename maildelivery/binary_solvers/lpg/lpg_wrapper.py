#optic:
#https://nms.kcl.ac.uk/planning/software/optic.html

import os
import subprocess

from maildelivery.binary_solvers.paths import DOMAIN_PATH, PROBLEM_PATH, PLAN_PATH
DIR_PATH = os.path.join(os.path.dirname(__file__))
BINARY_NAME = "lpg-td"

def run():
    p = subprocess.run(f"./{BINARY_NAME} -o {DOMAIN_PATH} -f {PROBLEM_PATH} > {PLAN_PATH}", \
         cwd = DIR_PATH, shell = True)
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

if __name__ == '__main__':
    t, cmd ,duration = get_plan()
    print(cmd)

