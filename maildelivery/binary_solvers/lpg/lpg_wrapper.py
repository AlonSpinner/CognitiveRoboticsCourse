#optic:
#https://nms.kcl.ac.uk/planning/software/optic.html

import os
import subprocess
from typing import Literal

from maildelivery.binary_solvers.paths import DOMAIN_PATH, PROBLEM_PATH, PLAN_PATH, PDDL_DIR
DIR_PATH = os.path.join(os.path.dirname(__file__))
BINARY_NAME = "lpg-td"


def run(mode : Literal["speed", "quality", "-n 2"] = 'speed'):
    p = subprocess.run(f"./{BINARY_NAME} -o {DOMAIN_PATH} -f {PROBLEM_PATH} -{mode}", \
         cwd = DIR_PATH, shell = True)
    p = subprocess.run('find . -type f -name "*.SOL"', cwd = PDDL_DIR, shell = True)
    p = subprocess.run(f"mv plan_problem.pddl.SOL {PLAN_PATH}", cwd = PDDL_DIR, shell = True)
    return p.returncode

def get_plan(file = None):
    if file is None:
        file = PLAN_PATH

    with open(file) as f:
        lines = f.readlines()

    #find last ';' and take everything from there
    plan_line_indicies = []
    for i,l in enumerate(lines):
        if not(l[0] == ';' or l[0] == ' ' or l[0:2] == '\n'):
            plan_line_indicies.append(i)

    lines = [lines[i] for i in plan_line_indicies]

    execution_times = []
    actions = []
    durations = []

    for l in lines:
        execution_times += [ float(l[:l.find(':')]) ]
        actions += [ tuple(l[l.find('(')+1:l.find(')')].lower().split(' ')) ]
        durations += [ float(l[l.find('[')+1:l.find(']')]) ]

    return execution_times, actions, durations

if __name__ == '__main__':
    t, cmd ,duration = get_plan()
    print(cmd)

