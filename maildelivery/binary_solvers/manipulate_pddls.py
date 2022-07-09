from maildelivery.binary_solvers.paths import PROBLEM_PATH

def add_problem_lines(added_lines : list[str]):
    with open(PROBLEM_PATH, 'r') as file:
        lines = file.readlines()
        ii = 0
        for line in lines[::-1]:
            if line == '\n':
                ii += 1
            else:
                break
    with open(PROBLEM_PATH, 'w') as file:
        for line in lines[:-(1+ii)]:
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