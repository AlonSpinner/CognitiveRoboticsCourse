import os
import subprocess
import maildelivery

DOCKER_DIR_PATH = os.path.join(maildelivery.__path__[0],'docker')
p = subprocess.run("bash run.sh", cwd = DOCKER_DIR_PATH, shell = True)