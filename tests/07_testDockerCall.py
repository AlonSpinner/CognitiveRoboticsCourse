import command
import os
import docker
import subprocess
import maildelivery

from matplotlib import interactive

py_dir_path = os.path.dirname(os.path.realpath(__file__))
dir_path = os.path.join(py_dir_path,"07_pddl")

domain_path = os.path.join(dir_path,"domain_turtlebot.pddl")
problem_path = os.path.join(dir_path,"problem.pddl")

dockerfile_path = os.path.join(os.path.dirname(maildelivery.__file__),'Dockerfile')
#run docker

os.system('newgrp docker') #not sure why it doesnt work without this
client = docker.from_env()
# client.images.get("karpase/planning")
client.images.build(dockerfile_path)
# client.images.get(dockerfile_path)
print(client.images.list())
# cmd = "/planners/temporal/optic/release/optic/optic-clp"
# client.containers.run("ubuntu:latest", "echo hello world", detach = True)
# client.containers.run("karpase/planning","echo hello world" , detach = True)
# client.containers.run()

# print(client.containers.list())

# res = docker.run(['docker run -it karpase/planning'])
# print(res.output)