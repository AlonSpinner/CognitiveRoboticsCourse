import numpy as np
import gtsam
from maildelivery.datatypes import beacon, landmark, package, mvnormal
from maildelivery.datatypes import goto, move
from maildelivery.robot import robot
from maildelivery.map import Map

CONTROL_THETA_THRESHOLD = np.radians(0.001)
CONTROL_DIST_THRESHOLD = 0.001

def control(r :robot, cmd : goto):
    #given r and goto command, produce proper move command
    e_theta = r.pose.bearing(cmd.lm.xy).theta()
    if abs(e_theta) > CONTROL_THETA_THRESHOLD:
        u = np.sign(e_theta)*min(abs(e_theta),r.max_rotate)
        return move(gtsam.Pose2(0,0,u))

    e_dist = r.pose.range(cmd.lm.xy)
    if e_dist > CONTROL_DIST_THRESHOLD:
        u = min(e_dist,r.max_forward)
        return move(gtsam.Pose2(u, 0, 0))
    
    return False

def execute_plan(robot : robot, plan):
    if ~plan: #plan is empty
        return True
    
    cmd  = plan[0]
    if cmd['type'] == 'move':
        success = robot.move(cmd['value'])
    elif cmd['type'] == 'reach':
        success = robot.reach(cmd['value'])
    elif cmd['type'] == 'pickup':
        success = robot.pickup(cmd['value'])
    elif cmd['type'] == 'drop':
        success = robot.drop(cmd['value'])

    if not success: #failure to do cmd
        return False

    execute_plan(robot,plan[1:])

class planner:
    def __init__(self) -> None:
        pass
    def create_plan(self, belief : mvnormal, map : Map, packages : list[package]):
        #create a sequence of [move,reach,pickup,drop] actions to later be formulated as PDDL
        pass

class estimator:
    unit_step_straight_cov = []
    unit_step_rotate_cov = []

    def __init__(self) -> None:
        self.isam2 = self.isam2Initalize()
        pass

    def update(self) -> None:
        #update factor graph
        pass
    
    def addOdometryFactor(self) -> None:
        pass

    def addMeasurementFactor(self) -> None:
        pass

    def isam2Initalize(self) -> None:
        parameters = gtsam.ISAM2Params()
        parameters.setRelinearizeThreshold(0.1)
        parameters.setRelinearizeSkip(1)
        self.isam2 = gtsam.ISAM2(parameters)

    def get_belief(self) -> None:
        pass