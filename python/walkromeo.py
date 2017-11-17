# ______________________________________________________________________________
# ******************************************************************************
# A simple Herdt walking pattern generator for ROMEO.
# ______________________________________________________________________________
# ******************************************************************************


#----------------CUSTOM-------------------------------#
#------SET THESE VALUES FOR YOUR ROBOT----------------#
dt=5e-3
_urdfPath = "/local/rbudhira/git/sot/pinocchio/models/romeo.urdf"
_urdfPath = "/local/rbudhira/git/laas/romeo/romeo_description/urdf/romeo.urdf"
_srdfPath = "/local/rbudhira/git/laas/romeo/romeo_description/srdf/romeo.srdf"
_urdfDir = ["/local/rbudhira/git/sot/pinocchio/models"]
_robotName = 'ROMEO'
_OperationalPointsMap = {'left-wrist'  : 'LWristPitch',
                         'right-wrist' : 'RWristPitch',
                         'left-ankle'  : 'LAnkleRoll',
                         'right-ankle' : 'RAnkleRoll',
                         'gaze'        : 'HeadRoll',
                         'waist'       : 'root_joint',
                         'chest'       : 'TrunkYaw'}

_initialConfig = (0, 0, .840252, 0, 0, 0,                                 # FF
                 0,  0,  -0.3490658,  0.6981317,  -0.3490658,   0,       # LLEG
                 0,  0,  -0.3490658,  0.6981317,  -0.3490658,   0,       # RLEG
                 0,                                                      # TRUNK
                 1.5,  0.6,  -0.5, -1.05, -0.4, -0.3, -0.2,              # LARM
                 0, 0, 0, 0,                                             # HEAD
                 1.5, -0.6,   0.5,  1.05, -0.4, -0.3, -0.2,              # RARM
                 )

#----------------PINOCCHIO----------------------------#
import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper

pinocchioRobot = RobotWrapper(_urdfPath, _urdfDir, se3.JointModelFreeFlyer())
pinocchioRobot.initDisplay(loadModel=True)



#----------------ROBOT - DEVICE AND DYNAMICS----------#
from dynamic_graph.sot.dynamics.humanoid_robot import HumanoidRobot
robot = HumanoidRobot(_robotName, pinocchioRobot.model,
                      pinocchioRobot.data, _initialConfig, _OperationalPointsMap)
#-----------------------------------------------------#

#----------------SOT (SOLVER)-------------------------#
from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize
solver = initialize( robot )
#-----------------------------------------------------#


#----------------PG--------------------------------#

from dynamic_graph.sot.pattern_generator.walking import walkNaveau, CreateEverythingForPG, walkFewSteps, walkAndrei

CreateEverythingForPG (robot, solver)

#------------------------------------------------#
# walkFewSteps ( robot )
walkNaveau( robot )
robot.pg.velocitydes.value =(0.1,0.0,0.0)
#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------

from dynamic_graph.sot.dynamics import fromSotToPinocchio
def inc():
    robot.device.increment(dt)
    #print robot.device.state.value
    pinocchioRobot.display(fromSotToPinocchio(robot.device.state.value))

def runner(n):
    for i in xrange(n):
        inc()
    robot.stopTracer()
  


# --- HERDT PG AND START -------------------------------------------------------
# Set the algorithm generating the ZMP reference trajectory to Herdt's one.
# pg.startHerdt(False)

print('You can now modifiy the speed of the robot by setting pg.pg.velocitydes')
print('example : robot.pg.velocitydes.value =(0.1,0.0,0.0)\n')
