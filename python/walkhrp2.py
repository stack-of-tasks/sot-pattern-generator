# ______________________________________________________________________________
# ******************************************************************************
# A simple Herdt walking pattern generator for HRP2.
# ______________________________________________________________________________
# ******************************************************************************


#----------------CUSTOM-------------------------------#
dt=5e-3
_urdfPath = "/local/rbudhira/git/trac/hrp2/hrp2_14_description/urdf/hrp2_14.urdf"
_srdfPath = "/local/rbudhira/git/trac/hrp2/hrp2_14_description/srdf/hrp2_14.srdf"
_urdfDir = ["/local/rbudhira/git/trac/hrp2"]
_robotName = 'HRP2LAAS'
_OperationalPointsMap = {'left-wrist'  : 'LARM_JOINT5',
                         'right-wrist' : 'RARM_JOINT5',
                         'left-ankle'  : 'LLEG_JOINT5',
                         'right-ankle' : 'RLEG_JOINT5',
                         'gaze'        : 'HEAD_JOINT1',
                         'waist'       : 'root_joint',
                         'chest'       : 'CHEST_JOINT1'}
_initialConfig = (0., 0., 0.648702, 0., 0. , 0.,                  # Free flyer
                  0., 0., 0., 0.,                                 # Chest and head
                  0.261799, 0.17453, 0., -0.523599, 0., 0., 0.1, # Left Arm
                  #0.,0.,0.,0.,0.,                                # Left hand
                  0.261799, -0.17453,  0., -0.523599, 0., 0., 0.1, # Right Arm
                  #0.,0.,0.,0.,0.,                                # Right Hand
                  0., 0., -0.453786, 0.872665, -0.418879, 0.,     # Left Leg
                  0., 0., -0.453786, 0.872665, -0.418879, 0.      # Right Leg
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
#from dynamic_graph.sot.pattern_generator.walking import initPg, initZMPRef, initWaistCoMTasks, initFeetTask, initPostureTask, pushTasks
from dynamic_graph.sot.pattern_generator.walking import walkNaveau, CreateEverythingForPG, walkFewSteps, walkAndrei

CreateEverythingForPG (robot, solver)

#------------------------------------------------#
#walkFewSteps ( robot )
walkNaveau( robot )
#walkAndrei( robot )
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
