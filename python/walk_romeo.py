# ______________________________________________________________________________
# ******************************************************************************
# A simple Herdt walking pattern generator for Romeo.
# ______________________________________________________________________________
# ******************************************************************************

from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay

from numpy import *
def totuple( a ):
    al=a.tolist()
    res=[]
    for i in range(a.shape[0]):
        res.append( tuple(al[i]) )
    return tuple(res)

# Create the robot.
from dynamic_graph.sot.romeo.romeo import *
robot = Robot('romeo', device=RobotSimu('romeo'))
plug(robot.device.state, robot.dynamic.position)

# Binds with ros, export joint_state.
from dynamic_graph.ros import *
ros = Ros(robot)

# Create a solver.
from dynamic_graph.sot.dynamics.solver import Solver
solver = Solver(robot)

# Alternate visualization tool
addRobotViewer(robot.device,small=True,small_extra=24,verbose=False)


#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------

dt=5e-3

@loopInThread
def inc():
    robot.device.increment(dt)
    updateComDisplay(robot.device,robot.dynamic.com)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

# --- PG ---------------------------------------------------------
from dynamic_graph.sot.pattern_generator.meta_pg import MetaPG
pg = MetaPG(robot.dynamic)
pg.plugZmp(robot.device)

# ---- TASKS -------------------------------------------------------------------
# ---- WAIST TASK ---
#  This task fix the motion of the waist around the z axis, the roll and the pitch
taskWaist=MetaTask6d('waist',robot.dynamic,'waist','waist')
pg.plugWaistTask(taskWaist)
taskWaist.task.controlGain.value = 5
taskWaist.feature.selec.value = '011100'

# --- TASK COM ---
# the x,y coords of the centor of mass are given by the pattern generator
taskCom = MetaTaskKineCom(robot.dynamic,"compd")
pg.plugComTask(taskCom)
taskCom.feature.selec.value = '011'

# --- TASK FEET
# The feet are constrained by two 6dofs tasks.
taskRF=MetaTask6d('rf',robot.dynamic,'rf','right-ankle')
plug(pg.pg.rightfootref,taskRF.featureDes.position)
taskRF.task.controlGain.value = 40

taskLF=MetaTask6d('lf',robot.dynamic,'lf','left-ankle')
plug(pg.pg.leftfootref,taskLF.featureDes.position)
taskLF.task.controlGain.value = 40

############################################################################
# Complete the task definition here.
# ---- HEAD ORIENTATION ---
#  set the orientation of the gaze (head) to be the same as the one of the foot.
# Define a metaTask for a 6d task controlling the waistOrientation.
# 1\ Define a MetaTask6d taskHead, constraining the head, attached to the gaze link
taskHead=MetaTask6d('head',robot.dynamic,'gaze','gaze')

# 2\ Link the orientation of the right foot to the desired position of the head.
plug(taskRF.featureDes.position, taskHead.featureDes.position)

# 3\ Only constraint the rotation of the head.
taskHead.feature.selec.value = '111000'

# 4\ set the gain
taskHead.task.controlGain.value = 5


# ---- ARMS ---
# set a default position for the joints arms
# 1\ Define two features (entity FeatureGeneric) corresponding to the desired potion.
#    the desired position corresponds to the initial configuration of the robot
#    romeo: initialConfig['romeo']
#    * the reference position is the position given by the entity dyn,
#    * the jacobian is the identity: ... = totuple( identity(robot.dimension) )
#    * the joint controlled are those of the two arms [12,25] sur 39.
featurePosition = FeatureGeneric('featurePosition')
featurePositionDes = FeatureGeneric('featurePositionDes')
featurePosition.setReference('featurePositionDes')
plug(robot.dynamic.position,featurePosition.errorIN)
featurePositionDes.errorIN.value = robot.halfSitting
featurePosition.jacobianIN.value = totuple( identity(robot.dimension) )

# 2\ Define the task. Associate to the task the position feature.
taskPosition = Task('taskPosition')
taskPosition.add('featurePosition')

# 3\ (Optional) attach an adaptive gain (entity GainAdaptive) to the task created.
gainPosition = GainAdaptive('gainPosition')
gainPosition.set(0.1,0.1,125e3)
gainPosition.gain.value = 5
plug(taskPosition.error,gainPosition.error)
plug(gainPosition.gain,taskPosition.controlGain)
featurePosition.selec.value = '000000000000001111111111111100000000000'


############################################################################


# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------
solver.push(taskWaist.task)
solver.push(taskRF.task)
solver.push(taskLF.task)
solver.push(taskCom.task)
# Stun the upper part of the body.
solver.push(taskHead.task)		# constraint the head orientation: look straight ahead
solver.push(taskPosition)			# stun the arms.


# --- HERDT PG AND START -------------------------------------------------------
# Set the algorithm generating the ZMP reference trajectory to Herdt's one.
pg.startHerdt(False)

print('You can now modifiy the speed of the robot by setting pg.pg.velocitydes')
print('example : pg.pg.velocitydes.value =(0.1,0.0,0.0)\n')

go()


