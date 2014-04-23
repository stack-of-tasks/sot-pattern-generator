# ______________________________________________________________________________
# ******************************************************************************
# A simple Herdt walking pattern generator for Romeo.
# ______________________________________________________________________________
# ******************************************************************************

# from dynamic_graph.sot.romeo.sot_romeo_controller import *
# Create the robot.
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.romeo.robot import *
robot = Robot('ROMEO', device=RobotSimu('ROMEO'))
from dynamic_graph import plug
plug(robot.device.state, robot.dynamic.position)


# Binds with ros, export joint_state.
from dynamic_graph.ros import *
ros = Ros(robot)

# Create a solver.
from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize
solver = initialize ( robot )

from dynamic_graph.sot.pattern_generator.walking import CreateEverythingForPGwithUrdf , walkFewSteps, walkAndrei
CreateEverythingForPGwithUrdf ( robot , solver )
# walkFewSteps ( robot )
walkAndrei( robot )

# Alternate visualization tool
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer
addRobotViewer(robot.device,small=True,small_extra=0,verbose=False)


#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------

from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
dt=5e-3
@loopInThread
def inc():
    robot.device.increment(dt)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

# --- HERDT PG AND START -------------------------------------------------------
# Set the algorithm generating the ZMP reference trajectory to Herdt's one.
# pg.startHerdt(False)

print('You can now modifiy the speed of the robot by setting pg.pg.velocitydes')
print('example : robot.pg.velocitydes.value =(0.1,0.0,0.0)\n')

robot.pg.velocitydes.value =(0.1,0.0,0.0)

go()

