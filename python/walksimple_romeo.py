import sys
sys.path.append('../../sot-dyninv/python')

from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from meta_double_task_6d import MetaDoubleTask6d
from dynamic_graph.sot.pattern_generator.toes_handler import FeatureToesHandler
from dynamic_graph.sot.pattern_generator.toes_handler_zmp import FeatureToesHandlerZmp
from dynamic_graph.sot.dyninv import *

from robotSpecific import *
from robotSpecific_romeo import * 

# robotName = 'hrp10small'
robotName = 'romeo'

from numpy import *
def totuple( a ):
    al=a.tolist()
    res=[]
    for i in range(a.shape[0]):
        res.append( tuple(al[i]) )
    return tuple(res)

# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------

robotDim=robotDimension[robotName]
robot = RobotSimu("robot")
robot.resize(robotDim)

robot.set( initialConfig[robotName] )
dt=5e-3

# --- VIEWER -------------------------------------------------------------------
# --- VIEWER -------------------------------------------------------------------
# --- VIEWER -------------------------------------------------------------------
try:
   import robotviewer

   def stateFullSize(robot):
       return [float(val) for val in robot.state.value]+10*[0.0]
   RobotSimu.stateFullSize = stateFullSize
   robot.viewer = robotviewer.client('XML-RPC')
   # Check the connection
   robot.viewer.updateElementConfig('hrp',robot.stateFullSize())

   def refreshView( robot ):
       robot.viewer.updateElementConfig('hrp',robot.stateFullSize())
   RobotSimu.refresh = refreshView
   def incrementView( robot,dt ):
       robot.incrementNoView(dt)
       robot.refresh()
   RobotSimu.incrementNoView = RobotSimu.increment
   RobotSimu.increment = incrementView
   def setView( robot,*args ):
       robot.setNoView(*args)
       robot.refresh()
   RobotSimu.setNoView = RobotSimu.set
   RobotSimu.set = setView

   robot.refresh()
except:
    print "No robot viewer, sorry."
    robot.viewer = None


# --- MAIN LOOP ------------------------------------------

qs=[]
def inc():
    robot.increment(dt)
    qs.append(robot.state.value)

from ThreadInterruptibleLoop import *
@loopInThread
def loop():
    inc()
runner=loop()

@optionalparentheses
def go(): runner.play()
@optionalparentheses
def stop(): runner.pause()
@optionalparentheses
def next(): inc() #runner.once()

# --- shortcuts -------------------------------------------------
@optionalparentheses
def n():
    inc()
    qdot()
@optionalparentheses
def n5():
    for loopIdx in range(5): inc()
@optionalparentheses
def n10():
    for loopIdx in range(10): inc()
@optionalparentheses
def q():
    if 'dyn' in globals(): print dyn.ffposition.__repr__()
    print robot.state.__repr__()
@optionalparentheses
def qdot(): print robot.control.__repr__()
@optionalparentheses
def t(): print robot.state.time-1
@optionalparentheses
def iter():         print 'iter = ',robot.state.time
@optionalparentheses
def status():       print runner.isPlay

# --- DYN ----------------------------------------------------------------------
modelDir = pkgDataRootDir[robotName]
xmlDir = pkgDataRootDir[robotName]
specificitiesPath = xmlDir + '/' + specificitiesName[robotName]
jointRankPath = xmlDir + '/' + jointRankName[robotName]

dyn = Dynamic("dyn")
dyn.setFiles(modelDir, modelName[robotName],specificitiesPath,jointRankPath)
dyn.parse()

dyn.inertiaRotor.value = inertiaRotor[robotName]
dyn.gearRatio.value = gearRatio[robotName]

plug(robot.state,dyn.position)
dyn.velocity.value = robotDim*(0.,)
dyn.acceleration.value = robotDim*(0.,)

dyn.ffposition.unplug()
dyn.ffvelocity.unplug()
dyn.ffacceleration.unplug()
robot.control.unplug()

# --- PG ---------------------------------------------------------
from dynamic_graph.sot.pattern_generator import PatternGenerator,Selector

pg = PatternGenerator('pg')
pg.setVrmlDir(modelDir+'/')
pg.setVrml(modelName[robotName])
pg.setXmlSpec(specificitiesPath)
pg.setXmlRank(jointRankPath)
pg.buildModel()

# Standard initialization
pg.parseCmd(":samplingperiod 0.005")
pg.parseCmd(":previewcontroltime 1.6")
pg.parseCmd(":comheight 0.814")
pg.parseCmd(":omega 0.0")
pg.parseCmd(":stepheight 0.05")
pg.parseCmd(":singlesupporttime 0.780")
pg.parseCmd(":doublesupporttime 0.020")
pg.parseCmd(":armparameters 0.5")
pg.parseCmd(":LimitsFeasibility 0.0")
pg.parseCmd(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
pg.parseCmd(":TimeDistributeParameters 2.0 3.5 1.0 3.0")
pg.parseCmd(":UpperBodyMotionParameters 0.0 -0.5 0.0")
pg.parseCmd(":SetAlgoForZmpTrajectory Morisawa")

plug(dyn.position,pg.position)
plug(dyn.com,pg.com)
pg.motorcontrol.value = robotDim*(0,)
pg.zmppreviouscontroller.value = (0,0,0)

pg.initState()

# --- PG INIT FRAMES ---
geom = Dynamic("geom")
geom.setFiles(modelDir, modelName[robotName],specificitiesPath,jointRankPath)
geom.parse()
geom.createOpPoint('rf','right-ankle')
geom.createOpPoint('lf','left-ankle')

# -- create the operational points for the toes
dyn.createOpPoint('rt','right-toe')
dyn.createOpPoint('lt','left-toe')

# -- end toes

plug(dyn.position,geom.position)
geom.ffposition.value = 6*(0,)
geom.velocity.value = robotDim*(0,)
geom.acceleration.value = robotDim*(0,)

# --- Selector of Com Ref: when pg is stopped, pg.inprocess becomes 0
comRef = Selector('comRef'
                    ,['vector','ref',dyn.com,pg.comref])
plug(pg.inprocess,comRef.selec)

selecSupportFoot = Selector('selecSupportFoot'
                              ,['matrixHomo','pg_H_sf',pg.rightfootref,pg.leftfootref]
                              ,['matrixHomo','wa_H_sf',geom.rf,geom.lf]
                              )
plug(pg.SupportFoot,selecSupportFoot.selec)
sf_H_wa = Inverse_of_matrixHomo('sf_H_wa')
plug(selecSupportFoot.wa_H_sf,sf_H_wa.sin)
pg_H_wa = Multiply_of_matrixHomo('pg_H_wa')
plug(selecSupportFoot.pg_H_sf,pg_H_wa.sin1)
plug(sf_H_wa.sout,pg_H_wa.sin2)

# --- Compute the ZMP ref in the Waist reference frame.
wa_H_pg = Inverse_of_matrixHomo('wa_H_pg')
plug(pg_H_wa.sout,wa_H_pg.sin)
wa_zmp = Multiply_matrixHomo_vector('wa_zmp')
plug(wa_H_pg.sout,wa_zmp.sin1)
plug(pg.zmpref,wa_zmp.sin2)
# Connect the ZMPref to OpenHRP in the waist reference frame.
pg.parseCmd(':SetZMPFrame world')
plug(wa_zmp.sout,robot.zmp)

# ---- TASKS -------------------------------------------------------------------

# ---- WAIST TASK ---
taskWaist=MetaTask6d('waist',dyn,'waist','waist')

# Build the reference waist pos homo-matrix from PG.
waistReferenceVector = Stack_of_vector('waistReferenceVector')
plug(pg.initwaistposref,waistReferenceVector.sin1)
plug(pg.initwaistattref,waistReferenceVector.sin2)
waistReferenceVector.selec1(0,3)
waistReferenceVector.selec2(0,3)
waistReference=PoseRollPitchYawToMatrixHomo('waistReference')
plug(waistReferenceVector.sout,waistReference.sin)
plug(waistReference.sout,taskWaist.featureDes.position)

taskWaist.feature.selec.value = '011100'
taskWaist.task.controlGain.value = 5

# --- TASK COM ---
featureCom = FeatureGeneric('featureCom')
plug(dyn.com,featureCom.errorIN)
plug(dyn.Jcom,featureCom.jacobianIN)
featureComDes = FeatureGeneric('featureComDes')
featureCom.sdes.value = 'featureComDes'
plug(comRef.ref,featureComDes.errorIN)
featureCom.selec.value = '011'

taskComPD = TaskPD('taskComPD')
taskComPD.add('featureCom')
plug(pg.dcomref,featureComDes.errordotIN)
plug(featureCom.errordot,taskComPD.errorDot)
taskComPD.controlGain.value = 40
taskComPD.setBeta(-1)

# create a task with changeable objective
# according to the position of the position of the zmp, the support zone is
# either the ankle or the toe
taskLFT=MetaDoubleTask6d('lft',dyn,'lf','left-ankle',pg.leftfootref, 'lt','left-toe',  pg.lefttoeref)
taskLFT.task.controlGain.value = 40
plug(pg.LeftFootSupportZone, taskLFT.selectorFT.selec)

taskRFT=MetaDoubleTask6d('rft',dyn,'rf','right-ankle',pg.rightfootref, 'rt','right-toe',  pg.righttoeref)
taskRFT.task.controlGain.value = 40
plug(pg.RightFootSupportZone, taskRFT.selectorFT.selec)

# ---- WAIST TASK ORIENTATION ---
#  set the orientation of the waist to be the same as the one of the foot.
taskWaistOr=MetaTask6d('waistOr',dyn,'waist','waist')
plug(taskRFT.featureDes.position,taskWaistOr.featureDes.position)
taskWaistOr.task.controlGain.value = 40
taskWaistOr.feature.selec.value = '100000'

# --- TASK Toe Zmp --------------------------------------------------
# compute the toe angle using the position of the zmp
featureToesZmp = FeatureToesHandlerZmp('featureToesZmp')
featureToesZmp.displaySignals()

taskToesZmp = Task('taskToesZmp')
taskToesZmp.add('featureToesZmp')
gainToesZmp = GainAdaptive('gainToesZmp')
gainToesZmp.set(0.1,0.1,125e3)
gainToesZmp.gain.value = 20
plug(taskToesZmp.error,gainToesZmp.error)
plug(gainToesZmp.gain,taskToesZmp.controlGain)

plug(pg.LeftToeOverlap, featureToesZmp.leftToeOverlap)
plug(pg.RightToeOverlap,featureToesZmp.rightToeOverlap)
plug(dyn.position,featureToesZmp.state)


# --- TASK Toe Pouet --------------------------------------------------
featureSuperToes = FeatureToesHandler('featureSuperToes')
featureSuperToes.dt.value = dt
taskToes = Task('taskToes')
taskToes.add('featureSuperToes')
gainToes = GainAdaptive('gainToes')
gainToes.set(0.1,0.1,125e3)
gainToes.gain.value = 20
plug(taskToes.error,gainToes.error)
plug(gainToes.gain,taskToes.controlGain)

plug(pg.LeftFootSupportZone,featureSuperToes.onLeftToe)
plug(pg.RightFootSupportZone,featureSuperToes.onRightToe)
plug(dyn.position,featureSuperToes.state)

taskHead=MetaTask6d('head',dyn,'gaze','gaze')
plug(taskRFT.featureDes.position, taskHead.featureDes.position)
taskHead.feature.selec.value = '111000'
taskHead.task.controlGain.value = 5

# --- TASK POSTURE --------------------------------------------------
# set a default position for the joints. 
featurePosition = FeatureGeneric('featurePosition')
featurePositionDes = FeatureGeneric('featurePositionDes')
featurePosition.sdes.value = 'featurePositionDes'
plug(dyn.position,featurePosition.errorIN)
featurePositionDes.errorIN.value = initialConfig['romeo']
featurePosition.jacobianIN.value = totuple( identity(robotDim) )

taskPosition = Task('taskPosition')
taskPosition.add('featurePosition')
# featurePosition.selec.value = toFlags((6,24))

gainPosition = GainAdaptive('gainPosition')
gainPosition.set(0.1,0.1,125e3)
gainPosition.gain.value = 5
plug(taskPosition.error,gainPosition.error)
plug(gainPosition.gain,taskPosition.controlGain)
featurePosition.selec.value = '000000000000001111111111111100000000000'

# Task on TaskJointLimits
# to avoid wrong behavior for the toes...
# --- TASK JL ------------------------------------------------------
taskJL = TaskJointLimits('taskJL')
plug(dyn.position,taskJL.position)
plug(dyn.lowerJl,taskJL.referenceInf)
plug(dyn.upperJl,taskJL.referenceSup)
taskJL.dt.value = dt
# taskJL.selec.value = toFlags(range(25,robotDim))
taskJL.selec.value = toFlags((31,38))

# ---- SOT ---------------------------------------------------------------------
# The solver SOTH of dyninv is used, but normally, the SOT solver should be sufficient
from dynamic_graph.sot.dyninv import SolverKine
sot = SolverKine('sot')
sot.setSize(robotDim)
sot.push('taskJL')					# force the joint value to be within the limits
sot.push(taskWaist.task.name)		# avoid the swing motion of the waist + constraint its height
sot.push(taskLFT.task.name)			# constraint the position of the left foot or toe
sot.push(taskRFT.task.name)			# constraint the position of the right foot or toe
sot.push(taskComPD.name)			
sot.push(taskWaistOr.task.name)		# constraint the waist orientation: follow the foot orientation

# Meth 1: depending of the state of the support:
#  if the foot lies on the ankle, set the joint toe hangle to 0
#  if the foot lies on the toe, do nothing (should reduce the knee velocity)
# requires the additional task taskJL
sot.push('taskToes')

# Meth 2: setting the toe angle using the zmp pos in the toe surface
# sot.push('taskToesZmp')

# Stun the upper part of the body.
sot.push(taskHead.task.name)		# constraint the head orientation: look straight ahead
sot.push('taskPosition')			# stun the arms.

plug(sot.control,robot.control)

# --- HERDT PG AND START -------------------------------------------------------
# Set the algorithm generating the ZMP reference trajectory to Herdt's one.
pg.parseCmd(':SetAlgoForZmpTrajectory Herdt')
pg.parseCmd(':doublesupporttime 0.1')
pg.parseCmd(':singlesupporttime 0.7')
# When velocity reference is at zero, the robot stops all motion after n steps
pg.parseCmd(':numberstepsbeforestop 4')
# Set constraints on XY
# pg.parseCmd(':setfeetconstraint XY 0.04 0.04')

# The next command must be runned after a OpenHRP.inc ... ???
# Start the robot with a speed of 0.1 m/0.8 s.
pg.parseCmd(':HerdtOnline 0.1 0.0 0.0')

# You can now modifiy the speed of the robot using set pg.velocitydes [3]( x, y, yaw)
pg.velocitydes.value =(0.1,0.0,0.0)
