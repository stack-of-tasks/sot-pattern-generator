import sys
sys.path.append('../../sot-dyninv/python')


from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from MetaTask6d import MetaTask6d,toFlags
from attime import attime

from robotSpecific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor
robotName = 'hrp10small'

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
    attime.run(robot.control.time+1)
    robot.increment(dt)
    tr.triger.recompute( robot.control.time )
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
def next(): runner.once()

# --- DYN ----------------------------------------------------------------------
# --- DYN ----------------------------------------------------------------------
# --- DYN ----------------------------------------------------------------------

modelDir = pkgDataRootDir[robotName]
xmlDir = pkgDataRootDir[robotName]
specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
jointRankPath = xmlDir + '/HRP2LinkJointRankSmall.xml'

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

# --- Task 6D ------------------------------------------
class MetaTaskKine6d( MetaTask6d ):
    def createTask(self):
        self.task = Task('task'+self.name)

    def createGain(self):
        self.gain = GainAdaptive('gain'+self.name)
        self.gain.set(0.1,0.1,125e3)
    def plugEverything(self):
        self.feature.sdes.value = self.featureDes.name
        plug(self.dyn.signal(self.opPoint),self.feature.signal('position'))
        plug(self.dyn.signal('J'+self.opPoint),self.feature.signal('Jq'))
        self.task.add(self.feature.name)
        plug(self.task.error,self.gain.error)
        plug(self.gain.gain,self.task.controlGain)
    def keep(self):
        self.feature.position.recompute(self.dyn.position.time)
        self.feature.keep()

# Task right hand
taskRH=MetaTaskKine6d('rh',dyn,'rh','right-wrist')
taskLH=MetaTaskKine6d('lh',dyn,'lh','left-wrist')
taskRF=MetaTaskKine6d('rf',dyn,'rf','right-ankle')
taskLF=MetaTaskKine6d('lf',dyn,'lf','left-ankle')

taskRH.ref = ((0,0,-1,0.22),(0,1,0,-0.37),(1,0,0,.74),(0,0,0,1))

# --- TASK COM ------------------------------------------------------
dyn.setProperty('ComputeCoM','true')

featureCom    = FeatureGeneric('featureCom')
featureComDes = FeatureGeneric('featureComDes')
plug(dyn.com,featureCom.errorIN)
plug(dyn.Jcom,featureCom.jacobianIN)
featureCom.sdes.value = 'featureComDes'
featureComDes.errorIN.value = (0.0478408688115,-0.0620357207995,0.684865189311)

taskCom = Task('taskCom')
taskCom.add('featureCom')

gCom = GainAdaptive('gCom')
plug(taskCom.error,gCom.error)
plug(gCom.gain,taskCom.controlGain)
gCom.set(1,1,1)

# --- shortcuts -------------------------------------------------
q=dyn.position
comref=featureComDes.errorIN

@optionalparentheses
def iter():         print 'iter = ',robot.state.time
@optionalparentheses
def status():       print runner.isPlay

# --- PG ---------------------------------------------------------


# --- A FIRST MOTION ---------------------------------------------
