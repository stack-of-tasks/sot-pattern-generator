# --- PG ---------------------------------------------------------
from dynamic_graph import plug
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix, Inverse_of_matrixHomo, Multiply_of_matrixHomo, Stack_of_vector, PoseRollPitchYawToMatrixHomo, MatrixHomoToPoseRollPitchYaw, Multiply_matrixHomo_vector
from dynamic_graph.sot.dynamics import Dynamic
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.pattern_generator import PatternGenerator,Selector
from dynamic_graph.sot.core.matrix_util import matrixToTuple
# from dynamic_graph.sot.core import FeatureGeneric, FeaturePoint6d, Task, TaskPD
from dynamic_graph.sot.core import FeaturePosture
from dynamic_graph.ros import RosRobotModel


from numpy import *
def totuple( a ):
    al=a.tolist()
    res=[]
    for i in range(a.shape[0]):
        res.append( tuple(al[i]) )
    return tuple(res)

def addPgToRobot(robot):
  modelDir=robot.modelDir+'/'
  robotName=robot.modelName
  specificitiesPath=robot.specificitiesPath
  jointRankPath=robot.jointRankPath

  robot.pg = PatternGenerator('pg')
  robot.pg.setVrmlDir(modelDir+'/')
  robot.pg.setVrml(robotName)
  robot.pg.setXmlSpec(specificitiesPath)
  robot.pg.setXmlRank(jointRankPath)
  print "At this stage"
  robot.pg.buildModel()

  # Standard initialization
  robot.pg.parseCmd(":samplingperiod 0.005")
  robot.pg.parseCmd(":previewcontroltime 1.6")
  robot.pg.parseCmd(":walkmode 0")
  robot.pg.parseCmd(":omega 0.0")
  robot.pg.parseCmd(":stepheight 0.05")
  robot.pg.parseCmd(":singlesupporttime 0.780")
  robot.pg.parseCmd(":doublesupporttime 0.020")
  robot.pg.parseCmd(":armparameters 0.5")
  robot.pg.parseCmd(":LimitsFeasibility 0.0")
  robot.pg.parseCmd(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
  robot.pg.parseCmd(":TimeDistributeParameters 2.0 3.5 1.0 3.0")
  robot.pg.parseCmd(":UpperBodyMotionParameters 0.0 -0.5 0.0")
  robot.pg.parseCmd(":comheight 0.814")
  robot.pg.parseCmd(":SetAlgoForZmpTrajectory Morisawa")

  plug(robot.dynamic.position,robot.pg.position)
  plug(robot.com, robot.pg.com)
  plug(robot.dynamic.signal('left-ankle'), robot.pg.leftfootcurrentpos)
  plug(robot.dynamic.signal('right-ankle'), robot.pg.rightfootcurrentpos)
  robotDim = len(robot.dynamic.velocity.value)
  robot.pg.motorcontrol.value = robotDim*(0,)
  robot.pg.zmppreviouscontroller.value = (0,0,0)

  robot.pg.initState()

def addPgToUrdfRobot(robot):
  robot.pg = PatternGenerator('pg')
  robot.pg.setUrdfDir(robot.urdfDir)
  robot.pg.setUrdf(robot.urdfName)
  robot.pg.setSoleParameters(robot.ankleLength, robot.ankleWidth)
  robot.pg.addJointMapping('BODY','body')

  print "At this stage"
  robot.pg.buildModelUrdf()

  # Standard initialization
  robot.pg.parseCmd(":samplingperiod 0.005")
  robot.pg.parseCmd(":previewcontroltime 1.6")
  robot.pg.parseCmd(":walkmode 0")
  robot.pg.parseCmd(":omega 0.0")
  robot.pg.parseCmd(":stepheight 0.05")
  robot.pg.parseCmd(":singlesupporttime 0.780")
  robot.pg.parseCmd(":doublesupporttime 0.020")
  robot.pg.parseCmd(":armparameters 0.5")
  robot.pg.parseCmd(":LimitsFeasibility 0.0")
  robot.pg.parseCmd(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
  robot.pg.parseCmd(":TimeDistributeParameters 2.0 3.5 1.0 3.0")
  robot.pg.parseCmd(":UpperBodyMotionParameters 0.0 -0.5 0.0")
  robot.pg.parseCmd(":comheight 0.814")
  robot.pg.parseCmd(":SetAlgoForZmpTrajectory Morisawa")

  plug(robot.dynamic.position,robot.pg.position)
  plug(robot.com, robot.pg.com)
  plug(robot.dynamic.signal('left-ankle'), robot.pg.leftfootcurrentpos)
  plug(robot.dynamic.signal('right-ankle'), robot.pg.rightfootcurrentpos)
  robotDim = len(robot.dynamic.velocity.value)
  robot.pg.motorcontrol.value = robotDim*(0,)
  robot.pg.zmppreviouscontroller.value = (0,0,0)

  robot.pg.initState()


def addPgTaskToRobot(robot,solver):
  # --- ROBOT.PG INIT FRAMES ---
  robot.geom = Dynamic("geom")
  print("modelDir: ",robot.modelDir)
  print("modelName:",robot.modelName)
  print("specificitiesPath:",robot.specificitiesPath)
  print("jointRankPath:",robot.jointRankPath)

  robot.geom.setFiles(robot.modelDir, robot.modelName,robot.specificitiesPath,robot.jointRankPath)
  robot.geom.parse()

def addPgTaskToUrdfRobot(robot,solver):
  # --- ROBOT.PG INIT FRAMES ---
  robot.geom = RosRobotModel("geom")
  robot.geom.loadFromParameterServer()

def initRobotGeom(robot):
  robot.geom.createOpPoint('rf2','right-ankle')
  robot.geom.createOpPoint('lf2','left-ankle')
  plug(robot.dynamic.position,robot.geom.position)
  robot.geom.ffposition.value = 6*(0,)
  robotDim = len(robot.dynamic.velocity.value)
  robot.geom.velocity.value = robotDim * (0,)
  robot.geom.acceleration.value = robotDim * (0,)

def initZMPRef(robot):
  # --- Selector of Com Ref: when robot.pg is stopped, pg.inprocess becomes 0
  robot.comSelector = Selector('comSelector',['vector', 'ref', robot.com,
                                              robot.pg.comref])
  plug(robot.pg.inprocess,robot.comSelector.selec)

  selecSupportFoot = Selector('selecSupportFoot' \
       ,['matrixHomo','pg_H_sf',robot.pg.rightfootref,robot.pg.leftfootref] \
       ,['matrixHomo','wa_H_sf',robot.geom.rf2,robot.geom.lf2])

  robot.addTrace(robot.pg.name,'rightfootref')
  robot.addTrace(robot.pg.name,'leftfootref')
  robot.addTrace(robot.pg.name,'comref')
  robot.addTrace(robot.pg.name,'zmpref')
  robot.addTrace(robot.pg.name,'inprocess')
  robot.addTrace(robot.device.name,'forceLLEG')
  robot.addTrace(robot.device.name,'forceRLEG')

  plug(robot.pg.SupportFoot,selecSupportFoot.selec)
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
  plug(robot.pg.zmpref,wa_zmp.sin2)
  # Connect the ZMPref to OpenHRP in the waist reference frame.
  robot.pg.parseCmd(':SetZMPFrame world')
  plug(robot.pg.zmpref,robot.device.zmp)

  robot.addTrace(robot.device.name,'zmp')
  robot.addTrace(pg_H_wa.name,'sout')

def initWaistCoMTasks(robot):
  # ---- TASKS -------------------------------------------------------------------
  # Make sure that the CoM is not controlling the Z
  robot.featureCom.selec.value='011'

  # Build the reference waist pos homo-matrix from PG.

  # Build a left foot roll pitch yaw representation from left foot current pos.
  curLeftPRPY = MatrixHomoToPoseRollPitchYaw('curLeftPRPY')
  plug(robot.dynamic.signal('left-ankle'),curLeftPRPY.sin)
  selecRPYfromCurLeftPRPY = Selec_of_vector('selecRPYfromCurLeftPRPY')
  selecRPYfromCurLeftPRPY.selec(3,6);

  plug(curLeftPRPY.sout,selecRPYfromCurLeftPRPY.sin)

  curRightPRPY = MatrixHomoToPoseRollPitchYaw('curRightPRPY')
  plug(robot.dynamic.signal('right-ankle'),curRightPRPY.sin)
  selecRPYfromCurRightPRPY = Selec_of_vector('selecRPYfromCurRightPRPY')
  selecRPYfromCurRightPRPY.selec(3,6);

  plug(curRightPRPY.sout,selecRPYfromCurRightPRPY.sin)

  addLeftRightRPY = Add_of_vector('addLeftRightRPY')
  plug(selecRPYfromCurLeftPRPY.sout,addLeftRightRPY.sin1)
  plug(selecRPYfromCurLeftPRPY.sout,addLeftRightRPY.sin2)
  
  mulLeftRightRPY = Multiply_double_vector('mulLeftRightRPY')
  mulLeftRightRPY.sin1.value=0.5
  plug(addLeftRightRPY.sout,mulLeftRightRPY.sin2)

  YawFromLeftRightRPY = Multiply_matrix_vector('YawFromLeftRightRPY')
  YawFromLeftRightRPY.sin1.value=matrixToTuple(array([[ 0.,  0.,  0.], \
       [ 0.,  0.,  0.,  ],
       [ 0.,  0.,  1.,  ]]))
  plug(mulLeftRightRPY.sout,YawFromLeftRightRPY.sin2)

  # Build a reference vector from init waist pos and 
  # init left foot roll pitch representation
  waistReferenceVector = Stack_of_vector('waistReferenceVector')
  plug(robot.pg.initwaistposref,waistReferenceVector.sin1)
  #plug(robot.pg.initwaistattref,waistReferenceVector.sin2)
  plug(YawFromLeftRightRPY.sout,waistReferenceVector.sin2)

  waistReferenceVector.selec1(0,3)
  waistReferenceVector.selec2(0,3)
  waistReference=PoseRollPitchYawToMatrixHomo('waistReference')

  # Controlling also the yaw.
  robot.waist.selec.value = '111100'

  robot.addTrace(waistReference.name,'sout')
  robot.addTrace(robot.geom.name,'position')
  robot.addTrace(robot.pg.name,'initwaistposref')
  plug(waistReferenceVector.sout,waistReference.sin)
  plug(waistReference.sout,robot.waist.reference)

  robot.tasks ['waist'].controlGain.value = 200


def initFeetTask(robot):
  robot.selecFeet = Selector('selecFeet',
                             ['matrixHomo','leftfootref', \
                               robot.dynamic.signal('left-ankle'),\
                               robot.pg.leftfootref], \
                             ['matrixHomo','rightfootref', \
                              robot.dynamic.signal('right-ankle'), \
                              robot.pg.rightfootref])

  plug(robot.pg.inprocess,robot.selecFeet.selec)
  robot.tasks['right-ankle'].controlGain.value = 180
  robot.tasks['left-ankle'].controlGain.value = 180

  print "After Task for Right and Left Feet"

def removeDofUsed(jacobian, target):
  for i in range(0,len(jacobian)):
    for j in range(6,len(jacobian[i])):
      if jacobian[i][j] != 0:
        target[j- 6] = False
  return target

def initPostureTask(robot):
  # --- TASK POSTURE --------------------------------------------------
  # set a default position for the joints. 
  robot.features['featurePosition'] = FeaturePosture('featurePosition')
  plug(robot.device.state,robot.features['featurePosition'].state)
  robotDim = len(robot.dynamic.velocity.value)
  robot.features['featurePosition'].posture.value = robot.halfSitting

  # Remove the dofs of the feet.
  postureTaskDofs = [True] * (len(robot.dynamic.position.value) - 6)
  jla = robot.dynamic.signal('Jleft-ankle').value
  postureTaskDofs = removeDofUsed(jla, postureTaskDofs)
  jra = robot.dynamic.signal('Jright-ankle').value
  postureTaskDofs = removeDofUsed(jra, postureTaskDofs)

  for dof,isEnabled in enumerate(postureTaskDofs):
    robot.features['featurePosition'].selectDof(dof+6,isEnabled)
    
  robot.tasks['robot_task_position']=Task('robot_task_position')
  robot.tasks['robot_task_position'].add('featurePosition')

  gainPosition = GainAdaptive('gainPosition')
  gainPosition.set(0.1,0.1,125e3)
  gainPosition.gain.value = 5
  plug(robot.tasks['robot_task_position'].error,gainPosition.error)
  plug(gainPosition.gain,robot.tasks['robot_task_position'].controlGain)
  
def pushTasks(robot,solver):
  # --- TASK COM ---
  plug(robot.pg.dcomref,robot.comdot)
  robot.addTrace (robot.pg.name, 'dcomref')
  plug(robot.comSelector.ref, robot.comRef)

  # --- Plug foot ref ---
  plug(robot.pg.rightfootref,robot.rightAnkle.reference)
  plug(robot.pg.leftfootref,robot.leftAnkle.reference)

  solver.push(robot.tasks['waist'])
  solver.push(robot.tasks['robot_task_position'])
  robot.tasks['com'].controlGain.value = 180

def createGraph(robot,solver):
  initRobotGeom(robot)
  initZMPRef(robot)
  initWaistCoMTasks(robot)
  initFeetTask(robot)
  initPostureTask(robot)
  pushTasks(robot,solver)

def CreateEverythingForPG(robot,solver):
  robot.initializeTracer()
  addPgToRobot(robot)
  addPgTaskToRobot(robot,solver)
  createGraph(robot,solver)

def CreateEverythingForPGwithUrdf(robot,solver):
  robot.initializeTracer()
  addPgToUrdfRobot(robot)
  addPgTaskToUrdfRobot(robot,solver)
  createGraph(robot,solver)

def walkFewSteps(robot):
  robot.pg.parseCmd(":stepseq 0.0 0.1025 0.0 0.17 -0.205 0.0 0.17 0.205 0.0 0.17 -0.205 0.0 0.17 0.205 0.0 0.0 -0.205 0.0")

def walkFewStepsCircular(robot):
  robot.pg.parseCmd(":stepseq 0.0 0.1025 0.0 0.1 -0.205 10.0 0.1 0.205 10.0 0.1 -0.205 10.0 0.1 0.205 10.0 0.0 -0.205 0.0")

def walkAndrei(robot):
  robot.startTracer()
  robot.pg.parseCmd(":SetAlgoForZmpTrajectory Herdt")
  robot.pg.parseCmd(":doublesupporttime 0.1")
  robot.pg.parseCmd(":singlesupporttime 0.8")
  robot.pg.velocitydes.value=(0.01,0.0,0.0)
  robot.pg.parseCmd(":numberstepsbeforestop 4")
  robot.pg.parseCmd(":setfeetconstraint XY 0.02 0.02")
  robot.pg.parseCmd(":setVelReference 0.01 0.0 0.0")
  robot.pg.parseCmd(":HerdtOnline 0.01 0.0 0.0")
  if robot.device.name == 'HRP2LAAS' or \
     robot.device.name == 'HRP2JRL':
    robot.pg.parseCmd(":setfeetconstraint XY 0.09 0.06")
  elif robot.device.name == 'HRP4LIRMM':
    robot.pg.parseCmd(":setfeetconstraint XY 0.04 0.04")
  elif robot.device.name == 'ROMEO':
    robot.pg.parseCmd(":setfeetconstraint XY 0.04 0.04")

