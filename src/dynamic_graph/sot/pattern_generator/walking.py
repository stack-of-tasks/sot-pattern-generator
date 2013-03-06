# --- PG ---------------------------------------------------------
from dynamic_graph import plug
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix, Inverse_of_matrixHomo, Multiply_of_matrixHomo, Stack_of_vector, PoseRollPitchYawToMatrixHomo, Multiply_matrixHomo_vector
from dynamic_graph.sot.dynamics import Dynamic
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.pattern_generator import PatternGenerator,Selector
from dynamic_graph.sot.core import FeatureGeneric, FeaturePoint6d, TaskPD


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
  robot.pg.parseCmd(":comheight 0.814")
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
  plug(robot.dynamic.com,robot.pg.com)
  plug(robot.dynamic.signal('left-ankle'), robot.pg.leftfootcurrentpos)
  plug(robot.dynamic.signal('right-ankle'), robot.pg.rightfootcurrentpos)
  # plug(robot.device.motorcontrol,robot.pg.motorcontrol)
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
  robot.comRef = Selector('comRef',['vector','ref',robot.dynamic.com,robot.pg.comref])
  plug(robot.pg.inprocess,robot.comRef.selec)

  selecSupportFoot = Selector('selecSupportFoot' \
       ,['matrixHomo','pg_H_sf',robot.pg.rightfootref,robot.pg.leftfootref] \
       ,['matrixHomo','wa_H_sf',robot.geom.rf2,robot.geom.lf2])

  robot.addTrace(robot.pg.name,'rightfootref')  
  robot.addTrace(robot.pg.name,'leftfootref')        
  robot.addTrace(robot.pg.name,'comref')         
  robot.addTrace(robot.pg.name,'zmpref')
  robot.addTrace(robot.pg.name,'inprocess')         

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
  #plug(wa_zmp.sout,robot.device.zmp)
  plug(robot.pg.zmpref,robot.device.zmp)    

  robot.addTrace(robot.device.name,'zmp')
  robot.addTrace(pg_H_wa.name,'sout')

def initWaistCoMTasks(robot):
    # ---- TASKS -------------------------------------------------------------------

  # ---- WAIST TASK ---
  robot.taskWaist=MetaTask6d('waist',robot.dynamic,'waist','waist')

  # Build the reference waist pos homo-matrix from PG.
  waistReferenceVector = Stack_of_vector('waistReferenceVector')
  plug(robot.pg.initwaistposref,waistReferenceVector.sin1)
  plug(robot.pg.initwaistattref,waistReferenceVector.sin2)
  waistReferenceVector.selec1(0,3)
  waistReferenceVector.selec2(0,3)
  waistReference=PoseRollPitchYawToMatrixHomo('waistReference')
  robot.addTrace(waistReference.name,'sout')
  robot.addTrace(robot.geom.name,'position')  
  robot.addTrace(robot.pg.name,'initwaistposref')  
  plug(waistReferenceVector.sout,waistReference.sin)
  plug(waistReference.sout,robot.taskWaist.featureDes.position)

  robot.taskWaist.feature.selec.value = '011100'
  robot.taskWaist.task.controlGain.value = 200


def initFeetTask(robot):
  # --- TASK RIGHT FOOT
  # Task right hand
  #taskRF=MetaTask6d('rf',robot.dynamic,'rf','right-ankle')
  #taskLF=MetaTask6d('lf',robot.dynamic,'lf','left-ankle')

  robot.selecFeet = Selector('selecFeet',['matrixHomo','leftfootref',robot.dynamic.signal('left-ankle'),robot.pg.leftfootref],['matrixHomo','rightfootref',robot.dynamic.signal('right-ankle'),robot.pg.rightfootref])

  plug(robot.pg.inprocess,robot.selecFeet.selec)
  robot.tasks['right-ankle'].controlGain.value = 180
  robot.tasks['left-ankle'].controlGain.value = 180

  print "After Task for Right and Left Feet"

def pushTasks(robot,solver):


  # --- TASK COM ---
  plug(robot.pg.dcomref,robot.featureComDes.errordotIN)
  plug(robot.comRef.ref,robot.featureComDes.errorIN)      

  robot.taskComPD = TaskPD('taskComPD')
  robot.taskComPD.add(robot.featureCom.name)
  plug(robot.pg.dcomref,robot.featureComDes.errordotIN)
  plug(robot.featureCom.errordot,robot.taskComPD.errorDot)
  robot.taskComPD.controlGain.value = 40
  robot.taskComPD.setBeta(-1)

  # --- Plug foot ref ---
  plug(robot.pg.rightfootref,robot.features['right-ankle'].reference)
  plug(robot.pg.leftfootref,robot.features['left-ankle'].reference)

  #robot.taskRF=MetaTask6d('rf',robot.dynamic,'rf','right-ankle')
  #plug(robot.pg.rightfootref,robot.taskRF.featureDes.position)
  #robot.taskRF.task.controlGain.value = 200

  #robot.taskLF=MetaTask6d('lf',robot.dynamic,'lf','left-ankle')
  #plug(robot.pg.leftfootref,robot.taskLF.featureDes.position)
  #robot.taskLF.task.controlGain.value = 200

  #solver.sot.push(robot.taskRF.task.name)
  #solver.sot.push(robot.taskLF.task.name)
  solver.sot.remove("robot_task_com")
  solver.sot.remove("robot_task_left-ankle")
  solver.sot.remove("robot_task_right-ankle")
  solver.sot.push(robot.taskWaist.task.name)
  solver.sot.push("robot_task_left-ankle")
  solver.sot.push("robot_task_right-ankle")
  solver.sot.push(robot.taskComPD.name)


  #solver.sot.remove(robot.tasks['left-ankle'].name)
  #solver.sot.remove(robot.tasks['right-ankle'].name)

  #solver.sot.push(robot.tasks['right-ankle'].name)
  #solver.sot.push(robot.tasks['left-ankle'].name)
  #solver.sot.push(robot.comTask.name)

def createGraph(robot,solver):
  initRobotGeom(robot)
  initZMPRef(robot)
  initWaistCoMTasks(robot)
  initFeetTask(robot)
  pushTasks(robot,solver)
  

def CreateEverythingForPG(robot,solver):
  robot.initializeTracer()
  addPgToRobot(robot)
  addPgTaskToRobot(robot,solver)
  robot.addTrace(solver.sot.name,'control')         
  robot.startTracer()
  createGraph(robot,solver)

  

def walkFewSteps(robot):
  robot.pg.parseCmd(":stepseq 0.0 0.095 0.0 0.17 -0.19 0.0 0.17 0.19 0.0 0.17 -0.19 0.0 0.17 0.19 0.0 0.0 -0.19 0.0")

def walkFewStepsCircular(robot):
  robot.pg.parseCmd(":stepseq 0.0 0.1025 0.0 0.1 -0.205 10.0 0.1 0.205 10.0 0.1 -0.205 10.0 0.1 0.205 10.0 0.0 -0.205 0.0")

def walkAndrei(robot):
  robot.pg.parseCmd(":SetAlgoForZmpTrajectory Herdt")
  robot.pg.parseCmd(":doublesupporttime 0.1")
  robot.pg.parseCmd(":singlesupporttime 0.8")
  robot.pg.velocitydes.value=(0.1,0.0,0.0)
  robot.pg.parseCmd(":numberstepsbeforestop 4")
  robot.pg.parseCmd(":setfeetconstraint XY 0.02 0.02")
  robot.pg.parseCmd(":setVelReference 0.1 0.0 0.0")
  robot.pg.parseCmd(":HerdtOnline 0.1 0.0 0.0")

