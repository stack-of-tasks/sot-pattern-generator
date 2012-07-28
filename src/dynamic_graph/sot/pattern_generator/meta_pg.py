from dynamic_graph.sot.core import *
from dynamic_graph import plug
from dynamic_graph.sot.pattern_generator import PatternGenerator,Selector
from dynamic_graph.sot.dynamics import *

class MetaPG:
    def __init__(self,dyn):
        self.pg = PatternGenerator('pg')

        modelDir = dyn.getProperty('vrmlDirectory')
        modelName = dyn.getProperty('vrmlMainFile')
        specificitiesPath = dyn.getProperty('xmlSpecificityFile')
        jointRankPath = dyn.getProperty('xmlRankFile')
        robotDim = len(dyn.position.value)
        #print(modelDir,modelName,specificitiesPath,jointRankPath,robotDim)

        self.pg.setVrmlDir(modelDir+'/')
        self.pg.setVrml(modelName)
        self.pg.setXmlSpec(specificitiesPath)
        self.pg.setXmlRank(jointRankPath)
        self.pg.buildModel()

        # Standard initialization
        self.pg.parseCmd(":samplingperiod 0.005")
        self.pg.parseCmd(":previewcontroltime 1.6")
        self.pg.parseCmd(":comheight 0.814")
        self.pg.parseCmd(":omega 0.0")
        self.pg.parseCmd(":stepheight 0.05")
        self.pg.parseCmd(":singlesupporttime 0.780")
        self.pg.parseCmd(":doublesupporttime 0.020")
        self.pg.parseCmd(":armparameters 0.5")
        self.pg.parseCmd(":LimitsFeasibility 0.0")
        self.pg.parseCmd(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
        self.pg.parseCmd(":TimeDistributeParameters 2.0 3.5 1.0 3.0")
        self.pg.parseCmd(":UpperBodyMotionParameters 0.0 -0.5 0.0")
        self.pg.parseCmd(":comheight 0.814")
        self.pg.parseCmd(":SetAlgoForZmpTrajectory Morisawa")

        plug(dyn.position,self.pg.position)
        plug(dyn.com,self.pg.com)
        self.pg.motorcontrol.value = robotDim*(0,)
        self.pg.zmppreviouscontroller.value = (0,0,0)

        self.pg.initState()

        # --- PG INIT FRAMES ---
        self.geom = Dynamic("geom")
        self.geom.setFiles(modelDir, modelName,specificitiesPath,jointRankPath)
        self.geom.parse()
        self.geom.createOpPoint('rf','right-ankle')
        self.geom.createOpPoint('lf','left-ankle')
        plug(dyn.position,self.geom.position)
        self.geom.ffposition.value = 6*(0,)
        self.geom.velocity.value = robotDim*(0,)
        self.geom.acceleration.value = robotDim*(0,)

        # --- Selector of Com Ref: when pg is stopped, pg.inprocess becomes 0
        self.comRef = Selector('comRef'
                               ,['vector','ref',dyn.com,self.pg.comref])
        plug(self.pg.inprocess,self.comRef.selec)

        self.selecSupportFoot = Selector('selecSupportFoot'
                                         ,['matrixHomo','pg_H_sf'
                                           ,self.pg.rightfootref
                                           ,self.pg.leftfootref]
                                         ,['matrixHomo','wa_H_sf'
                                           ,self.geom.rf,self.geom.lf])
        plug(self.pg.SupportFoot,self.selecSupportFoot.selec)
        self.sf_H_wa = Inverse_of_matrixHomo('sf_H_wa')
        plug(self.selecSupportFoot.wa_H_sf,self.sf_H_wa.sin)
        self.pg_H_wa = Multiply_of_matrixHomo('pg_H_wa')
        plug(self.selecSupportFoot.pg_H_sf,self.pg_H_wa.sin1)
        plug(self.sf_H_wa.sout,self.pg_H_wa.sin2)

        # --- Compute the ZMP ref in the Waist reference frame.
        self.wa_H_pg = Inverse_of_matrixHomo('wa_H_pg')
        plug(self.pg_H_wa.sout,self.wa_H_pg.sin)
        self.wa_zmp = Multiply_matrixHomo_vector('wa_zmp')
        plug(self.wa_H_pg.sout,self.wa_zmp.sin1)
        plug(self.pg.zmpref,self.wa_zmp.sin2)

        # --- Build the converter object for the waist task
        self.waistReferenceVector = Stack_of_vector('waistReferenceVector')
        plug(self.pg.initwaistposref,self.waistReferenceVector.sin1)
        plug(self.pg.initwaistattref,self.waistReferenceVector.sin2)
        self.waistReferenceVector.selec1(0,3)
        self.waistReferenceVector.selec2(0,3)

        self.waistReference=PoseRollPitchYawToMatrixHomo('waistReference')
        plug(self.waistReferenceVector.sout,self.waistReference.sin)

    def plugZmp(self,robot):
        # Connect the ZMPref to OpenHRP in the waist reference frame.
        self.pg.parseCmd(':SetZMPFrame world')
        plug(self.wa_zmp.sout,robot.zmp)


    def plugWaistTask(self,taskWaist):
        plug(self.waistReference.sout,taskWaist.featureDes.position)

    def startHerdt(self):
        self.pg.parseCmd(':SetAlgoForZmpTrajectory Herdt')
        self.pg.parseCmd(':doublesupporttime 0.1')
        self.pg.parseCmd(':singlesupporttime 0.7')
        # When velocity reference is at zero, the robot stops all motion after n steps
        self.pg.parseCmd(':numberstepsbeforestop 2')
        # Set constraints on XY
        self.pg.parseCmd(':setfeetconstraint XY 0.09 0.06')
        # The next command must be runned after a OpenHRP.inc ... ???
        # Start the robot with a speed of 0.1 m/0.8 s.
        self.pg.parseCmd(':HerdtOnline 0.1 0.0 0.0')
