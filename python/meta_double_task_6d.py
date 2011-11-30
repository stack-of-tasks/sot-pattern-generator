from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.pattern_generator import Selector

class MetaDoubleTask6d(object):
    name=''
    opPoint=''
    dyn=0
    task=0
    feature=0
    featureDes=0
    selectorFT=0

    def opPointExist(self,opPoint):
        sigsP = filter(lambda x: x.getName().split(':')[-1] == opPoint,
                       self.dyn.signals())
        sigsJ = filter(lambda x: x.getName().split(':')[-1] == 'J'+opPoint,
                       self.dyn.signals())
        return len(sigsP)==1 & len(sigsJ)==1

    def defineDynEntities(self,dyn):
        self.dyn=dyn

    def createOpPoint1(self,opPoint,opPointRef):
        self.opPoint1=opPoint
        if self.opPointExist(opPoint): return
        self.dyn.createOpPoint(opPoint,opPointRef)

    def createOpPoint2(self,opPoint,opPointRef):
        self.opPoint2=opPoint
        if self.opPointExist(opPoint): return
        self.dyn.createOpPoint(opPoint,opPointRef)

    def defineSelfSelector(self, posDes1, posDes2):
        self.selectorFT=Selector('selector'+self.name
        					 ,['matrixHomo', 'position', self.dyn.signal(self.opPoint1),     self.dyn.signal(self.opPoint2)]
        					 ,['matrixHomo', 'positionDes', posDes1, posDes2]
        					 ,['matrix'    , 'jacobian', self.dyn.signal('J'+self.opPoint1), self.dyn.signal('J'+self.opPoint2)]
                             )

    def createFeatures(self):
        self.feature    = FeaturePoint6d('feature'+self.name)
        self.featureDes = FeaturePoint6d('feature'+self.name+'_ref')
        self.feature.selec.value = '111111'
        self.feature.frame('current')
    def createTask(self):
        self.task = Task('task'+self.name)
    def createGain(self):
        self.gain = GainAdaptive('gain'+self.name)
        self.gain.set(0.1,0.1,125e3)

    def plugEverything(self):
        self.feature.sdes.value = self.featureDes.name
        plug(self.selectorFT.jacobian,self.feature.signal('Jq'))
        plug(self.selectorFT.position,self.feature.signal('position'))
        plug(self.selectorFT.positionDes, self.featureDes.position)

        self.task.add(self.feature.name)
        plug(self.task.error,self.gain.error)
        plug(self.gain.gain,self.task.controlGain)
    def keep(self):
        self.feature.position.recompute(self.dyn.position.time)
        self.feature.keep()

    def __init__(self,name,dyn,opPoint1,opPointRef1,posDes1, opPoint2,opPointRef2, posDes2):
        self.name=name
        self.defineDynEntities(dyn)
        self.createOpPoint1(opPoint1,opPointRef1)
        self.createOpPoint2(opPoint2,opPointRef2)
        self.defineSelfSelector(posDes1, posDes2)
        self.createFeatures()
        self.createTask()
        self.createGain()
        self.plugEverything()

    @property
    def ref(self):
        return self.featureDes.position.value

    @ref.setter
    def ref(self,m):
        self.featureDes.position.value = m
