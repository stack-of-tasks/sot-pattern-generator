/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2008
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      PatternGenerator.cpp
 * Project:   SOT
 * Author:    Olivier Stasse
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#include <iostream>

#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl/dynamics/dynamicsfactory.hh>

#ifdef WITH_HRP2DYNAMICS
#include <hrp2Dynamics/hrp2Opthumanoid-dynamic-robot.hh>
#endif

//#define VP_DEBUG
//#define VP_DEBUG_MODE 10

#include <sot-pattern-generator/pg.h>
#include <sot-core/debug.h>
#include <dynamic-graph/factory.h>
#include <sot-core/matrix-homogeneous.h>

using namespace std;
using namespace sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PatternGenerator,"PatternGenerator");

// some usefull macros
#define ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL(functionName, typeName, attribute, debugLevel) 			\
typeName & PatternGenerator::functionName(typeName & res, int time)		\
{							\
  sotDEBUGIN(debugLevel);	\
  OneStepOfControlS(time);	\
  res = attribute;			\
  sotDEBUGOUT(debugLevel);	\
  return res;				\
}


#define ACCESSOR_BUILDER_WITHOUT_ONESTEP_OF_CONTROL(functionName, typeName, attribute, debugLevel) 			\
typeName & PatternGenerator::functionName(typeName & res, int /*time*/)		\
{							\
  sotDEBUGIN(debugLevel);	\
  res = attribute;			\
  sotDEBUGOUT(debugLevel);	\
  return res;				\
}

#define PG_BIND(functionName, signalName) 					\
  boost::bind(&PatternGenerator::functionName,this,_1,_2),	\
	OneStepOfControlS,										\
	"sotPatternGenerator("+name+")::" + signalName

// some methods
namespace detail
{
  MatrixRotation skewMatrix(double x, double y, double z);
  MatrixRotation doubleSkewMatrix (double x, double y, double z);
}

// start the definition
PatternGenerator::
PatternGenerator( const std::string & name ) 
  :Entity(name)
  //  ,m_PGI(NULL)
  ,m_PreviewControlParametersFile()
  ,m_vrmlDirectory()
  ,m_vrmlMainFile()
  ,m_xmlSpecificitiesFile()
  ,m_xmlRankFile()

  ,m_init(false)
  ,m_InitPositionByRealState(true)
  ,firstSINTERN( boost::bind(&PatternGenerator::InitOneStepOfControl,this,_1,_2),
		 sotNOSIGNAL,"PatternGenerator("+name+")::intern(dummy)::init" ) 

  ,OneStepOfControlS( boost::bind(&PatternGenerator::OneStepOfControl,this,_1,_2),
		      firstSINTERN << jointPositionSIN ,"PatternGenerator("+name+")::onestepofcontrol" ) 

  ,m_dataInProcess(0)
  ,jointPositionSIN(NULL,"PatternGenerator("+name+")::input(vector)::position")

  ,motorControlJointPositionSIN(NULL,"PatternGenerator("+name+")::input(vector)::motorcontrol")

  ,ZMPPreviousControllerSIN(NULL,"PatternGenerator("+name+")::input(vector)::zmppreviouscontroller")
  ,ZMPRefSOUT(    PG_BIND(getZMPRef,   "output(vector)::zmpref") )

  // CoMRef
  ,CoMRefSOUT(    PG_BIND(getCoMRef,   "output(vector)::comref") )
  ,dCoMRefSOUT(   PG_BIND(getdCoMRef,  "output(vector)::dcomref") )
  ,ddCoMRefSOUT(  PG_BIND(getddCoMRef, "output(vector)::ddcomref") )

  ,comSIN(NULL,"PatternGenerator("+name+")::input(vector)::com")
  ,velocitydesSIN(NULL,"PatternGenerator("+name+")::input(vector)::velocitydes")

  ,LeftFootCurrentPosSIN( NULL,"PatternGenerator("+name+")::input(homogeneousmatrix)::leftfootcurrentpos")
  ,RightFootCurrentPosSIN(NULL,"PatternGenerator("+name+")::input(homogeneousmatrix)::rightfootcurrentpos")

  // Left foot
  ,LeftFootRefSOUT(    PG_BIND(getLeftFootRef,     "output(homogeneousmatrix)::leftfootref") )
  ,dotLeftFootRefSOUT( PG_BIND(getdotLeftFootRef,  "output(homogeneousmatrix)::dotleftfootref") )
  ,ddotLeftFootRefSOUT(PG_BIND(getddotLeftFootRef, "output(homogeneousmatrix)::ddotleftfootref") )

  // Right foot
  ,RightFootRefSOUT(    PG_BIND(getRightFootRef,     "output(homogeneousmatrix)::rightfootref") )
  ,dotRightFootRefSOUT( PG_BIND(getdotRightFootRef,  "output(homogeneousmatrix)::dotrightfootref") )
  ,ddotRightFootRefSOUT(PG_BIND(getddotRightFootRef, "output(homogeneousmatrix)::ddotrightfootref") )

  ,FlyingFootRefSOUT( PG_BIND(getFlyingFootRef, "output(homogeneousmatrix)::flyingfootref") )
  ,SupportFootSOUT( PG_BIND(getSupportFoot, "output(uint)::SupportFoot" ) )

  ,jointWalkingErrorPositionSOUT( PG_BIND(getjointWalkingErrorPosition, "output(vector)::walkingerrorposition") )

  ,comattitudeSOUT(   PG_BIND(getComAttitude, "output(vectorRPY)::comattitude") )
  ,dcomattitudeSOUT(  PG_BIND(getdComAttitude, "output(vectorRPY)::dcomattitude") )
  ,ddcomattitudeSOUT( PG_BIND(getddComAttitude, "output(vectorRPY)::ddcomattitude") )

  ,waistattitudeSOUT( PG_BIND(getWaistAttitude, "output(vectorRPY)::waistattitude") )
  ,waistattitudeabsoluteSOUT( PG_BIND(getWaistAttitudeAbsolute,  "output(vectorRPY)::waistattitudeabsolute") )

  ,waistpositionSOUT( PG_BIND(getWaistPosition,  "output(vector)::waistposition") )
  ,waistpositionabsoluteSOUT( PG_BIND(getWaistPositionAbsolute,  "output(vector)::waistpositionabsolute") )

  ,dataInProcessSOUT( PG_BIND(getDataInProcess,  "output(bool)::inprocess") )

  ,InitZMPRefSOUT( PG_BIND(getInitZMPRef,  "output(vector)::initzmpref") )
  ,InitCoMRefSOUT( PG_BIND(getInitCoMRef,  "output(matrix)::initcomref") )

  ,InitWaistPosRefSOUT( PG_BIND(getInitWaistPosRef,  "output(vector)::initwaistposref") )
  ,InitWaistAttRefSOUT( PG_BIND(getInitWaistAttRef,  "output(vectorRPY)::initwaistattref" )  )

  ,InitLeftFootRefSOUT( PG_BIND(getInitLeftFootRef,  "output(homogeneousmatrix)::initleftfootref") )
  ,InitRightFootRefSOUT(PG_BIND(getInitRightFootRef, "output(homogeneousmatrix)::initrightfootref") )
  
  
{
  m_MotionSinceInstanciationToThisSequence.setIdentity();

  m_LocalTime = 0;
  m_TimeStep = 0.005;

  m_ZMPRefPos.resize(4);
  m_ZMPRefPos.fill(0.0);
  m_ZMPRefPos(3) = 1.0;
  m_COMRefPos.resize(3);
  m_COMRefPos.fill(0.0);
  m_ZMPPrevious.resize(4);
  m_ZMPPrevious(3) = 1.0;
  m_dCOMRefPos.resize(3);
  m_dCOMRefPos.fill(0.0);
  m_ddCOMRefPos.resize(3);
  m_ddCOMRefPos.fill(0);
  m_InitZMPRefPos.resize(3);
  m_InitZMPRefPos.fill(0);
  m_InitCOMRefPos.resize(3);
  m_InitCOMRefPos.fill(0);
  m_InitWaistRefPos.resize(3);
  m_InitWaistRefPos.fill(0);

  m_VelocityReference.resize(3);
  m_VelocityReference.fill(0.0);
  m_WaistPosition.resize(3);
  m_WaistPosition.fill(0);
  m_WaistPositionAbsolute.resize(3);
  m_WaistPositionAbsolute.fill(0);

  m_k_Waist_kp1.setIdentity();

  m_SupportFoot = 1; // Means that we do not know which support foot it is.
  m_ReferenceFrame = WORLD_FRAME;
  m_AnkleSoilDistance = 0.0; // This value is initialized later.
  sotDEBUGIN(5);

  firstSINTERN.setDependencyType(dg::TimeDependency<int>::BOOL_DEPENDENT);
  // TODO: here, the 'setConstant' destroy the pointer toward
  // function initOneStepOfControl. By calling firstSINTERN(t), whatever t,
  // nothing will happen (well, it will just return 0).
  // To initialize firstSINTERN (without destroying the pointer), use
  // firstSINTERN.setReady() instead.
  // TODO: Remove the next line: // firstSINTERN.setConstant(0);
  firstSINTERN.setReady(true);

  //OneStepOfControlS.setDependencyType(dg::TimeDependency<int>::ALWAYS_READY);
  //  OneStepOfControlS.setConstant(0);

  signalRegistration( jointPositionSIN <<
		      motorControlJointPositionSIN <<
		      ZMPPreviousControllerSIN <<
		      ZMPRefSOUT <<
		      CoMRefSOUT <<
		      dCoMRefSOUT <<
		      ddCoMRefSOUT );

  signalRegistration( dataInProcessSOUT <<
		      LeftFootCurrentPosSIN <<
		      RightFootCurrentPosSIN << 
		      LeftFootRefSOUT <<
		      RightFootRefSOUT);

  signalRegistration( SupportFootSOUT <<
		      jointWalkingErrorPositionSOUT <<
		      waistattitudeSOUT <<
		      waistpositionSOUT <<
		      waistattitudeabsoluteSOUT <<
		      waistpositionabsoluteSOUT);

  signalRegistration( comattitudeSOUT <<
  		      dcomattitudeSOUT <<
  		      ddcomattitudeSOUT);

  signalRegistration( dotLeftFootRefSOUT <<
		      dotRightFootRefSOUT);

  signalRegistration( ddotLeftFootRefSOUT <<
		      ddotRightFootRefSOUT);
		      
  signalRegistration( InitZMPRefSOUT <<
		      InitCoMRefSOUT <<
 		      InitWaistPosRefSOUT << 
		      InitWaistAttRefSOUT << 
		      InitLeftFootRefSOUT <<
		      InitRightFootRefSOUT <<		      
		      comSIN <<
		      velocitydesSIN);

  dataInProcessSOUT.setReference( &m_dataInProcess );

  sotDEBUGOUT(5);
}

bool PatternGenerator::InitState(void)
{
  sotDEBUGIN(5); 
  // TODO
  // Instead of (0) ie .access(0), it could be rather used:
  // .accessCopy()
  // Instead of copy value (ml::Vector pos) it could be rather
  // used reference (const ml::Vector & post)
  ml::Vector res;
  if (m_InitPositionByRealState)
    {
      const ml::Vector& pos = jointPositionSIN(m_LocalTime); 

      //  m_ZMPPrevious[2] =m_AnkleSoilDistance; // Changed the reference frame.

      res.resize( pos.size()-6);
    
      for(unsigned i=0;i<res.size();i++)
	res(i) = pos(i+6);
    
      ml::Vector lZMPPrevious = ZMPPreviousControllerSIN(m_LocalTime);
      for(unsigned int i=0;i<3;i++)
	m_ZMPPrevious[i] = lZMPPrevious(i);
      
    }
  else
    {
      res = motorControlJointPositionSIN(m_LocalTime); 
    }

  ml::Vector com = comSIN(m_LocalTime); 


  m_JointErrorValuesForWalking.resize(res.size());

  sotDEBUG(5) << "m_LocalTime:" << m_LocalTime << endl;
  sotDEBUG(5) << "Joint Values:" << res << endl;

  try 
    {
	
      m_PGI->SetCurrentJointValues(res.accessToMotherLib());
      //      m_ZMPPrevious[2] = -m_AnkleSoilDistance; // Changed the reference frame.
      

      // Evaluate current position of the COM, ZMP and feet
      // according to the state of the robot.
      PatternGeneratorJRL::COMState lStartingCOMState;
      MAL_S3_VECTOR(,double) lStartingZMPPosition;
      MAL_VECTOR(,double) lWaistPosition;
      PatternGeneratorJRL::FootAbsolutePosition InitLeftFootAbsPos;
      PatternGeneratorJRL::FootAbsolutePosition InitRightFootAbsPos;

      m_PGI->EvaluateStartingState(lStartingCOMState,
				   lStartingZMPPosition,
				   lWaistPosition,
				   InitLeftFootAbsPos,
				   InitRightFootAbsPos);
      
      // Put inside sotHomogeneous representation
      m_InitCOMRefPos(0) = lStartingCOMState.x[0];
      m_InitCOMRefPos(1) = lStartingCOMState.y[0];
      m_InitCOMRefPos(2) = lStartingCOMState.z[0];
      
      m_InitZMPRefPos(0) = lStartingCOMState.x[0];
      m_InitZMPRefPos(1) = lStartingCOMState.y[0];
      m_InitZMPRefPos(2) = 0;

      if (m_InitPositionByRealState)
	{
	  m_ZMPPrevious[0] = lStartingCOMState.x[0];
	  m_ZMPPrevious[1] = lStartingCOMState.y[0];
	  m_ZMPPrevious[2] = 0;
	}
      sotDEBUG(5) << "InitZMPRefPos :" <<m_InitZMPRefPos<< endl;
      
      m_InitWaistRefPos(0) = 
	m_WaistPositionAbsolute(0) = lWaistPosition(0);
      m_InitWaistRefPos(1) = 
	m_WaistPositionAbsolute(1) = lWaistPosition(1);
      m_InitWaistRefPos(2) = 
	m_WaistPositionAbsolute(2) = lWaistPosition(2);
      
      m_InitWaistRefAtt(0) = 
	m_WaistAttitudeAbsolute(0) = lWaistPosition(3);
      m_InitWaistRefAtt(1) = 
	m_WaistAttitudeAbsolute(1) = lWaistPosition(4);
      m_InitWaistRefAtt(2) = 
	m_WaistAttitudeAbsolute(2) = lWaistPosition(5);
      
      
      FromAbsoluteFootPosToDDotHomogeneous(InitRightFootAbsPos,
					  m_InitRightFootPosition,
					  m_dotRightFootPosition,
					  m_ddotRightFootPosition
					  );
      FromAbsoluteFootPosToDDotHomogeneous(InitLeftFootAbsPos,
					  m_InitLeftFootPosition,
					  m_dotLeftFootPosition,
					  m_ddotLeftFootPosition
					  );
      ml::Vector newtmp(4),oldtmp(4);
      //InitCOMRefPos = MSI * InitCOMRefPos
      oldtmp(0) =  m_InitCOMRefPos(0); oldtmp(1) =  m_InitCOMRefPos(1);
      oldtmp(2) =  m_InitCOMRefPos(2); oldtmp(3)=1.0;
      newtmp = m_MotionSinceInstanciationToThisSequence* oldtmp;
      m_InitCOMRefPos(0) = newtmp(0);	m_InitCOMRefPos(1) = newtmp(1);
      m_InitCOMRefPos(2) = newtmp(2);

      //InitZMPRefPos = MSI * InitZMPRefPos
      oldtmp(0) =  m_InitZMPRefPos(0); oldtmp(1) =  m_InitZMPRefPos(1);
      oldtmp(2) =  m_InitZMPRefPos(2);
      newtmp = m_MotionSinceInstanciationToThisSequence* oldtmp;
      m_InitZMPRefPos(0) = newtmp(0); m_InitZMPRefPos(1) = newtmp(1);
      m_InitZMPRefPos(2) = newtmp(2);
      
	
      if (!m_InitPositionByRealState)
	{

	  MatrixHomogeneous invInitLeftFootRef;
	  m_InitLeftFootPosition.inverse(invInitLeftFootRef);
	  
	  m_k_Waist_kp1 = m_k_Waist_kp1 * invInitLeftFootRef;
	  m_MotionSinceInstanciationToThisSequence = 
	    m_MotionSinceInstanciationToThisSequence * m_k_Waist_kp1;
	}
      
      m_k_Waist_kp1 = m_InitLeftFootPosition;

      m_InitLeftFootPosition = m_MotionSinceInstanciationToThisSequence* 
	m_InitLeftFootPosition;
      m_InitRightFootPosition = m_MotionSinceInstanciationToThisSequence*
	m_InitRightFootPosition;
      
      m_LeftFootPosition = m_InitLeftFootPosition;
      m_RightFootPosition = m_InitRightFootPosition;
  
      sotDEBUG(5) << "m_InitCOMRefPos: " << m_InitCOMRefPos;
    
      sotDEBUG(5) << "m_InitZMPRefPos: " << m_InitZMPRefPos;
      sotDEBUG(5) << "m_LeftFootPosition: " << m_LeftFootPosition;
      sotDEBUG(5) << "m_RightFootPosition: " << m_RightFootPosition;
      sotDEBUG(5) << "m_MotionSinceInstanciationToThisSequence" <<
	m_MotionSinceInstanciationToThisSequence<< std::endl;

      sotDEBUG(5) << " Init Waist Ref. Position " << m_InitWaistRefPos << endl;
      sotDEBUG(5) << " Init Waist Ref. Attitude " << m_InitWaistRefAtt << endl;
  
      sotDEBUG(5) << "ILF :" <<m_InitLeftFootPosition << " "
		   << "LRF :" <<m_InitRightFootPosition << endl;
  
    }
  catch(...)
    {
      SOT_THROW ExceptionPatternGenerator( ExceptionPatternGenerator::PATTERN_GENERATOR_JRL,
					      "Error while setting the current joint values of the WPG.");
      return false;
    }

  m_InitPositionByRealState = false;
  sotDEBUGOUT(5);
  return true;
}
bool PatternGenerator::buildModel( void )
{

  // Creating the humanoid robot.
  dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
  CjrlHumanoidDynamicRobot * aHDR = 0;

#ifndef WITH_HRP2DYNAMICS
  aHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
#else
  aHDR = new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
#endif

  // Parsing the file.
  string RobotFileName = m_vrmlDirectory + m_vrmlMainFile;
  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR,RobotFileName,m_xmlRankFile,m_xmlSpecificitiesFile);

  // Initialize m_AnkleSoilDistance
  vector3d ankle;
  aHDR->leftFoot()->getAnklePositionInLocalFrame(ankle);
  m_AnkleSoilDistance = ankle[2];

  try
    {
      m_PGI = PatternGeneratorJRL::patternGeneratorInterfaceFactory(aHDR);
    }

  catch (...)
    {
      SOT_THROW ExceptionPatternGenerator( ExceptionPatternGenerator::PATTERN_GENERATOR_JRL,
					      "Error while allocating the Pattern Generator.",
					      "(PG creation process for object %s).",
					      getName().c_str());
    }

  m_init = true;
  return false;
}

PatternGenerator::
~PatternGenerator( void )
{
  sotDEBUGIN(25);
  if( 0!=m_PGI )
    {
      delete m_PGI;
      m_PGI = 0;
    }

  sotDEBUGOUT(25);
  
  return;
}


/* --- CONFIG --------------------------------------------------------------- */
/* --- CONFIG --------------------------------------------------------------- */
/* --- CONFIG --------------------------------------------------------------- */
/* --- CONFIG --------------------------------------------------------------- */
void PatternGenerator::
setVrmlDirectory( const std::string& filename )
{
  m_vrmlDirectory = filename;
}
void PatternGenerator::
setVrmlMainFile( const std::string& filename )
{
  m_vrmlMainFile = filename;
}
void PatternGenerator::
setXmlSpecificitiesFile( const std::string& filename )
{
  m_xmlSpecificitiesFile = filename;
}
void PatternGenerator::
setXmlRankFile( const std::string& filename )
{
  m_xmlRankFile = filename;
}

void PatternGenerator::
setParamPreviewFile( const std::string& filename )
{
  m_PreviewControlParametersFile = filename;
}


/* --- COMPUTE -------------------------------------------------------------- */
/* --- COMPUTE -------------------------------------------------------------- */
/* --- COMPUTE -------------------------------------------------------------- */

#include <jrl/mal/boostspecific.hh>

ml::Vector & PatternGenerator::
getZMPRef(ml::Vector & ZMPRefval, int time)
{
  sotDEBUGIN(5);

  OneStepOfControlS(time);

  ZMPRefval.resize(3);
  ZMPRefval(0) = m_ZMPRefPos(0);
  ZMPRefval(1) = m_ZMPRefPos(1);
  ZMPRefval(2) = m_ZMPRefPos(2);
  sotDEBUG(5) << "ZMPRefPos transmitted" << m_ZMPRefPos
	      << " " << ZMPRefval << endl;

  sotDEBUGOUT(5);
  return ZMPRefval;
}

int &PatternGenerator::
InitOneStepOfControl(int &dummy, int /*time*/)
{
  sotDEBUGIN(15);
  // TODO: modified first to avoid the loop.
  firstSINTERN.setReady(false);
  //  buildModel();
  // Todo: modified the order of the calls
  //OneStepOfControlS(time);
  sotDEBUGIN(15);
  return dummy;
}

void PatternGenerator::getAbsoluteWaistPosAttHomogeneousMatrix(MatrixHomogeneous &aWaistMH)
{
  
  const double cr = cos(m_WaistAttitudeAbsolute(0)); // ROLL
  const double sr = sin(m_WaistAttitudeAbsolute(0));
  const double cp = cos(m_WaistAttitudeAbsolute(1)); // PITCH
  const double sp = sin(m_WaistAttitudeAbsolute(1));
  const double cy = cos(m_WaistAttitudeAbsolute(2)); // YAW
  const double sy = sin(m_WaistAttitudeAbsolute(2));
  
  aWaistMH.fill(0.0);

  aWaistMH(0,0) = cy*cp;
  aWaistMH(0,1) = cy*sp*sr-sy*cr;
  aWaistMH(0,2) = cy*sp*cr+sy*sr;
  aWaistMH(0,3) = m_WaistPositionAbsolute(0);

  aWaistMH(1,0) = sy*cp;
  aWaistMH(1,1) = sy*sp*sr+cy*cr;
  aWaistMH(1,2) = sy*sp*cr-cy*sr;
  aWaistMH(1,3) = m_WaistPositionAbsolute(1);

  aWaistMH(2,0) = -sp;
  aWaistMH(2,1) = cp*sr;
  aWaistMH(2,2) = cp*cr;
  aWaistMH(2,3) = m_WaistPositionAbsolute(2);

  aWaistMH(3,3) = 1.0;
  
}

void PatternGenerator::FromAbsoluteFootPosToDDotHomogeneous(
							      const pg::FootAbsolutePosition & aFootPosition,
							      MatrixHomogeneous &aFootMH,
							      MatrixHomogeneous &adotFootMH,
							      MatrixHomogeneous &addotFootMH
							      )
{
  // the position.
  FromAbsoluteFootPosToHomogeneous(aFootPosition,aFootMH);

  MatrixRotation  Rot;
  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      Rot(i,j) = aFootMH(i,j);

  // the derivative.
  // dR = wx * R
  MatrixRotation velocityTwist = detail::skewMatrix( aFootPosition.domega2, aFootPosition.domega, aFootPosition.dtheta);
  MatrixRotation  dRot;
  velocityTwist.multiply(Rot,dRot);

  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      adotFootMH(i,j) = dRot(i,j);

  adotFootMH(0,3) = aFootPosition.dx;
  adotFootMH(1,3) = aFootPosition.dy;
  adotFootMH(2,3) = aFootPosition.dz;
  adotFootMH(3,3) = 1.0;

  // the second derivative.
  // ddR1 = (dw)x * R
  MatrixRotation  ddRot1;
  MatrixRotation accTwist1 = detail::skewMatrix( aFootPosition.ddomega2, aFootPosition.ddomega, aFootPosition.ddtheta);
  accTwist1.multiply(Rot,ddRot1);

  // ddR2 = wx * wx * R
  MatrixRotation  ddRot2;
  MatrixRotation accTwist2 = detail::doubleSkewMatrix( aFootPosition.domega2, aFootPosition.domega, aFootPosition.dtheta);
  accTwist2.multiply(Rot,ddRot2);

  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      addotFootMH(i,j) = ddRot1(i,j) + ddRot2(i,j);

  addotFootMH(0,3) = aFootPosition.ddx;
  addotFootMH(1,3) = aFootPosition.ddy;
  addotFootMH(2,3) = aFootPosition.ddz;
  addotFootMH(3,3) = 1.0;

}

void PatternGenerator::FromAbsoluteFootPosToDotHomogeneous(
							      const pg::FootAbsolutePosition & aFootPosition,
							      MatrixHomogeneous &aFootMH,
							      MatrixHomogeneous &adotFootMH)
{
  MatrixRotation dRot,Twist,Rot;
  adotFootMH.setIdentity();
  FromAbsoluteFootPosToHomogeneous(aFootPosition,aFootMH);

  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      Rot(i,j) = aFootMH(i,j);

  Twist = detail::skewMatrix( aFootPosition.domega2, aFootPosition.domega, aFootPosition.dtheta);
  Twist.multiply(Rot,dRot);

  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      adotFootMH(i,j) = dRot(i,j);

  adotFootMH(0,3) = aFootPosition.dx;
  adotFootMH(1,3) = aFootPosition.dy;
  adotFootMH(2,3) = aFootPosition.dz;
}

void PatternGenerator::FromAbsoluteFootPosToHomogeneous(
							   const pg::FootAbsolutePosition & aFootPosition,
							   MatrixHomogeneous &aFootMH)
{
  double c,s,co,so;
  c = cos(aFootPosition.theta*M_PI/180.0);
  s = sin(aFootPosition.theta*M_PI/180.0);

  co = cos(aFootPosition.omega*M_PI/180.0);
  so = sin(aFootPosition.omega*M_PI/180.0);
  
  aFootMH(0,0) = c*co;        aFootMH(0,1) = -s;       aFootMH(0,2) = c*so; 
  aFootMH(1,0) = s*co;        aFootMH(1,1) =  c;       aFootMH(1,2) = s*so; 
  aFootMH(2,0) = -so ;        aFootMH(2,1) =  0;       aFootMH(2,2) =   co; 
  aFootMH(3,0) = 0 ;          aFootMH(3,1) =  0;       aFootMH(3,2) =   0; 

  aFootMH(0,3) = aFootPosition.x + m_AnkleSoilDistance*so;
  aFootMH(1,3) = aFootPosition.y;
  aFootMH(2,3) = aFootPosition.z + m_AnkleSoilDistance*co;
  aFootMH(3,3) = 1.0;
}

int &PatternGenerator::
OneStepOfControl(int &dummy, int time)
{
  m_LocalTime=time;
  int lSupportFoot; // Local support foot.
  // Default value
  m_JointErrorValuesForWalking.fill(0.0);
 const int robotSize = m_JointErrorValuesForWalking.size()+6;

  try
    {
      for(unsigned int i=0;i<3;i++)
	m_ZMPRefPos(i) = m_ZMPPrevious[i];
    }
  catch(...) 
    { m_ZMPRefPos(0) = m_ZMPRefPos(1) = m_ZMPRefPos(2) = 0.0;
    m_ZMPRefPos(3) = 1.0;};
  //  m_WaistAttitudeAbsolute.fill(0);
  //  m_WaistPositionAbsolute.fill(0);

  try 
    {
      m_LeftFootPosition = LeftFootCurrentPosSIN(time);
      m_RightFootPosition = RightFootCurrentPosSIN(time);
    }
  catch (...) 
    { };

  try
    {
      m_VelocityReference = velocitydesSIN(time); 
    }
  catch(...)
    { };

  sotDEBUG(25) << "LeftFootCurrentPos:  " << m_LeftFootPosition << endl;
  sotDEBUG(25) << "RightFootCurrentPos:  " << m_RightFootPosition << endl;

  sotDEBUGIN(15);
  if (m_PGI!=0)
    {
      // TODO: Calling firstSINTERN may cause an infinite loop
      // since the function initonestepofcontrol calls without
      // control this actual function. 'Hopefully', the function
      // pointer of firstSINTERN has been earlier destroyed
      // by setconstant(0). 
      firstSINTERN(time);
      ml::Vector CurrentState = motorControlJointPositionSIN(time);
      /*! \brief Absolute Position for the left and right feet. */
      pg::FootAbsolutePosition lLeftFootPosition,lRightFootPosition;
      lLeftFootPosition.x=0.0;lLeftFootPosition.y=0.0;lLeftFootPosition.z=0.0;
      lRightFootPosition.x=0.0;lRightFootPosition.y=0.0;lRightFootPosition.z=0.0;
      /*! \brief Absolute position of the reference CoM. */
      pg::COMState lCOMRefState;
  
      sotDEBUG(45) << "mc = " << CurrentState << std::endl;
      
      MAL_VECTOR_DIM(CurrentConfiguration,double,robotSize);
      MAL_VECTOR_DIM(CurrentVelocity,double,robotSize);
      MAL_VECTOR_DIM(CurrentAcceleration,double,robotSize);
      /*! \brief Position of the reference ZMP. */
      MAL_VECTOR_DIM(ZMPTarget,double,3);
      MAL_VECTOR_FILL(ZMPTarget,0);

      sotDEBUG(25) << "Before One Step of control " << ZMPTarget[0] << " " 
		   << ZMPTarget[1] << " " 
		   << ZMPTarget[2] << endl;
      
      sotDEBUG(25) << "Before One Step of control " << lCOMRefState.x[0] << " "
      		   << lCOMRefState.y[0] << " "
      		   << lCOMRefState.z[0] << endl;
      sotDEBUG(4) << " VelocityReference " << m_VelocityReference << endl;
      
      m_PGI->setVelocityReference(m_VelocityReference(0),
				  m_VelocityReference(1),
				  m_VelocityReference(2));

      // Test if the pattern value has some value to provide.
      if (m_PGI->RunOneStepOfTheControlLoop(CurrentConfiguration,
					    CurrentVelocity,
					    CurrentAcceleration,
					    ZMPTarget,
					    lCOMRefState,
					    lLeftFootPosition,
					    lRightFootPosition))
	{
	  sotDEBUG(25) << "After One Step of control " << endl 
		       << "CurrentState:" << CurrentState << endl
		       << "CurrentConfiguration:" << CurrentConfiguration << endl;

	  m_ZMPRefPos(0) = ZMPTarget[0];
	  m_ZMPRefPos(1) = ZMPTarget[1];
	  m_ZMPRefPos(2) = ZMPTarget[2];
	  m_ZMPRefPos(3) = 1.0;
	  sotDEBUG(2) << "ZMPTarget returned by the PG: "<< m_ZMPRefPos <<endl;
	  for(int i=0;i<3;i++)
	    {
	      m_WaistPositionAbsolute(i) = CurrentConfiguration(i);
	      m_WaistAttitudeAbsolute(i) = CurrentConfiguration(i+3);
	    }
	  m_COMRefPos(0) = lCOMRefState.x[0];
	  m_COMRefPos(1) = lCOMRefState.y[0];
	  m_COMRefPos(2) = lCOMRefState.z[0];
	  sotDEBUG(2) << "COMRefPos returned by the PG: "<< m_COMRefPos <<endl;
	  m_dCOMRefPos(0) = lCOMRefState.x[1];
	  m_dCOMRefPos(1) = lCOMRefState.y[1];
	  m_dCOMRefPos(2) = lCOMRefState.z[1];

	  m_ddCOMRefPos(0) = lCOMRefState.x[2];
	  m_ddCOMRefPos(1) = lCOMRefState.y[2];
	  m_ddCOMRefPos(2) = lCOMRefState.z[2];

	  m_ComAttitude(0) = lCOMRefState.roll[0];
	  m_ComAttitude(1) = lCOMRefState.pitch[0];
	  m_ComAttitude(2) = lCOMRefState.yaw[0];

	  m_dComAttitude(0) = lCOMRefState.roll[1];
	  m_dComAttitude(1) = lCOMRefState.pitch[1];
	  m_dComAttitude(2) = lCOMRefState.yaw[1];

	  m_ddComAttitude(0) = lCOMRefState.roll[2];
	  m_ddComAttitude(1) = lCOMRefState.pitch[2];
	  m_ddComAttitude(2) = lCOMRefState.yaw[2];
	  
	  sotDEBUG(2) << "dCOMRefPos returned by the PG: "<< m_dCOMRefPos <<endl;
	  sotDEBUG(2) << "CurrentState.size()"<< CurrentState.size()<<endl;
	  sotDEBUG(2) << "CurrentConfiguration.size()"<< CurrentConfiguration.size()<<endl;
	  sotDEBUG(2) << "m_JointErrorValuesForWalking.size(): "<< m_JointErrorValuesForWalking.size() <<endl;

	  // In this setting we assume that there is a proper mapping between 
	  // CurrentState and CurrentConfiguration.
	  unsigned int SizeCurrentState = CurrentState.size();
	  unsigned int SizeCurrentConfiguration = CurrentConfiguration.size()-6;
	  unsigned int MinSize = SizeCurrentState < SizeCurrentConfiguration ?
	    SizeCurrentState : SizeCurrentConfiguration;

	  m_JointErrorValuesForWalking.fill(0.0);

	  if (m_JointErrorValuesForWalking.size()>=MinSize)
	    {
	      for(unsigned int li=0;li<MinSize;li++)
		m_JointErrorValuesForWalking(li)= (CurrentConfiguration(li+6)- CurrentState(li) )/m_TimeStep;
	      
	    }
	  else
	    {
	      std::cout <<"The state of the robot and the one returned by the WPG are different" << std::endl;
	      sotDEBUG(25) << "Size not coherent between CurrentState and m_JointErrorValuesForWalking: " 
			   << CurrentState.size()<< " " 
			   << m_JointErrorValuesForWalking.size()<< " " 
			   << endl;
	    }
	  sotDEBUG(2) << "Juste after updating m_JointErrorValuesForWalking" << endl;

	  sotDEBUG(1) << "lLeftFootPosition : " 
		      << lLeftFootPosition.x << " " 
		      << lLeftFootPosition.y << " " 
		      << lLeftFootPosition.z << " " 
		      << lLeftFootPosition.theta << endl;
	  sotDEBUG(1) << "lRightFootPosition : " 
		      << lRightFootPosition.x << " " 
		      << lRightFootPosition.y << " " 
		      << lRightFootPosition.z << " " 
		      << lRightFootPosition.theta << endl;

	  sotDEBUG(25) << "lCOMPosition : " 
		       << lCOMRefState.x[0] << " "
		       << lCOMRefState.y[0] << " "
		       << lCOMRefState.z[0] <<  endl;

	  /* Fill in the homogeneous matrix using the world reference frame*/
	  FromAbsoluteFootPosToDDotHomogeneous(lLeftFootPosition,
					      m_LeftFootPosition,
					      m_dotLeftFootPosition,
					      m_ddotLeftFootPosition
					      );
	  FromAbsoluteFootPosToDDotHomogeneous(lRightFootPosition,
					      m_RightFootPosition,
					      m_dotRightFootPosition,
					      m_ddotRightFootPosition
					      );
	
	  /* We assume that the left foot is always the origin of the new frame. */
	  m_LeftFootPosition = m_MotionSinceInstanciationToThisSequence * m_LeftFootPosition;
	  m_RightFootPosition = m_MotionSinceInstanciationToThisSequence * m_RightFootPosition;

	  // com = Motion * com
	  ml::Vector newRefPos(4), oldRefPos(4);
	  oldRefPos(0) = m_COMRefPos(0); oldRefPos(1) = m_COMRefPos(1);
	  oldRefPos(2) = m_COMRefPos(2); oldRefPos(3) = 1.0;
	  newRefPos = m_MotionSinceInstanciationToThisSequence * oldRefPos;
	  m_COMRefPos(0) = newRefPos(0);
	  m_COMRefPos(1) = newRefPos(1);
	  m_COMRefPos(2) = newRefPos(2);

	  // zmp = Motion * zmp
	  oldRefPos(0) = m_ZMPRefPos(0); oldRefPos(1) = m_ZMPRefPos(1);
	  oldRefPos(2) = m_ZMPRefPos(2); oldRefPos(3) = 1.0;
	  newRefPos = m_MotionSinceInstanciationToThisSequence * oldRefPos;
	  m_ZMPRefPos(0) = newRefPos(0);
	  m_ZMPRefPos(1) = newRefPos(1);
	  m_ZMPRefPos(2) = newRefPos(2);

	  sotDEBUG(25) << "lLeftFootPosition.stepType: " << lLeftFootPosition.stepType
		       << " lRightFootPosition.stepType: " << lRightFootPosition.stepType <<endl;
	  // Find the support foot feet. 
	  if (lLeftFootPosition.stepType==-1)
	    {
	      lSupportFoot=1;
	    }
	  else if (lRightFootPosition.stepType==-1)
	    {
	      lSupportFoot=0;
	    }
	  else /* m_LeftFootPosition.z ==m_RightFootPosition.z 
		  We keep the previous support foot half the time of the double phase..
	       */
	    {
	      lSupportFoot=m_SupportFoot;
	    }
	  
	  /* Update the class related member. */
	  m_SupportFoot = lSupportFoot;

	  if ((m_ReferenceFrame==EGOCENTERED_FRAME) ||
	      (m_ReferenceFrame==LEFT_FOOT_CENTERED_FRAME) || 
	      (m_ReferenceFrame==WAIST_CENTERED_FRAME))
	    {
	      sotDEBUG(25) << "Inside egocentered frame " <<endl;
	      MatrixHomogeneous PoseOrigin,iPoseOrigin, WaistPoseAbsolute;

	      getAbsoluteWaistPosAttHomogeneousMatrix(WaistPoseAbsolute);

	      if (m_ReferenceFrame==EGOCENTERED_FRAME)
		{
		  if (m_SupportFoot==1)
		    PoseOrigin = m_LeftFootPosition;
		  else 
		    PoseOrigin = m_RightFootPosition;	    
		}
	      else if (m_ReferenceFrame==LEFT_FOOT_CENTERED_FRAME) 
		{
		  PoseOrigin = m_LeftFootPosition;
		}
	      else if (m_ReferenceFrame==WAIST_CENTERED_FRAME)
		{
		  PoseOrigin = WaistPoseAbsolute;
		}
	      PoseOrigin.inverse(iPoseOrigin);
	      
	      sotDEBUG(25) << "Old ComRef:  " << m_COMRefPos << endl;
	      sotDEBUG(25) << "Old LeftFootRef:  " << m_LeftFootPosition << endl;
	      sotDEBUG(25) << "Old RightFootRef:  " << m_RightFootPosition << endl;
	      sotDEBUG(25) << "Old PoseOrigin:  " << PoseOrigin << endl;


	      ml::Vector lVZMPRefPos(4), lV2ZMPRefPos(4);
	      ml::Vector lVCOMRefPos(4), lV2COMRefPos(4);
	      
	      for(unsigned int li=0;li<3;li++)
		{
		  lVZMPRefPos(li) = m_ZMPRefPos(li);
		  lVCOMRefPos(li) = m_COMRefPos(li);
		}
	      lVZMPRefPos(3) = lVCOMRefPos(3) = 1.0;

	      // We do not touch to ZMP. 
	      lV2ZMPRefPos = iPoseOrigin * (WaistPoseAbsolute * lVZMPRefPos);
	      
	      // Put the CoM reference pos in the Pos Origin reference frame.
	      lV2COMRefPos = iPoseOrigin * lVCOMRefPos;
	  
	      MatrixHomogeneous lMLeftFootPosition = m_LeftFootPosition;
	      MatrixHomogeneous lMRightFootPosition = m_RightFootPosition;

	      m_LeftFootPosition = iPoseOrigin * lMLeftFootPosition;
	      m_RightFootPosition = iPoseOrigin * lMRightFootPosition;

	      for(unsigned int i=0;i<3;i++)
		{
		  m_ZMPRefPos(i) = lV2ZMPRefPos(i);
		  m_COMRefPos(i) = lV2COMRefPos(i);
		}
	      
	      MatrixHomogeneous lWaistPoseAbsoluste = WaistPoseAbsolute;
	      WaistPoseAbsolute = iPoseOrigin * WaistPoseAbsolute;

	      MatrixRotation newWaistRot;
	      WaistPoseAbsolute.extract(newWaistRot);
	      VectorRollPitchYaw newWaistRPY;
	      newWaistRPY.fromMatrix(newWaistRot);
	      m_WaistAttitude = newWaistRPY;

	      WaistPoseAbsolute.extract(m_WaistPosition);

	      sotDEBUG(25) << "ComRef:  " << m_COMRefPos << endl;
	      sotDEBUG(25) << "iPoseOrigin:  " << iPoseOrigin << endl;
	    }
	  sotDEBUG(25) << "After egocentered frame " << endl;

	      
	  sotDEBUG(25) << "ComRef:  " << m_COMRefPos << endl;
	  sotDEBUG(25) << "LeftFootRef:  " << m_LeftFootPosition << endl;
	  sotDEBUG(25) << "RightFootRef:  " << m_RightFootPosition << endl;
	  sotDEBUG(25) << "ZMPRefPos:  " << m_ZMPRefPos << endl;
	  sotDEBUG(25) << "m_MotionSinceInstanciationToThisSequence" <<
	    m_MotionSinceInstanciationToThisSequence<< std::endl;
	  
	  
	  for(unsigned int i=0;i<3;i++)
	    m_ZMPPrevious[i] = m_ZMPRefPos(i);

	  m_dataInProcess = 1;
	}
      else 
	{
	  sotDEBUG(1) << "Error while compute one step of PG." 
		      << m_dataInProcess << std::endl;
	  // TODO: SOT_THROW 
	  if (m_dataInProcess==1)
	    {
	      MatrixHomogeneous invInitLeftFootRef,Diff;
	      m_InitLeftFootPosition.inverse(invInitLeftFootRef);
	      Diff = invInitLeftFootRef * m_LeftFootPosition;

	      m_k_Waist_kp1 = m_k_Waist_kp1 * Diff;

	    }
	  m_dataInProcess = 0;

	
    
	}
      sotDEBUG(25) << "After computing error " << m_JointErrorValuesForWalking << endl;
    }
  else
    {
      m_COMRefPos = comSIN.access(time);
      m_ZMPRefPos(0) = m_COMRefPos(0);
      m_ZMPRefPos(1) = m_COMRefPos(1);
      m_ZMPRefPos(2) = 0.0;
      m_ZMPRefPos(3) = 1.0;
    }
  sotDEBUG(25) << "LeftFootRef:  " << m_LeftFootPosition << endl;
  sotDEBUG(25) << "RightFootRef:  " << m_RightFootPosition << endl;
  sotDEBUG(25) << "COMRef:  " << m_COMRefPos << endl;

  sotDEBUGOUT(15);
  return dummy;
}


/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */

void PatternGenerator::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  sotDEBUG(25) << "# In { Cmd " << cmdLine <<endl;

  std::string filename;
  if( cmdLine == "setVrmlDir" )
    {  cmdArgs>>filename; setVrmlDirectory( filename );  }
  else if( cmdLine == "setVrml" )
    {  cmdArgs>>filename; setVrmlMainFile( filename );  }
  else if( cmdLine == "setXmlSpec" )
    {  cmdArgs>>filename; setXmlSpecificitiesFile( filename );  }
  else if( cmdLine == "setXmlRank" )
    {  cmdArgs>>filename; setXmlRankFile( filename );  }
  else if( cmdLine == "setParamPreview")
    {cmdArgs>>filename; setParamPreviewFile( filename );  }
  else if( cmdLine == "setFiles" )
    {
      cmdArgs>>filename; setParamPreviewFile( filename ); 
      cmdArgs>>filename; setVrmlDirectory( filename ); 
      cmdArgs>>filename; setVrmlMainFile( filename );  
      cmdArgs>>filename; setXmlSpecificitiesFile( filename );  
      cmdArgs>>filename; setXmlRankFile( filename );  
    }
  else if( cmdLine == "displayFiles" )
    {
      cmdArgs >> ws; bool filespecified = false; 
      if( cmdArgs.good() )
	{
	  filespecified = true; 
	  std::string filetype; cmdArgs >> filetype;
	  sotDEBUG(15) << " Request: " << filetype << std::endl;
	  if( "vrmldir" == filetype ) { os << m_vrmlDirectory << std::endl; }
	  else if( "xmlspecificity" == filetype ) { os << m_xmlSpecificitiesFile << std::endl; }
	  else if( "xmlrank" == filetype ) { os << m_xmlRankFile << std::endl; }
	  else if( "vrmlmain" == filetype ) { os << m_vrmlMainFile << std::endl; }
	  else filespecified = false;
	}
      if( ! filespecified ) 
	{
	  os << "  - VRML Directory:\t\t\t" << m_vrmlDirectory <<endl
	     << "  - XML Specificities File:\t\t" << m_xmlSpecificitiesFile <<endl
	     << "  - XML Rank File:\t\t\t" << m_xmlRankFile <<endl
	     << "  - VRML Main File:\t\t\t" << m_vrmlMainFile <<endl;
	}
    }
  else if( cmdLine == "buildModel" )
    {
      if(! m_init )buildModel(); else os << "  !! Already parsed." <<endl;
    }
  else if( cmdLine == "initState" )
    {
      InitState();
    }
  else if (cmdLine == "FrameReference" )
    {
      string FrameReference;
      cmdArgs >> FrameReference;
      if (FrameReference=="World")
	{
	  m_ReferenceFrame = WORLD_FRAME;
	}
      else if (FrameReference=="Egocentered")
	{
	  m_ReferenceFrame = EGOCENTERED_FRAME;
	}
      else if (FrameReference=="LeftFootcentered")
	{
	  m_ReferenceFrame = LEFT_FOOT_CENTERED_FRAME;
	}
      else if (FrameReference=="Waistcentered")
	{
	  m_ReferenceFrame = WAIST_CENTERED_FRAME;
	}
      else 
	{
	  if (m_ReferenceFrame == EGOCENTERED_FRAME)
	    os << "Egocentered" <<endl;
	  else if (m_ReferenceFrame == WORLD_FRAME)
	    os << "World" <<endl;
	  else if (m_ReferenceFrame == LEFT_FOOT_CENTERED_FRAME)
	    os << "LeftFootcentered" <<endl;
	  else if (m_ReferenceFrame == WAIST_CENTERED_FRAME)
	    os << "Waistcentered" <<endl;
	  else 
	    os << "Something wrong reference frame ill-defined."<< endl;
	}

    }
  else if (cmdLine == "timestep")
    {
      if (cmdArgs.eof())
	{
	  os << "Timestep: " << m_TimeStep << endl;
	}
      else
	{
	  double ldt;
	  cmdArgs >> ldt;
	  if (ldt<0.0)
	    {
	      os << "\tNot a valid value for timestep.\n\tIt should be positive." <<endl;
	    }
	}
    }
  else if( cmdLine == "InitPositionByRealState" )
    { 
      if (cmdArgs.eof())
	{
	  os << "InitPositionByRealState:" << m_InitPositionByRealState <<endl;
	}
      else
	{
	  cmdArgs >> m_InitPositionByRealState;
	}
    }
  else if( cmdLine == "help" )
    {
      os << "PatternGenerator:"<<endl
	 << "  - setVrmlDir - setVrml - setXmlSpec - setXmlRank - setParamPreview <file>" <<endl
	 << "\t\t\t\t:set the config files" <<endl
	 << "  - setFiles <%1> ... <%5>\t:set files in the order cited above" <<endl
	 << "  - displayFiles\t\t\t:display the 5 config files" <<endl
	 << "  - buildModel\t\t\t:parse the files set unsing the set{Xml|Vrml} \
                  commands and create internal models." << endl
	 << "  - parsecmd\t\t\t: Command send directly to the Humanoid Walking Pattern Generator framework" << endl
	 << "  - timestep\t\t\t: without arguments display the internal time step, " << endl
	 << "     with a double dt set timestep to dt."
	 << "  - FrameReference\t\t\t: Change the reference frame used to compute the " << endl
         << "                  \t\t\t  features reference trajectories. Possible values are:"  << endl
	 << "                  \t\t\t  World, LeftFootcentered, Waistcentered, Egocentered" 
	 << "  - InitPositionByRealState:\n\t\t if true InitPosition is initialized on the real state of the robot" 
 	 << endl;

      Entity::commandLine(cmdLine,cmdArgs,os);

    }
  else if( cmdLine == "parsecmd")
    {
      if (m_PGI!=NULL)
	m_PGI->ParseCmd(cmdArgs);
	
    }
  else if( cmdLine == "addOnLineStep" )
    { 
      double x,y,th(0.0); 
      cmdArgs >> std::ws >> x  >> std::ws >> y  >> std::ws; 
      if( cmdArgs.good() ) {
	cmdArgs  >> th;
	m_PGI->AddOnLineStep(x,y,th); 
      } 
      else {
	os << "!! Error while introducing step " << x  << " " << y << " " << th << std::endl;
      }
    }
  else if( cmdLine == "addStep" )
    { 
      double x,y,th(0.0); 
      cmdArgs >> std::ws >> x  >> std::ws >> y  >> std::ws; 
      if( cmdArgs.good() ) { cmdArgs  >> th;  m_PGI->AddStepInStack(x,y,th); } 
      else { os << "!! Error while introducing step " << x  << " " << y << " " << th << std::endl; }
    }
  else if( cmdLine == "ChangeNextStep" )
    { 
      sotDEBUG(15) << "ChangeNextStep :begin"<<std::endl;
      PatternGeneratorJRL::FootAbsolutePosition aFAP;
      double NextStepTime,NextNextStepTime;
      cmdArgs >> std::ws >> NextStepTime >> std::ws >> aFAP.x  >> std::ws >> aFAP.y  >> std::ws; 
      if( cmdArgs.good() ) 
	{ 
	  cmdArgs  >> aFAP.theta;  m_PGI->ChangeOnLineStep(NextStepTime,aFAP,NextNextStepTime); 
	  os << NextNextStepTime << std::endl; ;
	} 
      else { sotDEBUG(15) << "!! Error while introducing step " << aFAP.x  
			  << " " << aFAP.y << " " << aFAP.theta << std::endl; }
      sotDEBUG(15) << "ChangeNextStep : end"<<std::endl;
    }
  else { 
    Entity::commandLine( cmdLine,cmdArgs,os); 
  }

  sotDEBUGOUT(15) ;

}


ACCESSOR_BUILDER_WITHOUT_ONESTEP_OF_CONTROL (getInitZMPRef,      ml::Vector, m_InitZMPRefPos, 25)
ACCESSOR_BUILDER_WITHOUT_ONESTEP_OF_CONTROL (getInitCoMRef,      ml::Vector, m_InitCOMRefPos, 25)
ACCESSOR_BUILDER_WITHOUT_ONESTEP_OF_CONTROL (getInitWaistPosRef, ml::Vector, m_InitWaistRefPos, 25)
ACCESSOR_BUILDER_WITHOUT_ONESTEP_OF_CONTROL (getInitWaistAttRef, VectorRollPitchYaw, m_InitWaistRefAtt, 25)
ACCESSOR_BUILDER_WITHOUT_ONESTEP_OF_CONTROL (getInitLeftFootRef,  MatrixHomogeneous, m_InitLeftFootPosition, 25)
ACCESSOR_BUILDER_WITHOUT_ONESTEP_OF_CONTROL (getInitRightFootRef, MatrixHomogeneous, m_InitRightFootPosition, 25)

ACCESSOR_BUILDER_WITHOUT_ONESTEP_OF_CONTROL (getSupportFoot, unsigned int, m_SupportFoot, 100)


ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getCoMRef,   ml::Vector, m_COMRefPos, 25)
ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getdCoMRef,  ml::Vector, m_dCOMRefPos, 25)
ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getddCoMRef, ml::Vector, m_ddCOMRefPos, 25)

ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getLeftFootRef,     MatrixHomogeneous, m_LeftFootPosition, 25)
ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getdotLeftFootRef,  MatrixHomogeneous, m_dotLeftFootPosition, 25)
ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getddotLeftFootRef, MatrixHomogeneous, m_ddotLeftFootPosition, 25)

ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getRightFootRef,     MatrixHomogeneous, m_RightFootPosition, 25)
ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getdotRightFootRef,  MatrixHomogeneous, m_dotRightFootPosition, 25)
ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getddotRightFootRef, MatrixHomogeneous, m_ddotRightFootPosition, 25)

ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getFlyingFootRef, MatrixHomogeneous, m_FlyingFootPosition, 25)

ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getjointWalkingErrorPosition, ml::Vector, m_JointErrorValuesForWalking, 5)


ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getComAttitude,   VectorRollPitchYaw, m_ComAttitude, 5)
ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getdComAttitude,  VectorRollPitchYaw, m_dComAttitude, 5)
ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getddComAttitude, VectorRollPitchYaw, m_ddComAttitude, 5)

ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getWaistAttitude,         VectorRollPitchYaw, m_WaistAttitude, 5)
ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getWaistAttitudeAbsolute, VectorRollPitchYaw, m_WaistAttitudeAbsolute, 5)

ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getWaistPosition, ml::Vector, m_WaistPosition, 5)
ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getWaistPositionAbsolute, ml::Vector, m_WaistPositionAbsolute, 5)
ACCESSOR_BUILDER_WITH_ONESTEP_OF_CONTROL (getDataInProcess, unsigned, m_dataInProcess, 5)

namespace detail
{
	MatrixRotation skewMatrix(double x, double y, double z)
	{
	  MatrixRotation Twist;
	  Twist(0,0)= 0.; Twist(0,1)= -z; Twist(0,2) = y;
	  Twist(1,0)=  z; Twist(1,1)= 0.; Twist(1,2) = x;
	  Twist(2,0)= -y; Twist(2,1)= -x; Twist(2,2) = 0.;
	  return Twist;
	}

	MatrixRotation doubleSkewMatrix (double x, double y, double z)
	{
	  double xx = x * x;
	  double yy = y * y;
	  double zz = z * z;
	  double xy = - x * y;
	  double xz = - x * z;
	  double yz = - y * z;

	  MatrixRotation Twist;
	  Twist(0,0) = yy + zz;	Twist(0,1) = xy;	  Twist(0,2) = 	xz;
	  Twist(1,0) = xy;	  	Twist(1,1) = xx + zz; Twist(1,2) = 	yz;
	  Twist(2,0) = xz;		Twist(2,1) = yz;	  Twist(2,2) = 	xx + yy;
	  return Twist;
	}
}
