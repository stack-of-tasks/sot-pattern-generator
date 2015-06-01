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

//#define VP_DEBUG
//#define VP_DEBUG_MODE 45
#include <sot/core/debug.hh>
#ifdef VP_DEBUG
 class sotPG__INIT
 {
 public:sotPG__INIT( void ) { dynamicgraph::sot::DebugTrace::openFile(); }
 };
 sotPG__INIT sotPG_initiator;
#endif //#ifdef VP_DEBUG


#include <jrl/mal/matrixabstractlayer.hh>
#include <jrl/dynamics/dynamicsfactory.hh>

#ifdef WITH_HRP2DYNAMICS
#  include <hrp2-dynamics/hrp2OptHumanoidDynamicRobot.h>
#endif

#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <sot/core/matrix-homogeneous.hh>

#include <sot-pattern-generator/pg.h>
#include <jrl/dynamics/urdf/parser.hh>

using namespace std;
namespace dynamicgraph {
  namespace sot {

    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PatternGenerator,"PatternGenerator");

    PatternGenerator::
    PatternGenerator( const std::string & name )
      :Entity(name)
       //  ,m_PGI(NULL)
      ,m_PreviewControlParametersFile()
      ,m_vrmlDirectory()
      ,m_vrmlMainFile()
      ,m_xmlSpecificitiesFile()
      ,m_xmlRankFile()
      ,m_urdfDirectory("")
      ,m_urdfMainFile("")
      ,m_soleLength(0)
      ,m_soleWidth(0)
      ,m_init(false)
      ,m_InitPositionByRealState(true)
      ,firstSINTERN( boost::bind(&PatternGenerator::InitOneStepOfControl,this,_1,_2),
		     sotNOSIGNAL,"PatternGenerator("+name+")::intern(dummy)::init" )

      ,OneStepOfControlS( boost::bind(&PatternGenerator::OneStepOfControl,this,_1,_2),
			  firstSINTERN << jointPositionSIN ,"PatternGenerator("+name+")::onestepofcontrol" )

      ,m_dataInProcess(0)
      ,m_rightFootContact(true) // It is assumed that the robot is standing.
      ,m_leftFootContact(true)
      ,jointPositionSIN(NULL,"PatternGenerator("+name+")::input(vector)::position")

      ,motorControlJointPositionSIN(NULL,"PatternGenerator("+name+")::input(vector)::motorcontrol")

      ,ZMPPreviousControllerSIN(NULL,"PatternGenerator("+name+")::input(vector)::zmppreviouscontroller")

      ,ZMPRefSOUT( boost::bind(&PatternGenerator::getZMPRef,this,_1,_2),
		   OneStepOfControlS,
		   "PatternGenerator("+name+")::output(vector)::zmpref" )

      ,CoMRefSOUT( boost::bind(&PatternGenerator::getCoMRef,this,_1,_2),
		   OneStepOfControlS,
		   "PatternGenerator("+name+")::output(matrix)::comref" )

      ,dCoMRefSOUT( boost::bind(&PatternGenerator::getdCoMRef,this,_1,_2),
		    OneStepOfControlS,
		    "PatternGenerator("+name+")::output(matrix)::dcomref" )

      ,comSIN(NULL,"PatternGenerator("+name+")::input(vector)::com")

      ,velocitydesSIN(NULL,"PatternGenerator("+name+")::input(vector)::velocitydes")

      ,LeftFootCurrentPosSIN(NULL,"PatternGenerator("+name+")::input(homogeneousmatrix)::leftfootcurrentpos")

      ,RightFootCurrentPosSIN(NULL,"PatternGenerator("+name+")::input(homogeneousmatrix)::rightfootcurrentpos")

      ,LeftFootRefSOUT( boost::bind(&PatternGenerator::getLeftFootRef,this,_1,_2),
			OneStepOfControlS,
			"PatternGenerator("+name+")::output(homogeneousmatrix)::leftfootref" )

      ,RightFootRefSOUT( boost::bind(&PatternGenerator::getRightFootRef,this,_1,_2),
			 OneStepOfControlS,
			 "PatternGenerator("+name+")::output(homogeneousmatrix)::rightfootref" )
      ,dotLeftFootRefSOUT( boost::bind(&PatternGenerator::getdotLeftFootRef,this,_1,_2),
			   OneStepOfControlS,
			   "PatternGenerator("+name+")::output(homogeneousmatrix)::dotleftfootref" )

      ,dotRightFootRefSOUT( boost::bind(&PatternGenerator::getdotRightFootRef,this,_1,_2),
			    OneStepOfControlS,
			    "PatternGenerator("+name+")::output(homogeneousmatrix)::dotrightfootref" )

      ,FlyingFootRefSOUT( boost::bind(&PatternGenerator::getFlyingFootRef,this,_1,_2),
			  OneStepOfControlS,
			  "PatternGenerator("+name+")::output(homogeneousmatrix)::flyingfootref" )


      ,SupportFootSOUT( boost::bind(&PatternGenerator::getSupportFoot,this,_1,_2),
			OneStepOfControlS,
			"PatternGenerator("+name+")::output(uint)::SupportFoot" )
      ,jointWalkingErrorPositionSOUT(boost::bind(&PatternGenerator::getjointWalkingErrorPosition,this,_1,_2),
				     OneStepOfControlS,
				     "PatternGenerator("+name+")::output(vector)::walkingerrorposition")

      ,comattitudeSOUT(boost::bind(&PatternGenerator::getComAttitude,this,_1,_2),
		       OneStepOfControlS,
		       "sotPatternGenerator("+name+")::output(vectorRPY)::comattitude")
      ,dcomattitudeSOUT(boost::bind(&PatternGenerator::getdComAttitude,this,_1,_2),
			OneStepOfControlS,
			"sotPatternGenerator("+name+")::output(vectorRPY)::dcomattitude")

      ,waistattitudeSOUT(boost::bind(&PatternGenerator::getWaistAttitude,this,_1,_2),
			 OneStepOfControlS,
			 "PatternGenerator("+name+")::output(vectorRPY)::waistattitude")
      ,waistattitudeabsoluteSOUT(boost::bind(&PatternGenerator::getWaistAttitudeAbsolute,this,_1,_2),
				 OneStepOfControlS,
				 "PatternGenerator("+name+")::output(vectorRPY)::waistattitudeabsolute")

      ,waistpositionSOUT(boost::bind(&PatternGenerator::getWaistPosition,this,_1,_2),
			 OneStepOfControlS,
			 "PatternGenerator("+name+")::output(vector)::waistposition")
      ,waistpositionabsoluteSOUT(boost::bind(&PatternGenerator::getWaistPositionAbsolute,this,_1,_2),
				 OneStepOfControlS,
				 "PatternGenerator("+name+")::output(vector)::waistpositionabsolute")

      ,dataInProcessSOUT(boost::bind(&PatternGenerator::getDataInProcess, this, _1, _2),
			 OneStepOfControlS,
			 "PatternGenerator("+name+")::output(bool)::inprocess")
      ,InitZMPRefSOUT( boost::bind(&PatternGenerator::getInitZMPRef,this,_1,_2),
		       OneStepOfControlS,
		       "PatternGenerator("+name+")::output(vector)::initzmpref" )

      ,InitCoMRefSOUT( boost::bind(&PatternGenerator::getInitCoMRef,this,_1,_2),
		       OneStepOfControlS,
		       "PatternGenerator("+name+")::output(matrix)::initcomref" )

      ,InitWaistPosRefSOUT( boost::bind(&PatternGenerator::getInitWaistPosRef,this,_1,_2),
			    OneStepOfControlS,
			    "PatternGenerator("+name+")::output(vector)::initwaistposref" )

      ,InitWaistAttRefSOUT( boost::bind(&PatternGenerator::getInitWaistAttRef,this,_1,_2),
			    OneStepOfControlS,
			    "PatternGenerator("+name+")::output(vectorRPY)::initwaistattref" )

      ,InitLeftFootRefSOUT( boost::bind(&PatternGenerator::getInitLeftFootRef,this,_1,_2),
			    OneStepOfControlS,
			    "PatternGenerator("+name+")::output(homogeneousmatrix)::initleftfootref" )

      ,InitRightFootRefSOUT( boost::bind(&PatternGenerator::getInitRightFootRef,this,_1,_2),
			     OneStepOfControlS,
			     "PatternGenerator("+name+")::output(homogeneousmatrix)::initrightfootref" )
      ,leftFootContactSOUT( boost::bind(&PatternGenerator::getLeftFootContact,this,_1,_2),
			     OneStepOfControlS,
			    "PatternGenerator("+name+")::output(bool)::leftfootcontact" )
      ,rightFootContactSOUT( boost::bind(&PatternGenerator::getRightFootContact,this,_1,_2),
			     OneStepOfControlS,
			     "PatternGenerator("+name+")::output(bool)::rightfootcontact")


    {
      m_MotionSinceInstanciationToThisSequence.setIdentity();

      m_LocalTime = 0;
      m_TimeStep = 0.005;
      m_DoubleSupportPhaseState = false;

      m_ZMPRefPos.resize(4);
      m_ZMPRefPos.fill(0.0);
      m_ZMPRefPos(3) = 1.0;
      m_COMRefPos.resize(3);
      m_COMRefPos.fill(0.0);
      m_ZMPPrevious.resize(4);
      m_ZMPPrevious(3) = 1.0;
      m_dCOMRefPos.resize(3);
      m_dCOMRefPos.fill(0.0);
      m_InitZMPRefPos.resize(3);
      m_InitZMPRefPos.fill(0);
      m_InitCOMRefPos.resize(3);
      m_InitCOMRefPos.fill(0);
      m_InitWaistRefPos.resize(3);
      m_InitWaistRefPos.fill(0);
      m_InitWaistRefAtt.resize(3);
      m_InitWaistRefAtt.fill(0);
      m_dComAttitude.resize(3);
      m_dComAttitude.fill(0);
      m_VelocityReference.resize(3);
      m_VelocityReference.fill(0.0);
      m_WaistAttitude.resize(3);
      m_WaistAttitude.fill(0);
      m_ComAttitude.resize(3);
      m_ComAttitude.fill(0);
      m_WaistPosition.resize(3);
      m_WaistPosition.fill(0);
      m_WaistAttitudeAbsolute.resize(3);
      m_WaistAttitudeAbsolute.fill(0);
      m_WaistPositionAbsolute.resize(3);
      m_WaistPositionAbsolute.fill(0);

      m_k_Waist_kp1.setIdentity();

      m_SupportFoot = 1; // Means that we do not know which support foot it is.
      m_ReferenceFrame = WORLD_FRAME;

      sotDEBUGIN(5);

      firstSINTERN.setDependencyType(TimeDependency<int>::BOOL_DEPENDENT);
      // TODO: here, the 'setConstant' destroy the pointer toward
      // function initOneStepOfControl. By calling firstSINTERN(t), whatever t,
      // nothing will happen (well, it will just return 0).
      // To initialize firstSINTERN (without destroying the pointer), use
      // firstSINTERN.setReady() instead.
      // TODO: Remove the next line: // firstSINTERN.setConstant(0);
      firstSINTERN.setReady(true);

      //OneStepOfControlS.setDependencyType(TimeDependency<int>::ALWAYS_READY);
      //  OneStepOfControlS.setConstant(0);


      OneStepOfControlS.addDependency( LeftFootCurrentPosSIN  );
      OneStepOfControlS.addDependency( RightFootCurrentPosSIN );
      OneStepOfControlS.addDependency( velocitydesSIN );
      OneStepOfControlS.addDependency( firstSINTERN );
      OneStepOfControlS.addDependency( motorControlJointPositionSIN );
      OneStepOfControlS.addDependency( comSIN );

      // For debug, register OSOC (not relevant for normal use).
      signalRegistration( OneStepOfControlS );

#if 0

     signalRegistration( jointPositionSIN <<
			  motorControlJointPositionSIN <<
			  ZMPPreviousControllerSIN <<
			  ZMPRefSOUT <<
			  CoMRefSOUT <<
			  dCoMRefSOUT);

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
			  dcomattitudeSOUT );

      signalRegistration( dotLeftFootRefSOUT <<
			  dotRightFootRefSOUT);

      signalRegistration( InitZMPRefSOUT <<
			  InitCoMRefSOUT <<
			  InitWaistPosRefSOUT <<
			  InitWaistAttRefSOUT <<
			  InitLeftFootRefSOUT <<
			  InitRightFootRefSOUT <<
			  comSIN <<
			  velocitydesSIN);
#else
      signalRegistration( dataInProcessSOUT );

      signalRegistration( jointPositionSIN <<
			  motorControlJointPositionSIN <<
			  ZMPPreviousControllerSIN <<
			  ZMPRefSOUT <<
			  CoMRefSOUT <<
			  dCoMRefSOUT);

      signalRegistration(comSIN <<
			 velocitydesSIN <<
			  LeftFootCurrentPosSIN <<
			  RightFootCurrentPosSIN <<
			  LeftFootRefSOUT <<
			  RightFootRefSOUT);

      signalRegistration( SupportFootSOUT <<
			  jointWalkingErrorPositionSOUT <<
			  comattitudeSOUT <<
			  dcomattitudeSOUT <<
			  waistattitudeSOUT );

      signalRegistration( waistpositionSOUT <<
			  waistattitudeabsoluteSOUT <<
			  waistpositionabsoluteSOUT);


      signalRegistration( dotLeftFootRefSOUT <<
			  dotRightFootRefSOUT);

      signalRegistration( InitZMPRefSOUT <<
			  InitCoMRefSOUT <<
			  InitWaistPosRefSOUT <<
			  InitWaistAttRefSOUT <<
			  InitLeftFootRefSOUT <<
			  InitRightFootRefSOUT );

      signalRegistration( leftFootContactSOUT <<
			  rightFootContactSOUT);

#endif
      initCommands();

      //dataInProcessSOUT.setReference( &m_dataInProcess );

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
	  MAL_VECTOR_TYPE(double) lWaistPosition;
      if (m_InitPositionByRealState)
	{
	  const ml::Vector& pos = jointPositionSIN(m_LocalTime);

      MAL_VECTOR_RESIZE(lWaistPosition, 6);
      for(unsigned int i = 0; i < 6; ++i)
      {
        lWaistPosition(i) = pos(i);
      }
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
	  MAL_S3_VECTOR_TYPE(double) lStartingZMPPosition;
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


	  FromAbsoluteFootPosToDotHomogeneous(InitRightFootAbsPos,
					      m_InitRightFootPosition,
					      m_dotRightFootPosition);
	  FromAbsoluteFootPosToDotHomogeneous(InitLeftFootAbsPos,
					      m_InitLeftFootPosition,
					      m_dotLeftFootPosition);
	  ml::Vector newtmp(4),oldtmp(4);
	  oldtmp(0) =  m_InitCOMRefPos(0); oldtmp(1) =  m_InitCOMRefPos(1);
	  oldtmp(2) =  m_InitCOMRefPos(2); oldtmp(3)=1.0;
	  newtmp = m_MotionSinceInstanciationToThisSequence* oldtmp;
	  m_InitCOMRefPos(0) = newtmp(0);	m_InitCOMRefPos(1) = newtmp(1);
	  m_InitCOMRefPos(2) = newtmp(2);

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

#ifndef WITH_HRP2DYNAMICS // WITH_HRP2DYNAMICS is not defined
      aHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
#else // WITH_HRP2DYNAMICS is defined
      Chrp2OptHumanoidDynamicRobot *aHRP2HDR = new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor) ;
      aHDR = aHRP2HDR ;
#endif // end "if WITH_HRP2DYNAMICS defined"

      // Parsing the file.
      string RobotFileName = m_vrmlDirectory + m_vrmlMainFile;
      dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR,RobotFileName,m_xmlRankFile,m_xmlSpecificitiesFile);
      bool ok=true;

      if (aHDR!=0)
	{
	  CjrlFoot * rightFoot = aHDR->rightFoot();
	  if (rightFoot!=0)
	    {
	      vector3d AnkleInFoot;
	      rightFoot->getAnklePositionInLocalFrame(AnkleInFoot);
	      m_AnkleSoilDistance = fabs(AnkleInFoot(2));
	    }
	  else ok=false;
	}
      else ok=false;
      
      if (!ok)
	{
	  SOT_THROW ExceptionPatternGenerator( ExceptionPatternGenerator::PATTERN_GENERATOR_JRL,
					       "Error while creating humanoid robot dynamical model.",
					       "(PG creation process for object %s).",
					       getName().c_str());
	}
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

    bool PatternGenerator::buildModelUrdf( void )
    {
      jrl::dynamics::urdf::Parser parser;

      // Creating the humanoid robot.
      dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
      CjrlHumanoidDynamicRobot * aHDR = 0;

      // Parsing the file.
      string RobotFileName = m_urdfDirectory + m_urdfMainFile;

      std::map<std::string, std::string>::const_iterator it = specialJoints_.begin();
      for (;it!=specialJoints_.end();++it) {
        parser.specifyREPName(it->first, it->second);
      }
      aHDR = parser.parse(RobotFileName);
      bool ok=true;

      if (aHDR!=0)
      	{
      	  CjrlFoot * rightFoot = aHDR->rightFoot();
      	  if (rightFoot!=0)
      	    {
      	      vector3d AnkleInFoot;
      	      rightFoot->getAnklePositionInLocalFrame(AnkleInFoot);
	            m_AnkleSoilDistance = fabs(AnkleInFoot[2]);
	            aHDR->leftFoot()->setSoleSize(m_soleLength, m_soleWidth);
	            aHDR->rightFoot()->setSoleSize(m_soleLength, m_soleWidth);
      	    }
      	  else ok=false;
      	}
      else ok=false;
      if (!ok)
      	{
      	  SOT_THROW ExceptionPatternGenerator( ExceptionPatternGenerator::PATTERN_GENERATOR_JRL,
      					       "Error while creating humanoid robot dynamical model.",
      					       "(PG creation process for object %s).",
      					       getName().c_str());
      	}
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

    void PatternGenerator::
    setUrdfDirectory( const std::string& filename )
    {
      m_urdfDirectory = filename;
    }
    void PatternGenerator::
    setUrdfMainFile( const std::string& filename )
    {
      m_urdfMainFile = filename;
    }
    void PatternGenerator::
    addJointMapping(const std::string &link, const std::string &repName)
    {
      specialJoints_[link] = repName;
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

    ml::Vector & PatternGenerator::
    getCoMRef(ml::Vector & CoMRefval, int time)
    {
      sotDEBUGIN(25);

      OneStepOfControlS(time);
      CoMRefval = m_COMRefPos;

      sotDEBUGOUT(25);
      return CoMRefval;
    }

    ml::Vector & PatternGenerator::
    getdCoMRef(ml::Vector & CoMRefval, int time)
    {
      sotDEBUGIN(25);

      OneStepOfControlS(time);
      CoMRefval = m_dCOMRefPos;

      sotDEBUGOUT(25);
      return CoMRefval;
    }

    ml::Vector & PatternGenerator::
    getInitZMPRef(ml::Vector & InitZMPRefval, int /*time*/)
    {
      sotDEBUGIN(25);

      sotDEBUG(25) << "InitZMPRefPos transmitted" << m_InitZMPRefPos
		   << " " << InitZMPRefval << std::endl;
      InitZMPRefval.resize(3);
      InitZMPRefval(0) = m_InitZMPRefPos(0);
      InitZMPRefval(1) = m_InitZMPRefPos(1);
      InitZMPRefval(2) = m_InitZMPRefPos(2);

      sotDEBUGOUT(25);
      return InitZMPRefval;
    }

    ml::Vector & PatternGenerator::
    getInitCoMRef(ml::Vector & InitCoMRefval, int /*time*/)
    {
      sotDEBUGIN(25);

      InitCoMRefval.resize(3);
      InitCoMRefval(0) = m_InitCOMRefPos(0);
      InitCoMRefval(1) = m_InitCOMRefPos(1);
      InitCoMRefval(2) = m_InitCOMRefPos(2);


      sotDEBUGOUT(25);
      return InitCoMRefval;
    }

    ml::Vector & PatternGenerator::
    getInitWaistPosRef(ml::Vector & InitWaistRefval, int /*time*/)
    {
      sotDEBUGIN(25);

      InitWaistRefval = m_InitWaistRefPos;

      sotDEBUGOUT(25);
      return InitWaistRefval;
    }
    VectorRollPitchYaw & PatternGenerator::
    getInitWaistAttRef(VectorRollPitchYaw & InitWaistRefval, int /*time*/)
    {
      sotDEBUGIN(25);

      for(unsigned int i=0;i<3;++i)
	InitWaistRefval(i) = m_InitWaistRefAtt(i);

      sotDEBUGOUT(25);
      return InitWaistRefval;
    }



    MatrixHomogeneous & PatternGenerator::
    getLeftFootRef(MatrixHomogeneous & LeftFootRefVal, int time)
    {
      sotDEBUGIN(25);

      OneStepOfControlS(time);
      LeftFootRefVal = m_LeftFootPosition;
      sotDEBUGOUT(25) ;
      return LeftFootRefVal;
    }
    MatrixHomogeneous & PatternGenerator::
    getRightFootRef(MatrixHomogeneous & RightFootRefval, int time)
    {
      sotDEBUGIN(25);

      OneStepOfControlS(time);

      RightFootRefval = m_RightFootPosition;
      sotDEBUGOUT(25);
      return RightFootRefval;
    }
    MatrixHomogeneous & PatternGenerator::
    getdotLeftFootRef(MatrixHomogeneous & LeftFootRefVal, int time)
    {
      sotDEBUGIN(25);

      OneStepOfControlS(time);
      LeftFootRefVal = m_dotLeftFootPosition;
      sotDEBUGOUT(25) ;
      return LeftFootRefVal;
    }
    MatrixHomogeneous & PatternGenerator::
    getdotRightFootRef(MatrixHomogeneous & RightFootRefval, int time)
    {
      sotDEBUGIN(25);

      OneStepOfControlS(time);

      RightFootRefval = m_dotRightFootPosition;
      sotDEBUGOUT(25);
      return RightFootRefval;
    }

    MatrixHomogeneous & PatternGenerator::
    getInitLeftFootRef(MatrixHomogeneous & LeftFootRefVal, int /*time*/)
    {
      sotDEBUGIN(25);

      LeftFootRefVal = m_InitLeftFootPosition;
      sotDEBUGOUT(25) ;
      return LeftFootRefVal;
    }
    MatrixHomogeneous & PatternGenerator::
    getInitRightFootRef(MatrixHomogeneous & RightFootRefval, int /*time*/)
    {
      sotDEBUGIN(25);

      RightFootRefval = m_InitRightFootPosition;
      sotDEBUGOUT(25);
      return RightFootRefval;
    }

    MatrixHomogeneous & PatternGenerator::
    getFlyingFootRef(MatrixHomogeneous & FlyingFootRefval, int time)
    {
      sotDEBUGIN(25);
      OneStepOfControlS(time);
      FlyingFootRefval = m_FlyingFootPosition;
      sotDEBUGOUT(25);
      return FlyingFootRefval;
    }

    bool & PatternGenerator ::
    getLeftFootContact(bool &res, int time)
    {
      sotDEBUGIN(25);
      OneStepOfControlS(time);
      res = m_leftFootContact;
      sotDEBUGOUT(25);
      return res;
      
    }

    bool & PatternGenerator ::
    getRightFootContact(bool &res, int time)
    {
      sotDEBUGIN(25);
      OneStepOfControlS(time);
      res = m_rightFootContact;
      sotDEBUGOUT(25);
      return res;
      
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

    void PatternGenerator::FromAbsoluteFootPosToDotHomogeneous(pg::FootAbsolutePosition aFootPosition,
							       MatrixHomogeneous &aFootMH,
							       MatrixHomogeneous &adotFootMH)
    {
      MatrixRotation dRot,Twist,Rot;
      adotFootMH.setIdentity();
      FromAbsoluteFootPosToHomogeneous(aFootPosition,aFootMH);

      for(unsigned int i=0;i<3;i++)
	for(unsigned int j=0;j<3;j++)
	  Rot(i,j) = aFootMH(i,j);

      Twist(0,0)=0.0; Twist(0,1)= -aFootPosition.dtheta; Twist(0,2) = aFootPosition.domega;
      Twist(1,0)= aFootPosition.dtheta; Twist(1,1)= 0.0; Twist(1,2) = aFootPosition.domega2;
      Twist(2,0)= -aFootPosition.domega; Twist(2,1)= -aFootPosition.domega2; Twist(2,2) = 0.0;

      Twist.multiply(Rot,dRot);

      for(unsigned int i=0;i<3;i++)
	for(unsigned int j=0;j<3;j++)
	  adotFootMH(i,j) = dRot(i,j);

      adotFootMH(0,3) = aFootPosition.dx;
      adotFootMH(1,3) = aFootPosition.dy;
      adotFootMH(2,3) = aFootPosition.dz;

    }

    void PatternGenerator::FromAbsoluteFootPosToHomogeneous(pg::FootAbsolutePosition aFootPosition,
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
	  assert( CurrentState.size() == robotSize );

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
	  MAL_VECTOR_DIM(ZMPTarget,double,3);
	  MAL_VECTOR_FILL(ZMPTarget,0);

	  sotDEBUG(25) << "Before One Step of control " << lCOMRefState.x[0] << " "
		       << lCOMRefState.y[0] << " " << lCOMRefState.z[0] << endl;
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

	      m_ComAttitude(0) = lCOMRefState.roll[0];
	      m_ComAttitude(1) = lCOMRefState.pitch[0];
	      m_ComAttitude(2) = lCOMRefState.yaw[0];

	      m_dComAttitude(0) = lCOMRefState.roll[1];
	      m_dComAttitude(1) = lCOMRefState.pitch[1];
	      m_dComAttitude(2) = lCOMRefState.yaw[1];

	      sotDEBUG(2) << "dCOMRefPos returned by the PG: "<< m_dCOMRefPos <<endl;
	      sotDEBUG(2) << "CurrentState.size()"<< CurrentState.size()<<endl;
	      sotDEBUG(2) << "CurrentConfiguration.size()"<< CurrentConfiguration.size()<<endl;
	      sotDEBUG(2) << "m_JointErrorValuesForWalking.size(): "<< m_JointErrorValuesForWalking.size() <<endl;

	
	      // In this setting we assume that there is a proper mapping between
	      // CurrentState and CurrentConfiguration.
	      unsigned int SizeCurrentState = CurrentState.size();
	      unsigned int SizeCurrentConfiguration = CurrentConfiguration.size()-6;
	      unsigned int MinSize = std::min(SizeCurrentState,SizeCurrentConfiguration);

	      if (m_JointErrorValuesForWalking.size()>=MinSize)
		{
		  for(unsigned int li=0;li<MinSize;li++)
		    m_JointErrorValuesForWalking(li)= (CurrentConfiguration(li+6)- CurrentState(li) )/m_TimeStep;
		}
	      else
		{
		  std::cout <<"The state of the robot and the one return by the WPG are different" << std::endl;
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
	      FromAbsoluteFootPosToDotHomogeneous(lLeftFootPosition,
						  m_LeftFootPosition,
						  m_dotLeftFootPosition);
	      FromAbsoluteFootPosToDotHomogeneous(lRightFootPosition,
						  m_RightFootPosition,
						  m_dotRightFootPosition);

	      /* We assume that the left foot is always the origin of the new frame. */
	      m_LeftFootPosition = m_MotionSinceInstanciationToThisSequence * m_LeftFootPosition;
	      m_RightFootPosition = m_MotionSinceInstanciationToThisSequence * m_RightFootPosition;

	      ml::Vector newRefPos(4), oldRefPos(4);
	      oldRefPos(0) = m_COMRefPos(0); oldRefPos(1) = m_COMRefPos(1);
	      oldRefPos(2) = m_COMRefPos(2); oldRefPos(3) = 1.0;
	      newRefPos = m_MotionSinceInstanciationToThisSequence * oldRefPos;
	      m_COMRefPos(0) = newRefPos(0);
	      m_COMRefPos(1) = newRefPos(1);
	      m_COMRefPos(2) = newRefPos(2);

	      oldRefPos(0) = m_ZMPRefPos(0); oldRefPos(1) = m_ZMPRefPos(1);
	      oldRefPos(2) = m_ZMPRefPos(2); oldRefPos(3) = 1.0;
	      newRefPos = m_MotionSinceInstanciationToThisSequence * oldRefPos;
	      m_ZMPRefPos(0) = newRefPos(0);
	      m_ZMPRefPos(1) = newRefPos(1);
	      m_ZMPRefPos(2) = newRefPos(2);

	      sotDEBUG(25) << "lLeftFootPosition.stepType: " << lLeftFootPosition.stepType
			   << " lRightFootPosition.stepType: " << lRightFootPosition.stepType <<endl;
	      // Find the support foot feet.
	      m_leftFootContact = true;
	      m_rightFootContact = true;
	      if (lLeftFootPosition.stepType==-1)
		{
		  lSupportFoot=1; m_leftFootContact = true;
		  if (lRightFootPosition.stepType!=-1)
		    m_rightFootContact = false;
		  m_DoubleSupportPhaseState = 0;
		}
	      else if (lRightFootPosition.stepType==-1)
		{
		  lSupportFoot=0; m_rightFootContact = true;
		  if (lLeftFootPosition.stepType!=-1)
		    m_leftFootContact = false;
		  m_DoubleSupportPhaseState = 0;
		}
	      else /* m_LeftFootPosition.z ==m_RightFootPosition.z
		      We keep the previous support foot half the time of the double support phase..
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
    initCommands( void )
    {
      using namespace command;
      addCommand("setVrmlDir",
		 makeCommandVoid1(*this,&PatternGenerator::setVrmlDirectory,
				  docCommandVoid1("Set VRML directory.",
						  "string (path name)")));
      addCommand("setVrml",
		 makeCommandVoid1(*this,&PatternGenerator::setVrmlMainFile,
				  docCommandVoid1("Set VRML main file.",
						  "string (file name)")));
      addCommand("setUrdfDir",
		 makeCommandVoid1(*this,&PatternGenerator::setUrdfDirectory,
				  docCommandVoid1("Set Urdf directory.",
						  "string (path name)")));
      addCommand("setUrdf",
		 makeCommandVoid1(*this,&PatternGenerator::setUrdfMainFile,
				  docCommandVoid1("Set Urdf main file.",
						  "string (file name)")));

      std::string docstring = "    \n"
        "    Set foot parameters\n"
        "      Input:\n"
        "        - a floating point number: the sole length,\n"
        "        - a floating point number: the sole width,\n"
        "    \n";
      addCommand("setSoleParameters",
		 makeCommandVoid2(*this,&PatternGenerator::setSoleParameters, 
                        docstring));


      addCommand("addJointMapping",
		 makeCommandVoid2(*this,&PatternGenerator::addJointMapping,
				  docCommandVoid1("Map link names.",
						  "string (link name)"
						  "string (rep name)")));
      addCommand("setXmlSpec",
		 makeCommandVoid1(*this,&PatternGenerator::setXmlSpecificitiesFile,
				  docCommandVoid1("Set Xml file for specicifities.",
						  "string (path/filename)")));
      addCommand("setXmlRank",
		 makeCommandVoid1(*this,&PatternGenerator::setXmlRankFile,
				  docCommandVoid1("Set XML rank file.",
						  "string (path/filename)")));
      addCommand("setParamPreview",
		 makeCommandVoid1(*this,&PatternGenerator::setParamPreviewFile,
				  docCommandVoid1("Set [guess what!] file",
						  "string (path/filename)")));
      // for the setFiles, need to implement the makeCmdVoid5... later
      // displayfiles... later too
      addCommand("buildModel",
       		 makeCommandVoid0(*this,
				  (void (PatternGenerator::*) (void))&PatternGenerator::buildModel,
				  docCommandVoid0("From the files, parse and build.")));
     addCommand("buildModelUrdf",
       		 makeCommandVoid0(*this,
				  (void (PatternGenerator::*) (void))&PatternGenerator::buildModelUrdf,
				  docCommandVoid0("From the files, parse and build.")));
      addCommand("initState",
       		 makeCommandVoid0(*this,
				  (void (PatternGenerator::*) (void))&PatternGenerator::InitState,
				  docCommandVoid0("From q and model, compute the initial geometry.")));
      addCommand("frameReference",
       		 makeCommandVoid1(*this,
				  &PatternGenerator::setReferenceFromString,
				  docCommandVoid1("Set the reference.",
						  "string among "
						  "World|Egocentered|LeftFootcentered|Waistcentered")));

      addCommand("getTimeStep",
		 makeDirectGetter(*this,&m_TimeStep,docDirectGetter("timestep","double")));
      addCommand("setTimeStep",
		 makeDirectSetter(*this,&m_TimeStep,docDirectSetter("timestep","double")));

      addCommand("getInitByRealState",
		 makeDirectGetter(*this,&m_InitPositionByRealState,
				  docDirectGetter("initByRealState","bool")));
      addCommand("setInitByRealState",
		 makeDirectSetter(*this,&m_InitPositionByRealState,
				  docDirectSetter("initByRealState","bool")));

      addCommand("addOnLineStep",
       		 makeCommandVoid3(*this,&PatternGenerator::addOnLineStep,
				  docCommandVoid3("Add a step on line.",
						  "double (x)","double (y)","double (theta)")));
      addCommand("addStep",
       		 makeCommandVoid3(*this,&PatternGenerator::addOnLineStep,
				  docCommandVoid3("Add a step in the stack.",
						  "double (x)","double (y)","double (theta)")));
      addCommand("parseCmd",
       		 makeCommandVoid1(*this,&PatternGenerator::pgCommandLine,
				  docCommandVoid1("Send the command line to the internal pg object.",
						  "string (command line)")));
      // Change next step : todo (deal with FootAbsolutePosition...).

     addCommand("debug",
       		 makeCommandVoid0(*this,
				  (void (PatternGenerator::*) (void))&PatternGenerator::debug,
				  docCommandVoid0("Launch a debug command.")));

    }

    void PatternGenerator::debug(void)
    {
      std::cout << "t = " << dataInProcessSOUT.getTime() << std::endl;
      std::cout << "deptype = " << dataInProcessSOUT.dependencyType << std::endl;
      std::cout << "child = " << dataInProcessSOUT.updateFromAllChildren << std::endl;
      std::cout << "last = " << dataInProcessSOUT.lastAskForUpdate << std::endl;

      std::cout << "inprocess = " << dataInProcessSOUT.needUpdate(40) << std::endl;
      std::cout << "onestep = " << OneStepOfControlS.needUpdate(40) << std::endl;

      dataInProcessSOUT.Signal<unsigned int,int>::access(1);
    }


    void PatternGenerator::addOnLineStep( const double & x, const double & y, const double & th)
    {
      assert( m_PGI!=NULL );
      m_PGI->AddOnLineStep(x,y,th);
    }
    void PatternGenerator::addStep( const double & x, const double & y, const double & th)
    {
      assert( m_PGI!=NULL );
      m_PGI->AddStepInStack(x,y,th);
    }
    void PatternGenerator::pgCommandLine( const std::string & cmdline )
    {
      assert( m_PGI!=NULL );
      std::istringstream cmdArgs( cmdline );
      m_PGI->ParseCmd(cmdArgs);
    }


    int PatternGenerator::
    stringToReferenceEnum( const std::string & FrameReference )
    {
      if (FrameReference=="World") return WORLD_FRAME;
      else if (FrameReference=="Egocentered") return EGOCENTERED_FRAME;
      else if (FrameReference=="LeftFootcentered")return LEFT_FOOT_CENTERED_FRAME;
      else if (FrameReference=="Waistcentered")return WAIST_CENTERED_FRAME;
      assert( false && "String name should be in the list "
	      "World|Egocentered|LeftFootcentered|Waistcentered" );
      return 0;
    }

    void PatternGenerator::
    setReferenceFromString( const std::string & str )
    {
      m_ReferenceFrame = stringToReferenceEnum( str );
    }

    ml::Vector & PatternGenerator::getjointWalkingErrorPosition(ml::Vector &res,int time)
    {
      sotDEBUGIN(5);

      OneStepOfControlS(time);

      res=m_JointErrorValuesForWalking;

      sotDEBUGOUT(5);

      return res;
    }

    unsigned int & PatternGenerator::
    getSupportFoot(unsigned int &res, int /*time*/)
    {
      res = m_SupportFoot;
      return res;
    }

    VectorRollPitchYaw & PatternGenerator::
    getWaistAttitude( VectorRollPitchYaw&res, int time)
    {
      sotDEBUGIN(5);
      OneStepOfControlS(time);
      for( unsigned int i=0;i<3;++i ) { res(i) = m_WaistAttitude(i); }
      sotDEBUG(5) << "WaistAttitude: " << m_WaistAttitude << endl;
      sotDEBUGOUT(5);
      return res;
    }

    VectorRollPitchYaw & PatternGenerator::
    getdComAttitude( VectorRollPitchYaw&res, int time)
    {
      sotDEBUGIN(5);
      OneStepOfControlS(time);
      for( unsigned int i=0;i<3;++i ) { res(i) = m_dComAttitude(i); }
      sotDEBUG(5) << "ComAttitude: " << m_dComAttitude << endl;
      sotDEBUGOUT(5);
      return res;
    }


    VectorRollPitchYaw & PatternGenerator::
    getComAttitude( VectorRollPitchYaw&res, int time)
    {
      sotDEBUGIN(5);
      OneStepOfControlS(time);
      for( unsigned int i=0;i<3;++i ) { res(i) = m_ComAttitude(i); }
      sotDEBUG(5) << "ComAttitude: " << m_ComAttitude << endl;
      sotDEBUGOUT(5);
      return res;
    }

    VectorRollPitchYaw & PatternGenerator::getWaistAttitudeAbsolute(VectorRollPitchYaw &res, int time)
    {
      sotDEBUGIN(5);
      OneStepOfControlS(time);
      sotDEBUG(15) << "I survived one step of control" << std::endl;
      for( unsigned int i=0;i<3;++i ) { res(i) = m_WaistAttitudeAbsolute(i); }
      sotDEBUG(5) << "WaistAttitude: " << m_WaistAttitudeAbsolute << endl;
      sotDEBUGOUT(5);
      return res;
    }

    ml::Vector & PatternGenerator::
    getWaistPosition(ml::Vector &res, int time)
    {
      sotDEBUGIN(5);
      OneStepOfControlS(time);
      res = m_WaistPosition;
      sotDEBUG(5) << "WaistPosition: " << m_WaistPosition << endl;
      sotDEBUGOUT(5);
      return res;
    }
    ml::Vector & PatternGenerator::
    getWaistPositionAbsolute(ml::Vector &res, int time)
    {
      sotDEBUGIN(5);
      OneStepOfControlS(time);
      res = m_WaistPositionAbsolute;
      /* ARGH ! ->  res(2) =0*/
      sotDEBUG(5) << "WaistPosition: " << m_WaistPositionAbsolute << endl;
      sotDEBUGOUT(5);
      return res;
    }

    unsigned & PatternGenerator::
    getDataInProcess(unsigned &res, int time)
    {
      sotDEBUGIN(5);
      OneStepOfControlS(time);
      res = m_dataInProcess;
      sotDEBUG(5) << "DataInProcess: " << m_dataInProcess << endl;
      sotDEBUGOUT(5);
      return res;
    }

    void PatternGenerator::
    setSoleParameters(const double& inSoleLength, const double& inSoleWidth)
    {
      m_soleLength = inSoleLength;
      m_soleWidth  = inSoleWidth;
    }
  } // namespace dg
} // namespace sot
