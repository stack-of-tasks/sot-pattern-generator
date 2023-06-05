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

/* Local Variables:  */
/* mode: c++         */
/* comment-column: 0            */
/* visual-fill-column-width: 80 */
/* indent-tabs-mode: nil */
/* End:                         */

// #define VP_DEBUG
// #define VP_DEBUG_MODE 45
#include <pinocchio/fwd.hpp>
// pin first
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <sot/core/debug.hh>
#include <sot/core/robot-utils.hh>
#include <sstream>
#include <stdexcept>

#ifdef VP_DEBUG
class sotPG__INIT {
 public:
  sotPG__INIT(void) { dynamicgraph::sot::DebugTrace::openFile(); }
};
sotPG__INIT sotPG_initiator;
#endif  // #ifdef VP_DEBUG

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>
#include <sot/pattern-generator/pg.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <sot/core/matrix-geometry.hh>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"

using namespace std;
namespace dynamicgraph {
namespace sot {

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PatternGenerator, "PatternGenerator");

PatternGenerator::PatternGenerator(const std::string &name)
    : Entity(name),
      m_PGI(0),
      m_PreviewControlParametersFile(),
      m_urdfFile(""),
      m_srdfFile(""),
      m_xmlRankFile(""),
      m_soleWidth(0),
      m_init(false),
      m_InitPositionByRealState(true),
      firstSINTERN(
          boost::bind(&PatternGenerator::InitOneStepOfControl, this, _1, _2),
          sotNOSIGNAL, "PatternGenerator(" + name + ")::intern(dummy)::init")

      ,
      OneStepOfControlS(
          boost::bind(&PatternGenerator::OneStepOfControl, this, _1, _2),
          firstSINTERN << jointPositionSIN,
          "PatternGenerator(" + name + ")::onestepofcontrol")

      ,
      m_dataInProcess(0),
      m_rightFootContact(true)  // It is assumed that the robot is standing.
      ,
      m_leftFootContact(true),
      jointPositionSIN(
          NULL, "PatternGenerator(" + name + ")::input(vector)::position")

      ,
      motorControlJointPositionSIN(
          NULL, "PatternGenerator(" + name + ")::input(vector)::motorcontrol")

      ,
      ZMPPreviousControllerSIN(NULL,
                               "PatternGenerator(" + name +
                                   ")::input(vector)::zmppreviouscontroller")

      ,
      ZMPRefSOUT(boost::bind(&PatternGenerator::getZMPRef, this, _1, _2),
                 OneStepOfControlS,
                 "PatternGenerator(" + name + ")::output(vector)::zmpref")

      ,
      CoMRefSOUT(boost::bind(&PatternGenerator::getCoMRef, this, _1, _2),
                 OneStepOfControlS,
                 "PatternGenerator(" + name + ")::output(vector)::comref")

      ,
      dCoMRefSOUT(boost::bind(&PatternGenerator::getdCoMRef, this, _1, _2),
                  OneStepOfControlS,
                  "PatternGenerator(" + name + ")::output(vector)::dcomref")

      ,
      ddCoMRefSOUT(boost::bind(&PatternGenerator::getddCoMRef, this, _1, _2),
                   OneStepOfControlS,
                   "PatternGenerator(" + name + ")::output(vector)::ddcomref")

      ,
      comSIN(NULL, "PatternGenerator(" + name + ")::input(vector)::com")

      ,
      comStateSIN(NULL,
                  "PatternGenerator(" + name + ")::input(vector)::comStateSIN")

      ,
      zmpSIN(NULL, "PatternGenerator(" + name + ")::input(vector)::zmpSIN")

      ,
      forceSIN(NULL, "PatternGenerator(" + name + ")::input(vector)::forceSIN")

      ,
      forceSOUT(boost::bind(&PatternGenerator::getExternalForces, this, _1, _2),
                OneStepOfControlS,
                "PatternGenerator(" + name + ")::output(vector)::forceSOUT")

      ,
      velocitydesSIN(
          NULL, "PatternGenerator(" + name + ")::input(vector)::velocitydes")

      ,
      triggerSIN(NULL, "PatternGenerator(" + name + ")::input(bool)::trigger")

      ,
      LeftFootCurrentPosSIN(
          NULL, "PatternGenerator(" + name +
                    ")::input(homogeneousmatrix)::leftfootcurrentpos")

      ,
      RightFootCurrentPosSIN(
          NULL, "PatternGenerator(" + name +
                    ")::input(homogeneousmatrix)::rightfootcurrentpos")

      ,
      LeftFootRefSOUT(
          boost::bind(&PatternGenerator::getLeftFootRef, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name +
              ")::output(homogeneousmatrix)::leftfootref")

      ,
      RightFootRefSOUT(
          boost::bind(&PatternGenerator::getRightFootRef, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name +
              ")::output(homogeneousmatrix)::rightfootref")

      ,
      dotLeftFootRefSOUT(
          boost::bind(&PatternGenerator::getdotLeftFootRef, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name +
              ")::output(homogeneousmatrix)::dotleftfootref")

      ,
      dotRightFootRefSOUT(
          boost::bind(&PatternGenerator::getdotRightFootRef, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name +
              ")::output(homogeneousmatrix)::dotrightfootref")

      ,
      FlyingFootRefSOUT(
          boost::bind(&PatternGenerator::getFlyingFootRef, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name +
              ")::output(homogeneousmatrix)::flyingfootref")

      ,
      SupportFootSOUT(
          boost::bind(&PatternGenerator::getSupportFoot, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name + ")::output(uint)::SupportFoot"),
      jointWalkingErrorPositionSOUT(
          boost::bind(&PatternGenerator::getjointWalkingErrorPosition, this, _1,
                      _2),
          OneStepOfControlS,
          "PatternGenerator(" + name +
              ")::output(vector)::walkingerrorposition")

      ,
      comattitudeSOUT(
          boost::bind(&PatternGenerator::getComAttitude, this, _1, _2),
          OneStepOfControlS,
          "sotPatternGenerator(" + name + ")::output(vectorRPY)::comattitude")

      ,
      dcomattitudeSOUT(
          boost::bind(&PatternGenerator::getdComAttitude, this, _1, _2),
          OneStepOfControlS,
          "sotPatternGenerator(" + name + ")::output(vectorRPY)::dcomattitude")

      ,
      ddcomattitudeSOUT(
          boost::bind(&PatternGenerator::getddComAttitude, this, _1, _2),
          OneStepOfControlS,
          "sotPatternGenerator(" + name + ")::output(vectorRPY)::ddcomattitude")

      ,
      waistattitudeSOUT(
          boost::bind(&PatternGenerator::getWaistAttitude, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name + ")::output(vectorRPY)::waistattitude")

      ,
      waistattitudeabsoluteSOUT(
          boost::bind(&PatternGenerator::getWaistAttitudeAbsolute, this, _1,
                      _2),
          OneStepOfControlS,
          "PatternGenerator(" + name +
              ")::output(vectorRPY)::waistattitudeabsolute")

      ,
      waistattitudematrixabsoluteSOUT(
          boost::bind(&PatternGenerator::getWaistAttitudeMatrixAbsolute, this,
                      _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name +
              ")::output(homogeneousmatrix)::waistattitudematrixabsolute")

      ,
      waistpositionSOUT(
          boost::bind(&PatternGenerator::getWaistPosition, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name + ")::output(vector)::waistposition")

      ,
      waistpositionabsoluteSOUT(
          boost::bind(&PatternGenerator::getWaistPositionAbsolute, this, _1,
                      _2),
          OneStepOfControlS,
          "PatternGenerator(" + name +
              ")::output(vector)::waistpositionabsolute")

      ,
      dataInProcessSOUT(
          boost::bind(&PatternGenerator::getDataInProcess, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name + ")::output(bool)::inprocess")

      ,
      InitZMPRefSOUT(
          boost::bind(&PatternGenerator::getInitZMPRef, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name + ")::output(vector)::initzmpref")

      ,
      InitCoMRefSOUT(
          boost::bind(&PatternGenerator::getInitCoMRef, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name + ")::output(matrix)::initcomref")

      ,
      InitWaistPosRefSOUT(
          boost::bind(&PatternGenerator::getInitWaistPosRef, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name + ")::output(vector)::initwaistposref")

      ,
      InitWaistAttRefSOUT(
          boost::bind(&PatternGenerator::getInitWaistAttRef, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name + ")::output(vectorRPY)::initwaistattref")

      ,
      InitLeftFootRefSOUT(
          boost::bind(&PatternGenerator::getInitLeftFootRef, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name +
              ")::output(homogeneousmatrix)::initleftfootref")

      ,
      InitRightFootRefSOUT(
          boost::bind(&PatternGenerator::getInitRightFootRef, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name +
              ")::output(homogeneousmatrix)::initrightfootref")

      ,
      leftFootContactSOUT(
          boost::bind(&PatternGenerator::getLeftFootContact, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name + ")::output(bool)::leftfootcontact")

      ,
      rightFootContactSOUT(
          boost::bind(&PatternGenerator::getRightFootContact, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name + ")::output(bool)::rightfootcontact"),
      contactPhaseSOUT(
          boost::bind(&PatternGenerator::getContactPhase, this, _1, _2),
          OneStepOfControlS,
          "PatternGenerator(" + name + ")::output(int)::contactphase")

{
  m_MotionSinceInstanciationToThisSequence.setIdentity();

  m_LocalTime = 0;
  m_count = 0;
  m_TimeStep = 0.005;
  m_ContactPhase = DOUBLE_SUPPORT_PHASE;
  m_forceFeedBack = false;
  m_feedBackControl = false;

  m_ZMPRefPos.resize(4);
  m_ZMPRefPos.fill(0.0);
  m_ZMPRefPos(3) = 1.0;
  m_COMRefPos.resize(3);
  m_COMRefPos.fill(0.0);
  m_PrevSamplingCOMRefPos.resize(3);
  m_PrevSamplingCOMRefPos.fill(0.0);
  m_NextSamplingCOMRefPos.resize(3);
  m_NextSamplingCOMRefPos.fill(0.0);
  m_ZMPPrevious.resize(4);
  m_ZMPPrevious(3) = 1.0;
  m_dCOMRefPos.resize(3);
  m_dCOMRefPos.fill(0.0);
  m_PrevSamplingdCOMRefPos.resize(3);
  m_PrevSamplingdCOMRefPos.fill(0.0);
  m_NextSamplingdCOMRefPos.resize(3);
  m_NextSamplingdCOMRefPos.fill(0.0);
  m_ddCOMRefPos.resize(3);
  m_ddCOMRefPos.fill(0.0);
  m_PrevSamplingddCOMRefPos.resize(3);
  m_PrevSamplingddCOMRefPos.fill(0.0);
  m_NextSamplingddCOMRefPos.resize(3);
  m_NextSamplingddCOMRefPos.fill(0.0);
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
  m_ddComAttitude.resize(3);
  m_ddComAttitude.fill(0);
  m_VelocityReference.resize(3);
  m_VelocityReference.fill(0.0);
  m_trigger = false;
  m_WaistAttitude.resize(3);
  m_WaistAttitude.fill(0);
  m_ComAttitude.resize(3);
  m_ComAttitude.fill(0);
  m_WaistPosition.resize(3);
  m_WaistPosition.fill(0);
  m_WaistAttitudeAbsolute.resize(3);
  m_WaistAttitudeAbsolute.fill(0);
  m_PrevSamplingWaistAttAbs.resize(3);
  m_PrevSamplingWaistAttAbs.fill(0);
  m_NextSamplingWaistAttAbs.resize(3);
  m_NextSamplingWaistAttAbs.fill(0);
  m_WaistPositionAbsolute.resize(3);
  m_WaistPositionAbsolute.fill(0);

  m_WaistAttitudeMatrixAbsolute.setIdentity();
  m_LeftFootPosition.setIdentity();
  m_RightFootPosition.setIdentity();

  m_dotLeftFootPosition.setIdentity();
  m_dotRightFootPosition.setIdentity();

  m_InitLeftFootPosition.setIdentity();
  m_InitRightFootPosition.setIdentity();
  m_FlyingFootPosition.setIdentity();

  m_k_Waist_kp1.setIdentity();

  m_SupportFoot = 1;  // Means that we do not know which support foot it is.
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

  // OneStepOfControlS.setDependencyType(TimeDependency<int>::ALWAYS_READY);
  //  OneStepOfControlS.setConstant(0);

  OneStepOfControlS.addDependency(LeftFootCurrentPosSIN);
  OneStepOfControlS.addDependency(RightFootCurrentPosSIN);
  OneStepOfControlS.addDependency(velocitydesSIN);
  OneStepOfControlS.addDependency(triggerSIN);
  OneStepOfControlS.addDependency(firstSINTERN);
  OneStepOfControlS.addDependency(motorControlJointPositionSIN);
  OneStepOfControlS.addDependency(comSIN);
  OneStepOfControlS.addDependency(comStateSIN);
  OneStepOfControlS.addDependency(zmpSIN);
  OneStepOfControlS.addDependency(forceSIN);

  // For debug, register OSOC (not relevant for normal use).
  signalRegistration(OneStepOfControlS);

  signalRegistration(dataInProcessSOUT);

  signalRegistration(jointPositionSIN << motorControlJointPositionSIN
                                      << ZMPPreviousControllerSIN << ZMPRefSOUT
                                      << CoMRefSOUT << dCoMRefSOUT
                                      << ddCoMRefSOUT);

  signalRegistration(comStateSIN << zmpSIN << forceSIN << forceSOUT);

  signalRegistration(comSIN << velocitydesSIN << triggerSIN
                            << LeftFootCurrentPosSIN << RightFootCurrentPosSIN
                            << LeftFootRefSOUT << RightFootRefSOUT);

  signalRegistration(SupportFootSOUT << jointWalkingErrorPositionSOUT
                                     << comattitudeSOUT << dcomattitudeSOUT
                                     << ddcomattitudeSOUT << waistattitudeSOUT
                                     << waistattitudematrixabsoluteSOUT);

  signalRegistration(waistpositionSOUT << waistattitudeabsoluteSOUT
                                       << waistpositionabsoluteSOUT);

  signalRegistration(dotLeftFootRefSOUT << dotRightFootRefSOUT);

  signalRegistration(InitZMPRefSOUT << InitCoMRefSOUT << InitWaistPosRefSOUT
                                    << InitWaistAttRefSOUT
                                    << InitLeftFootRefSOUT
                                    << InitRightFootRefSOUT);

  signalRegistration(leftFootContactSOUT << rightFootContactSOUT
                                         << contactPhaseSOUT);

  initCommands();

  // init filter for force signals "to be removed"
  m_bufferForce.clear();
  std::vector<double>::size_type n = 10;
  double sum = 0, tmp = 0;
  m_filterWindow.resize((std::vector<double>::size_type)(n + 1));
  for (std::vector<double>::size_type i = 0; i < n + 1; i++) {
    tmp = sin((M_PI * i) / n);
    m_filterWindow[i] = tmp * tmp;
  }

  for (std::vector<double>::size_type i = 0; i < n + 1; i++)
    sum += m_filterWindow[i];

  for (std::vector<double>::size_type i = 0; i < n + 1; i++)
    m_filterWindow[i] /= sum;

  m_initForce.resize(6);
  m_currentForces.resize(6);
  // dataInProcessSOUT.setReference( &m_dataInProcess );
  // m_wrml2urdfIndex.clear();

  sotDEBUGOUT(5);
}

bool PatternGenerator::InitState(void) {
  sotDEBUGIN(5);
  // TODO
  // Instead of (0) ie .access(0), it could be rather used:
  // .accessCopy()
  // Instead of copy value (ml::Vector pos) it could be rather
  // used reference (const ml::Vector & post)
  Vector res;
  Eigen::Matrix<double, 6, 1> lWaistPosition;
  if (m_InitPositionByRealState) {
    const Vector &pos = jointPositionSIN(m_LocalTime);

    lWaistPosition.resize(6);
    for (unsigned int i = 0; i < 6; ++i) {
      lWaistPosition(i) = pos(i);
    }
    //  m_ZMPPrevious[2] =m_AnkleSoilDistance;
    // Changed the reference frame.

    res.resize(pos.size() - 6);

    for (unsigned i = 0; i < res.size(); i++) res(i) = pos(i + 6);

    Vector lZMPPrevious = ZMPPreviousControllerSIN(m_LocalTime);
    for (unsigned int i = 0; i < 3; i++) m_ZMPPrevious[i] = lZMPPrevious(i);
  } else {
    res = motorControlJointPositionSIN(m_LocalTime);
    // for(unsigned i=0;i<res.size();i++)
    // res(m_wrml2urdfIndex[i]) = res(i);
  }

  Vector com = comSIN(m_LocalTime);

  m_JointErrorValuesForWalking.resize(res.size());

  sotDEBUG(5) << "m_LocalTime:" << m_LocalTime << endl;
  sotDEBUG(5) << "Joint Values:" << res << endl;

  try {
    Eigen::VectorXd bres;
    bres.resize(res.size());
    for (int i = 0; i < res.size(); i++) bres(i) = res(i);
    m_PGI->SetCurrentJointValues(bres);

    // Evaluate current position of the COM, ZMP and feet
    // according to the state of the robot.
    PatternGeneratorJRL::COMState lStartingCOMState;
    Eigen::Vector3d lStartingZMPPosition;

    m_PGI->EvaluateStartingState(lStartingCOMState, lStartingZMPPosition,
                                 lWaistPosition, m_InitLeftFootAbsPos,
                                 m_InitRightFootAbsPos);

    // Put inside sotHomogeneous representation
    m_InitCOMRefPos(0) = lStartingCOMState.x[0];
    m_InitCOMRefPos(1) = lStartingCOMState.y[0];
    m_InitCOMRefPos(2) = lStartingCOMState.z[0];

    m_InitZMPRefPos(0) = lStartingCOMState.x[0];
    m_InitZMPRefPos(1) = lStartingCOMState.y[0];
    m_InitZMPRefPos(2) = 0;

    if (m_InitPositionByRealState) {
      m_ZMPPrevious[0] = lStartingCOMState.x[0];
      m_ZMPPrevious[1] = lStartingCOMState.y[0];
      m_ZMPPrevious[2] = 0;
    }
    sotDEBUG(5) << "InitZMPRefPos :" << m_InitZMPRefPos << endl;

    m_InitWaistRefPos(0) = m_WaistPositionAbsolute(0) = lWaistPosition(0);
    m_InitWaistRefPos(1) = m_WaistPositionAbsolute(1) = lWaistPosition(1);
    m_InitWaistRefPos(2) = m_WaistPositionAbsolute(2) = lWaistPosition(2);

    m_InitWaistRefAtt(0) = m_WaistAttitudeAbsolute(0) = lWaistPosition(3);
    m_InitWaistRefAtt(1) = m_WaistAttitudeAbsolute(1) = lWaistPosition(4);
    m_InitWaistRefAtt(2) = m_WaistAttitudeAbsolute(2) = lWaistPosition(5);

    FromAbsoluteFootPosToDotHomogeneous(
        m_InitRightFootAbsPos, m_InitRightFootPosition, m_dotRightFootPosition);
    FromAbsoluteFootPosToDotHomogeneous(
        m_InitLeftFootAbsPos, m_InitLeftFootPosition, m_dotLeftFootPosition);

    Eigen::Matrix<double, 4, 1> newtmp, oldtmp;
    oldtmp(0) = m_InitCOMRefPos(0);
    oldtmp(1) = m_InitCOMRefPos(1);
    oldtmp(2) = m_InitCOMRefPos(2);
    oldtmp(3) = 1.0;
    newtmp = m_MotionSinceInstanciationToThisSequence * oldtmp;
    m_InitCOMRefPos(0) = newtmp(0);
    m_InitCOMRefPos(1) = newtmp(1);
    m_InitCOMRefPos(2) = newtmp(2);

    oldtmp(0) = m_InitZMPRefPos(0);
    oldtmp(1) = m_InitZMPRefPos(1);
    oldtmp(2) = m_InitZMPRefPos(2);
    newtmp = m_MotionSinceInstanciationToThisSequence * oldtmp;
    m_InitZMPRefPos(0) = newtmp(0);
    m_InitZMPRefPos(1) = newtmp(1);
    m_InitZMPRefPos(2) = newtmp(2);

    if (!m_InitPositionByRealState) {
      MatrixHomogeneous invInitLeftFootRef;
      invInitLeftFootRef = m_InitLeftFootPosition.inverse();
      m_k_Waist_kp1 = m_k_Waist_kp1 * invInitLeftFootRef;
      m_MotionSinceInstanciationToThisSequence =
          m_MotionSinceInstanciationToThisSequence * m_k_Waist_kp1;
    }

    m_k_Waist_kp1 = m_InitLeftFootPosition;

    m_InitLeftFootPosition =
        m_MotionSinceInstanciationToThisSequence * m_InitLeftFootPosition;
    m_InitRightFootPosition =
        m_MotionSinceInstanciationToThisSequence * m_InitRightFootPosition;

    m_LeftFootPosition = m_InitLeftFootPosition;
    m_RightFootPosition = m_InitRightFootPosition;

  } catch (...) {
    SOT_THROW ExceptionPatternGenerator(
        ExceptionPatternGenerator::PATTERN_GENERATOR_JRL,
        "Error while setting the current joint values of the WPG.");
    return false;
  }

  m_InitPositionByRealState = false;
  sotDEBUGOUT(5);
  return true;
}

bool PatternGenerator::buildReducedModel(void) {
  // Name of the parameter
  const std::string lparameter_name("/robot_description");

  // Model of the robot inside a string.
  std::string lrobot_description;

  // Reading the parameter.
  std::string model_name("robot");

  // Search for the robot util related to robot_name.
  sot::RobotUtilShrPtr aRobotUtil = sot::getRobotUtil(model_name);

  // If does not exist then it is created.
  if (aRobotUtil.get() == sot::RefVoidRobotUtil().get()) {
    ostringstream oss;
    oss << __FILE__ << " PatternGenerator::buildModel "
        << "The robot with name " << model_name << " was not found !";
    throw std::invalid_argument(oss.str());
    return false;
  }

  try {
    // Then build a complete robot model.
    lrobot_description = aRobotUtil->get_parameter<string>(lparameter_name);
  } catch (...) {
    SOT_THROW ExceptionPatternGenerator(
        ExceptionPatternGenerator::PATTERN_GENERATOR_JRL,
        "Error while getting parameter " + lparameter_name + " for the WPG.");
    return false;
  }

  pinocchio::Model lrobotModel;
  pinocchio::urdf::buildModelFromXML(
      lrobot_description, pinocchio::JointModelFreeFlyer(), lrobotModel);

  // Then extract a reduced model
  Eigen::VectorXd q_neutral = neutral(lrobotModel);
  ExtractJointMimics an_extract_joint_mimics(lrobot_description);

  const std::vector<std::string> &list_of_joints_to_lock_by_name =
      an_extract_joint_mimics.get_mimic_joints();

  std::ostringstream oss;
  oss << "Size of mimic joints: " << lrobotModel.nq << " "
      << list_of_joints_to_lock_by_name.size() << " " << q_neutral.size();
  sendMsg(oss.str(), MSG_TYPE_INFO);

  std::vector<pinocchio::JointIndex> list_of_joints_to_lock_by_id;
  for (auto it : list_of_joints_to_lock_by_name) {
    const std::string &joint_name = it;

    if (lrobotModel.existJointName(joint_name)) {
      // do not consider joint that are not in the model
      list_of_joints_to_lock_by_id.push_back(
          lrobotModel.getJointId(joint_name));
    } else
      std::cout << "joint_name not found: " << joint_name << std::endl;
  }

  if (list_of_joints_to_lock_by_id.size() == 0)
    m_robotModel = pinocchio::buildReducedModel(
        lrobotModel, list_of_joints_to_lock_by_id, q_neutral);
  else
    m_robotModel = lrobotModel;

  m_robotData = new pinocchio::Data(m_robotModel);

  return true;
}

bool PatternGenerator::addComplementaryFrames() {
  // Reading the parameter.
  std::string model_name("robot");

  // Search for the robot util related to robot_name.
  sot::RobotUtilShrPtr aRobotUtil = sot::getRobotUtil(model_name);

  std::vector<std::string> lparameter_names = {
      "/pg/remap/l_ankle", "/pg/remap/r_ankle", "/pg/remap/l_wrist",
      "/pg/remap/r_wrist", "/pg/remap/body",    "/pg/remap/torso"};

  std::vector<std::string> lframe_remapped = {"l_ankle", "r_ankle", "l_wrist",
                                              "r_wrist", "BODY",    "torso"};

  auto it_frame_remap = lframe_remapped.begin();
  for (auto it_param_name : lparameter_names) {
    std::string lbody_name;
    lbody_name = aRobotUtil->get_parameter<string>(it_param_name);
    if (m_robotModel.existFrame(lbody_name)) {
      pinocchio::Model::Index idx = m_robotModel.getFrameId(lbody_name);
      m_robotModel.frames[idx].name = *it_frame_remap;
    } else {
      SOT_THROW ExceptionPatternGenerator(
          ExceptionPatternGenerator::PATTERN_GENERATOR_JRL,
          "Error for parameter " + it_param_name + " body name " + lbody_name +
              " doest no exist");
      return false;
    }
    it_frame_remap++;
  }

  return true;
}

void PatternGenerator::readFootParameters(std::string &rootFootPath,
                                          pg::PRFoot &aFoot) {
  // Reading the parameter.
  std::string model_name("robot");

  // Search for the robot util related to robot_name.
  sot::RobotUtilShrPtr aRobotUtil = sot::getRobotUtil(model_name);

  std::string pathname = rootFootPath + "/size/height";
  aFoot.soleHeight = aRobotUtil->get_parameter<double>(pathname);
  pathname = rootFootPath + "/size/width";
  aFoot.soleWidth = aRobotUtil->get_parameter<double>(pathname);
  pathname = rootFootPath + "/size/depth";
  aFoot.soleDepth = aRobotUtil->get_parameter<double>(pathname);

  pathname = rootFootPath + "/anklePosition/x";
  aFoot.anklePosition(0) = aRobotUtil->get_parameter<double>(pathname);
  pathname = rootFootPath + "/anklePosition/y";
  aFoot.anklePosition(1) = aRobotUtil->get_parameter<double>(pathname);
  pathname = rootFootPath + "/anklePosition/z";
  aFoot.anklePosition(2) = aRobotUtil->get_parameter<double>(pathname);
}

bool PatternGenerator::buildPGI(void) {
  bool ok = true;

  // Build the reduced model of the robot
  buildReducedModel();
  addComplementaryFrames();

  // Creating the humanoid robot.
  m_PR = new pg::PinocchioRobot();
  m_PR->initializeRobotModelAndData(&m_robotModel, m_robotData);

  // Read xml/srdf stream
  using boost::property_tree::ptree;
  ptree pt;
  try {
    // Initialize the Right Foot
    pg::PRFoot aFoot;

    // First find the joint to which the r_ankle body is related
    pinocchio::FrameIndex ra = m_robotModel.getFrameId("r_ankle");
    aFoot.associatedAnkle = m_robotModel.frames.at(ra).parent;

    // Then populates the PRFoot structure with the property tree.
    std::string path = "/robot/specificities/feet/right";
    readFootParameters(path, aFoot);

    // Initialize internal state of the right foot.
    m_PR->initializeRightFoot(aFoot);

    // Initialize the Left Foot
    // First find the joint to which the l_ankle body is related
    pinocchio::FrameIndex la = m_robotModel.getFrameId("l_ankle");
    aFoot.associatedAnkle = m_robotModel.frames.at(la).parent;

    // Then populates the PRFoot structure with the property tree.
    path = "/robot/specificities/feet/left";
    readFootParameters(path, aFoot);

    // Initialize internal state of the left foot.
    m_PR->initializeLeftFoot(aFoot);

  } catch (...) {
    cerr << "problem while setting the feet informations. File corrupted?"
         << endl;
    ok = false;
  }

  if (m_PR != 0) {
    pg::PRFoot *rightFoot = m_PR->rightFoot();
    if (rightFoot != 0) {
      Eigen::Vector3d AnkleInFoot;
      AnkleInFoot = rightFoot->anklePosition;
      m_AnkleSoilDistance = fabs(AnkleInFoot(2));
    } else
      ok = false;
  } else
    ok = false;

  if (!ok) {
    SOT_THROW ExceptionPatternGenerator(
        ExceptionPatternGenerator::PATTERN_GENERATOR_JRL,
        "Error while creating humanoid robot dynamical model.",
        "(PG creation process for object %s).", getName().c_str());
  }
  try {
    m_PGI = PatternGeneratorJRL::patternGeneratorInterfaceFactory(m_PR);
  }

  catch (...) {
    SOT_THROW ExceptionPatternGenerator(
        ExceptionPatternGenerator::PATTERN_GENERATOR_JRL,
        "Error while allocating the Pattern Generator.",
        "(PG creation process for object %s).", getName().c_str());
  }
  m_init = true;
  return false;
}

PatternGenerator::~PatternGenerator(void) {
  sotDEBUGIN(25);
  if (0 != m_PR) {
    delete m_PR;
    m_PR = 0;
  }
  if (0 != m_PGI) {
    delete m_PGI;
    m_PGI = 0;
  }
  if (0 != m_robotData) {
    delete m_robotData;
    m_robotData = 0;
  }
  sotDEBUGOUT(25);
  return;
}

/* --- CONFIG ---------------------------------------------------------- */
/* --- CONFIG ---------------------------------------------------------- */
/* --- CONFIG ---------------------------------------------------------- */
/* --- CONFIG ---------------------------------------------------------- */
void PatternGenerator::setParamPreviewFile(const std::string &filename) {
  m_PreviewControlParametersFile = filename;
}

void PatternGenerator::setURDFFile(const std::string &filename) {
  m_urdfFile = filename;
}
void PatternGenerator::setSRDFFile(const std::string &filename) {
  m_srdfFile = filename;
}
void PatternGenerator::setXmlRankFile(const std::string &filename) {
  m_xmlRankFile = filename;
}
void PatternGenerator::addJointMapping(const std::string &link,
                                       const std::string &repName) {
  specialJoints_[link] = repName;
}

/* --- COMPUTE --------------------------------------------------------- */
/* --- COMPUTE --------------------------------------------------------- */
/* --- COMPUTE --------------------------------------------------------- */

Vector &PatternGenerator::getZMPRef(Vector &ZMPRefval, int time) {
  sotDEBUGIN(5);

  OneStepOfControlS(time);

  ZMPRefval.resize(3);
  ZMPRefval(0) = m_ZMPRefPos(0);
  ZMPRefval(1) = m_ZMPRefPos(1);
  ZMPRefval(2) = m_ZMPRefPos(2);
  sotDEBUG(5) << "ZMPRefPos transmitted" << m_ZMPRefPos << " " << ZMPRefval
              << endl;

  sotDEBUGOUT(5);
  return ZMPRefval;
}

Vector &PatternGenerator::getCoMRef(Vector &CoMRefval, int time) {
  sotDEBUGIN(25);

  OneStepOfControlS(time);
  CoMRefval = m_COMRefPos;

  sotDEBUGOUT(25);
  return CoMRefval;
}

Vector &PatternGenerator::getdCoMRef(Vector &CoMRefval, int time) {
  sotDEBUGIN(25);

  OneStepOfControlS(time);
  CoMRefval = m_dCOMRefPos;

  sotDEBUGOUT(25);
  return CoMRefval;
}

Vector &PatternGenerator::getddCoMRef(Vector &CoMRefval, int time) {
  sotDEBUGIN(25);

  OneStepOfControlS(time);
  CoMRefval = m_ddCOMRefPos;

  sotDEBUGOUT(25);
  return CoMRefval;
}

Vector &PatternGenerator::getExternalForces(Vector &forces, int time) {
  sotDEBUGIN(25);

  OneStepOfControlS(time);
  forces = m_currentForces;

  sotDEBUGOUT(25);
  return forces;
}

Vector &PatternGenerator::getInitZMPRef(Vector &InitZMPRefval, int /*time*/) {
  sotDEBUGIN(25);

  sotDEBUG(25) << "InitZMPRefPos transmitted" << m_InitZMPRefPos << " "
               << InitZMPRefval << std::endl;
  InitZMPRefval.resize(3);
  InitZMPRefval(0) = m_InitZMPRefPos(0);
  InitZMPRefval(1) = m_InitZMPRefPos(1);
  InitZMPRefval(2) = m_InitZMPRefPos(2);

  sotDEBUGOUT(25);
  return InitZMPRefval;
}

Vector &PatternGenerator::getInitCoMRef(Vector &InitCoMRefval, int /*time*/) {
  sotDEBUGIN(25);

  InitCoMRefval.resize(3);
  InitCoMRefval(0) = m_InitCOMRefPos(0);
  InitCoMRefval(1) = m_InitCOMRefPos(1);
  InitCoMRefval(2) = m_InitCOMRefPos(2);

  sotDEBUGOUT(25);
  return InitCoMRefval;
}

Vector &PatternGenerator::getInitWaistPosRef(Vector &InitWaistRefval,
                                             int /*time*/) {
  sotDEBUGIN(25);

  InitWaistRefval = m_InitWaistRefPos;

  sotDEBUGOUT(25);
  return InitWaistRefval;
}
VectorRollPitchYaw &PatternGenerator::getInitWaistAttRef(
    VectorRollPitchYaw &InitWaistRefval, int /*time*/) {
  sotDEBUGIN(25);

  for (unsigned int i = 0; i < 3; ++i)
    InitWaistRefval(i) = m_InitWaistRefAtt(i);

  sotDEBUGOUT(25);
  return InitWaistRefval;
}

MatrixHomogeneous &PatternGenerator::getLeftFootRef(
    MatrixHomogeneous &LeftFootRefVal, int time) {
  sotDEBUGIN(25);

  OneStepOfControlS(time);
  LeftFootRefVal = m_LeftFootPosition;
  sotDEBUGOUT(25);
  return LeftFootRefVal;
}

MatrixHomogeneous &PatternGenerator::getRightFootRef(
    MatrixHomogeneous &RightFootRefval, int time) {
  sotDEBUGIN(25);

  OneStepOfControlS(time);

  RightFootRefval = m_RightFootPosition;
  sotDEBUGOUT(25);
  return RightFootRefval;
}
MatrixHomogeneous &PatternGenerator::getdotLeftFootRef(
    MatrixHomogeneous &LeftFootRefVal, int time) {
  sotDEBUGIN(25);

  OneStepOfControlS(time);
  LeftFootRefVal = m_dotLeftFootPosition;
  sotDEBUGOUT(25);
  return LeftFootRefVal;
}
MatrixHomogeneous &PatternGenerator::getdotRightFootRef(
    MatrixHomogeneous &RightFootRefval, int time) {
  sotDEBUGIN(25);

  OneStepOfControlS(time);

  RightFootRefval = m_dotRightFootPosition;
  sotDEBUGOUT(25);
  return RightFootRefval;
}

MatrixHomogeneous &PatternGenerator::getInitLeftFootRef(
    MatrixHomogeneous &LeftFootRefVal, int /*time*/) {
  sotDEBUGIN(25);

  LeftFootRefVal = m_InitLeftFootPosition;
  sotDEBUGOUT(25);
  return LeftFootRefVal;
}
MatrixHomogeneous &PatternGenerator::getInitRightFootRef(
    MatrixHomogeneous &RightFootRefval, int /*time*/) {
  sotDEBUGIN(25);

  RightFootRefval = m_InitRightFootPosition;
  sotDEBUGOUT(25);
  return RightFootRefval;
}

MatrixHomogeneous &PatternGenerator::getFlyingFootRef(
    MatrixHomogeneous &FlyingFootRefval, int time) {
  sotDEBUGIN(25);
  OneStepOfControlS(time);
  FlyingFootRefval = m_FlyingFootPosition;
  sotDEBUGOUT(25);
  return FlyingFootRefval;
}

bool &PatternGenerator ::getLeftFootContact(bool &res, int time) {
  sotDEBUGIN(25);
  OneStepOfControlS(time);
  res = m_leftFootContact;
  sotDEBUGOUT(25);
  return res;
}

bool &PatternGenerator ::getRightFootContact(bool &res, int time) {
  sotDEBUGIN(25);
  OneStepOfControlS(time);
  res = m_rightFootContact;
  sotDEBUGOUT(25);
  return res;
}

int &PatternGenerator ::getContactPhase(int &res, int time) {
  sotDEBUGIN(25);
  OneStepOfControlS(time);
  res = m_ContactPhase;
  sotDEBUGOUT(25);
  return res;
}

int &PatternGenerator::InitOneStepOfControl(int &dummy, int /*time*/) {
  sotDEBUGIN(15);
  // TODO: modified first to avoid the loop.
  firstSINTERN.setReady(false);
  //  buildModel();
  // Todo: modified the order of the calls
  // OneStepOfControlS(time);
  sotDEBUGIN(15);
  return dummy;
}

void PatternGenerator::getAbsoluteWaistPosAttHomogeneousMatrix(
    MatrixHomogeneous &aWaistMH) {
  const double cr = cos(m_WaistAttitudeAbsolute(0));  // ROLL
  const double sr = sin(m_WaistAttitudeAbsolute(0));
  const double cp = cos(m_WaistAttitudeAbsolute(1));  // PITCH
  const double sp = sin(m_WaistAttitudeAbsolute(1));
  const double cy = cos(m_WaistAttitudeAbsolute(2));  // YAW
  const double sy = sin(m_WaistAttitudeAbsolute(2));

  aWaistMH.matrix().setZero();

  aWaistMH(0, 0) = cy * cp;
  aWaistMH(0, 1) = cy * sp * sr - sy * cr;
  aWaistMH(0, 2) = cy * sp * cr + sy * sr;
  aWaistMH(0, 3) = m_WaistPositionAbsolute(0);

  aWaistMH(1, 0) = sy * cp;
  aWaistMH(1, 1) = sy * sp * sr + cy * cr;
  aWaistMH(1, 2) = sy * sp * cr - cy * sr;
  aWaistMH(1, 3) = m_WaistPositionAbsolute(1);

  aWaistMH(2, 0) = -sp;
  aWaistMH(2, 1) = cp * sr;
  aWaistMH(2, 2) = cp * cr;
  aWaistMH(2, 3) = m_WaistPositionAbsolute(2);

  aWaistMH(3, 3) = 1.0;
}

void PatternGenerator::FromAbsoluteFootPosToDotHomogeneous(
    pg::FootAbsolutePosition aFootPosition, MatrixHomogeneous &aFootMH,
    MatrixHomogeneous &adotFootMH) {
  MatrixRotation dRot, Twist, Rot;
  adotFootMH.setIdentity();
  FromAbsoluteFootPosToHomogeneous(aFootPosition, aFootMH);

  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++) Rot(i, j) = aFootMH(i, j);

  Twist(0, 0) = 0.0;
  Twist(0, 1) = -aFootPosition.dtheta;
  Twist(0, 2) = aFootPosition.domega;
  Twist(1, 0) = aFootPosition.dtheta;
  Twist(1, 1) = 0.0;
  Twist(1, 2) = aFootPosition.domega2;
  Twist(2, 0) = -aFootPosition.domega;
  Twist(2, 1) = -aFootPosition.domega2;
  Twist(2, 2) = 0.0;

  dRot = Twist * Rot;

  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++) adotFootMH(i, j) = dRot(i, j);

  adotFootMH(0, 3) = aFootPosition.dx;
  adotFootMH(1, 3) = aFootPosition.dy;
  adotFootMH(2, 3) = aFootPosition.dz;
}

void PatternGenerator::FromAbsoluteFootPosToHomogeneous(
    pg::FootAbsolutePosition aFootPosition, MatrixHomogeneous &aFootMH) {
  double c, s, co, so;
  c = cos(aFootPosition.theta * M_PI / 180.0);
  s = sin(aFootPosition.theta * M_PI / 180.0);

  co = cos(aFootPosition.omega * M_PI / 180.0);
  so = sin(aFootPosition.omega * M_PI / 180.0);

  aFootMH(0, 0) = c * co;
  aFootMH(0, 1) = -s;
  aFootMH(0, 2) = c * so;
  aFootMH(1, 0) = s * co;
  aFootMH(1, 1) = c;
  aFootMH(1, 2) = s * so;
  aFootMH(2, 0) = -so;
  aFootMH(2, 1) = 0;
  aFootMH(2, 2) = co;
  aFootMH(3, 0) = 0;
  aFootMH(3, 1) = 0;
  aFootMH(3, 2) = 0;

  aFootMH(0, 3) = aFootPosition.x + m_AnkleSoilDistance * so;
  aFootMH(1, 3) = aFootPosition.y;
  aFootMH(2, 3) = aFootPosition.z + m_AnkleSoilDistance * co;
  aFootMH(3, 3) = 1.0;
}

void PatternGenerator::SubsamplingFootPos(
    pg::FootAbsolutePosition &PrevFootPosition,
    pg::FootAbsolutePosition &NextFootPosition,
    MatrixHomogeneous &FootPositionOut, MatrixHomogeneous &dotFootPositionOut,
    unsigned int &count) {
  pg::FootAbsolutePosition lFootPosition;

  lFootPosition.x =
      PrevFootPosition.x +
      (NextFootPosition.x - PrevFootPosition.x) * (double)(count % 5) / 5.0;
  lFootPosition.y =
      PrevFootPosition.y +
      (NextFootPosition.y - PrevFootPosition.y) * (double)(count % 5) / 5.0;
  lFootPosition.z =
      PrevFootPosition.z +
      (NextFootPosition.z - PrevFootPosition.z) * (double)(count % 5) / 5.0;
  lFootPosition.theta = PrevFootPosition.theta +
                        (NextFootPosition.theta - PrevFootPosition.theta) *
                            (double)(count % 5) / 5.0;
  lFootPosition.omega = PrevFootPosition.omega +
                        (NextFootPosition.omega - PrevFootPosition.omega) *
                            (double)(count % 5) / 5.0;
  lFootPosition.omega2 = PrevFootPosition.omega2 +
                         (NextFootPosition.omega2 - PrevFootPosition.omega2) *
                             (double)(count % 5) / 5.0;

  /* Fill in the homogeneous matrix using the world reference frame*/
  FromAbsoluteFootPosToDotHomogeneous(lFootPosition, FootPositionOut,
                                      dotFootPositionOut);
}

void PatternGenerator::SubsamplingVector(dynamicgraph::Vector &PrevPosition,
                                         dynamicgraph::Vector &NextPosition,
                                         dynamicgraph::Vector &PositionOut,
                                         unsigned int &count) {
  for (unsigned int i = 0; i < 3; i++) {
    PositionOut(i) = PrevPosition(i) + (NextPosition(i) - PrevPosition(i)) *
                                           (double)(count % 5) / 5.0;
  }
}

void PatternGenerator::CopyFootPosition(
    pg::FootAbsolutePosition &FootPositionIn,
    pg::FootAbsolutePosition &FootPositionOut) {
  FootPositionOut.x = FootPositionIn.x;
  FootPositionOut.y = FootPositionIn.y;
  FootPositionOut.z = FootPositionIn.z;
  FootPositionOut.theta = FootPositionIn.theta;
  FootPositionOut.omega = FootPositionIn.omega;
  FootPositionOut.omega2 = FootPositionIn.omega2;
}

int &PatternGenerator::OneStepOfControl(int &dummy, int time) {
  m_LocalTime = time;
  int lSupportFoot;  // Local support foot.
  // Default value
  m_JointErrorValuesForWalking.fill(0.0);
  const dynamicgraph::Vector::Index robotSize =
      m_JointErrorValuesForWalking.size() + 6;

  try {
    for (unsigned int i = 0; i < 3; i++) m_ZMPRefPos(i) = m_ZMPPrevious[i];
  } catch (...) {
    m_ZMPRefPos(0) = m_ZMPRefPos(1) = m_ZMPRefPos(2) = 0.0;
    m_ZMPRefPos(3) = 1.0;
  };
  //  m_WaistAttitudeAbsolute.fill(0);
  //  m_WaistPositionAbsolute.fill(0);

  try {
    m_LeftFootPosition = LeftFootCurrentPosSIN(time);
    m_RightFootPosition = RightFootCurrentPosSIN(time);
  } catch (...) {
  };

  try {
    m_VelocityReference = velocitydesSIN(time);
  } catch (...) {
  };

  try {
    m_trigger = triggerSIN(time);
  } catch (...) {
  };

  sotDEBUG(25) << "LeftFootCurrentPos:  " << m_LeftFootPosition << endl;
  sotDEBUG(25) << "RightFootCurrentPos:  " << m_RightFootPosition << endl;

  sotDEBUGIN(15);

  if (m_PGI != 0 and m_trigger) {
    // TODO: Calling firstSINTERN may cause an infinite loop
    // since the function initonestepofcontrol calls without
    // control this actual function. 'Hopefully', the function
    // pointer of firstSINTERN has been earlier destroyed
    // by setconstant(0).
    firstSINTERN(time);
    Vector CurrentState = motorControlJointPositionSIN(time);
    assert(CurrentState.size() == robotSize);

    /*! \brief Absolute Position for the left and right feet. */
    pg::FootAbsolutePosition lLeftFootPosition, lRightFootPosition;
    lLeftFootPosition.x = 0.0;
    lLeftFootPosition.y = 0.0;
    lLeftFootPosition.z = 0.0;
    lRightFootPosition.x = 0.0;
    lRightFootPosition.y = 0.0;
    lRightFootPosition.z = 0.0;
    /*! \brief Absolute position of the reference CoM. */

    pg::COMState lCOMRefState;
    sotDEBUG(45) << "mc = " << CurrentState << std::endl;

    Eigen::VectorXd CurrentConfiguration(robotSize);
    Eigen::VectorXd CurrentVelocity(robotSize);
    Eigen::VectorXd CurrentAcceleration(robotSize);
    Eigen::VectorXd ZMPTarget(3);
    ZMPTarget.setZero();

    sotDEBUG(25) << "Before One Step of control " << lCOMRefState.x[0] << " "
                 << lCOMRefState.y[0] << " " << lCOMRefState.z[0] << endl;
    sotDEBUG(4) << " VelocityReference " << m_VelocityReference << endl;

    m_PGI->setVelocityReference(m_VelocityReference(0), m_VelocityReference(1),
                                m_VelocityReference(2));

    try {
      if (m_feedBackControl) {
        Eigen::Vector3d curCoMState;
        Eigen::Vector3d curZMP;

        curCoMState = comStateSIN(time);
        curZMP = zmpSIN(time);

        lCOMRefState.x[0] = curCoMState(0);
        lCOMRefState.y[0] = curCoMState(1);
        lCOMRefState.z[0] = curCoMState(2);
        lCOMRefState.x[2] =
            (lCOMRefState.x[0] - curZMP(0)) * 9.81 / lCOMRefState.z[0];
        lCOMRefState.y[2] =
            (lCOMRefState.y[0] - curZMP(1)) * 9.81 / lCOMRefState.z[0];
        lCOMRefState.z[2] = 0.0;

        //          for (unsigned i=0 ; i<3 ; ++i)
        //            lCOMRefState.x[i] = curCoMState(i) ;
        //          for (unsigned i=0 ; i<3 ; ++i)
        //            lCOMRefState.y[i] = curCoMState(i+3) ;
        //          for (unsigned i=0 ; i<3 ; ++i)
        //            lCOMRefState.z[i] = curCoMState(i+6) ;

        //          ZMPTarget(0) = lCOMRefState.x[0]
        // -lCOMRefState.x[2]*lCOMRefState.z[0]/9.81 ;
        //          ZMPTarget(1) = lCOMRefState.y[0]-
        //  lCOMRefState.y[2]*lCOMRefState.z[0]/9.81 ;
        //          ZMPTarget(2) = 0.0;
        // to be fixed considering the support foot
      }
    } catch (...) {
      cout << "problems with signals reading" << endl;
      useFeedBackSignals(false);
    };

    try {
      if (m_forceFeedBack) {
        Vector extForce(3);
        extForce = forceSIN(time);
        if (time < 50 * 0.005) {
          m_initForce = extForce;
        }
        extForce -= m_initForce;
        unsigned int n = 321;
        if (m_bufferForce.size() < n - 1) {
          m_bufferForce.push_back(extForce);
        } else {
          m_bufferForce.push_back(extForce);
          double ltmp1(0.0), ltmp2(0.0), ltmp3(0.0);
          for (unsigned int k = 0; k < m_filterWindow.size(); k++) {
            ltmp1 += m_filterWindow[k] * m_bufferForce[n - 1 - k](0);
            ltmp2 += m_filterWindow[k] * m_bufferForce[n - 1 - k](1);
            ltmp3 += m_filterWindow[k] * m_bufferForce[n - 1 - k](2);
          }
          extForce(0) = ltmp1;
          extForce(1) = ltmp2;
          extForce(2) = ltmp3;
          m_bufferForce.pop_front();
        }
        double threshold = 7.0;
        double thresholdy = 4.0;
        if (extForce(0) > threshold) extForce(0) = threshold;
        if (extForce(0) < -threshold) extForce(0) = -threshold;

        if (extForce(1) > thresholdy) extForce(1) = thresholdy;
        if (extForce(1) < -thresholdy) extForce(1) = -thresholdy;

        if (extForce(2) > threshold) extForce(2) = threshold;
        if (extForce(2) < -threshold) extForce(2) = -threshold;

        if ((extForce(0) * extForce(0) + extForce(1) * extForce(1)) < 100) {
          extForce(0) = 0.0;
          extForce(1) = 0.0;
        }
        m_currentForces = extForce;
        ostringstream oss("");
        // oss << ":perturbationforce " << extForce(0) << " "
        // << extForce(1) << " " << extForce(2);
        oss << ":perturbationforce " << -m_currentForces(1) << " "
            << /*m_currentForces(0)*/ 0.0 << " " << m_currentForces(2);
        // cout << oss.str() << endl ;
        pgCommandLine(oss.str());
      }
    } catch (...) {
      // cout << "problems with force signals reading" << endl;
    };

    if (m_count % 5 == 0) {
      if (m_count > 1)  // Change of previous state
      {
        for (unsigned int i = 0; i < 3; i++) {
          m_PrevSamplingCOMRefPos(i) = m_NextSamplingCOMRefPos(i);
          m_PrevSamplingdCOMRefPos(i) = m_NextSamplingdCOMRefPos(i);
          m_PrevSamplingddCOMRefPos(i) = m_NextSamplingddCOMRefPos(i);
          m_PrevSamplingWaistAttAbs(i) = m_NextSamplingWaistAttAbs(i);
        }
        CopyFootPosition(m_NextSamplingLeftFootAbsPos,
                         m_PrevSamplingLeftFootAbsPos);
        CopyFootPosition(m_NextSamplingRightFootAbsPos,
                         m_PrevSamplingRightFootAbsPos);
      } else {
        for (unsigned int i = 0; i < 3; i++) {
          m_PrevSamplingCOMRefPos(i) = m_InitCOMRefPos(i);
          m_PrevSamplingdCOMRefPos(i) = 0.0;
          m_PrevSamplingddCOMRefPos(i) = 0.0;
          m_PrevSamplingWaistAttAbs(i) = m_InitWaistRefAtt(i);
        }
        CopyFootPosition(m_InitLeftFootAbsPos, m_PrevSamplingLeftFootAbsPos);
        CopyFootPosition(m_InitRightFootAbsPos, m_PrevSamplingRightFootAbsPos);
      }
      // Test if the pattern value has some value to provide.
      if (m_PGI->RunOneStepOfTheControlLoop(
              CurrentConfiguration, CurrentVelocity, CurrentAcceleration,
              ZMPTarget, lCOMRefState, lLeftFootPosition, lRightFootPosition)) {
        sotDEBUG(25) << "After One Step of control " << endl
                     << "CurrentState:" << CurrentState << endl
                     << "CurrentConfiguration:" << CurrentConfiguration << endl;

        m_ZMPRefPos(0) = ZMPTarget[0];
        m_ZMPRefPos(1) = ZMPTarget[1];
        m_ZMPRefPos(2) = ZMPTarget[2];
        m_ZMPRefPos(3) = 1.0;
        sotDEBUG(2) << "ZMPTarget returned by the PG: " << m_ZMPRefPos << endl;
        for (int i = 0; i < 3; i++) {
          m_WaistPositionAbsolute(i) = CurrentConfiguration(i);
          m_WaistAttitudeAbsolute(i) = CurrentConfiguration(i + 3);
        }

        getAbsoluteWaistPosAttHomogeneousMatrix(m_WaistAttitudeMatrixAbsolute);

        m_COMRefPos(0) = lCOMRefState.x[0];
        m_COMRefPos(1) = lCOMRefState.y[0];
        m_COMRefPos(2) = lCOMRefState.z[0];
        sotDEBUG(2) << "COMRefPos returned by the PG: " << m_COMRefPos << endl;

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

        sotDEBUG(2) << "dCOMRefPos returned by the PG: " << m_dCOMRefPos
                    << endl;
        sotDEBUG(2) << "CurrentState.size()" << CurrentState.size() << endl;
        sotDEBUG(2) << "CurrentConfiguration.size()"
                    << CurrentConfiguration.size() << endl;
        sotDEBUG(2) << "m_JointErrorValuesForWalking.size(): "
                    << m_JointErrorValuesForWalking.size() << endl;

        // In this setting we assume that there is a
        // proper mapping between
        // CurrentState and CurrentConfiguration.
        Vector::Index SizeCurrentState = CurrentState.size();
        Vector::Index SizeCurrentConfiguration =
            CurrentConfiguration.size() - 6;
        Vector::Index MinSize =
            std::min(SizeCurrentState, SizeCurrentConfiguration);

        if (m_JointErrorValuesForWalking.size() >= MinSize) {
          for (unsigned int li = 0; li < MinSize; li++)
            m_JointErrorValuesForWalking(li) =
                (CurrentConfiguration(li + 6) - CurrentState(li)) / m_TimeStep;
        } else {
          std::cout << "The state of the robot and the one "
                    << "return by the WPG are different" << std::endl;
          sotDEBUG(25) << "Size not coherent between "
                       << "CurrentState and "
                       << "m_JointErrorValuesForWalking: "
                       << CurrentState.size() << " "
                       << m_JointErrorValuesForWalking.size() << " " << endl;
        }
        sotDEBUG(2) << "Juste after updating "
                    << "m_JointErrorValuesForWalking" << endl;

        sotDEBUG(1) << "lLeftFootPosition : " << lLeftFootPosition.x << " "
                    << lLeftFootPosition.y << " " << lLeftFootPosition.z << " "
                    << lLeftFootPosition.theta << endl;

        sotDEBUG(1) << "lRightFootPosition : " << lRightFootPosition.x << " "
                    << lRightFootPosition.y << " " << lRightFootPosition.z
                    << " " << lRightFootPosition.theta << endl;

        sotDEBUG(25) << "lCOMPosition : " << lCOMRefState.x[0] << " "
                     << lCOMRefState.y[0] << " " << lCOMRefState.z[0] << endl;

        // Set Next position ---------------------------------------------
        for (unsigned int i = 0; i < 3; i++) {
          m_NextSamplingCOMRefPos(i) = m_COMRefPos(i);
          m_NextSamplingdCOMRefPos(i) = m_dCOMRefPos(i);
          m_NextSamplingddCOMRefPos(i) = m_ddCOMRefPos(i);
          m_NextSamplingWaistAttAbs(i) = m_WaistAttitudeAbsolute(i);
        }

        CopyFootPosition(lRightFootPosition, m_NextSamplingRightFootAbsPos);
        CopyFootPosition(lLeftFootPosition, m_NextSamplingLeftFootAbsPos);
      } else {
        sotDEBUG(1) << "Error while compute one step of PG." << m_dataInProcess
                    << std::endl;
        // TODO: SOT_THROW
        if (m_dataInProcess == 1) {
          MatrixHomogeneous invInitLeftFootRef, Diff;
          invInitLeftFootRef = m_InitLeftFootPosition.inverse();
          Diff = invInitLeftFootRef * m_LeftFootPosition;

          m_k_Waist_kp1 = m_k_Waist_kp1 * Diff;
        }
        m_dataInProcess = 0;
      }
      sotDEBUG(25) << "After computing error " << m_JointErrorValuesForWalking
                   << endl;
    }
    // Subsampling

    SubsamplingVector(m_PrevSamplingCOMRefPos, m_NextSamplingCOMRefPos,
                      m_COMRefPos, m_count);
    SubsamplingVector(m_PrevSamplingdCOMRefPos, m_NextSamplingdCOMRefPos,
                      m_dCOMRefPos, m_count);
    SubsamplingVector(m_PrevSamplingddCOMRefPos, m_NextSamplingddCOMRefPos,
                      m_ddCOMRefPos, m_count);
    SubsamplingVector(m_PrevSamplingWaistAttAbs, m_NextSamplingWaistAttAbs,
                      m_WaistAttitudeAbsolute, m_count);
    SubsamplingFootPos(m_PrevSamplingRightFootAbsPos,
                       m_NextSamplingRightFootAbsPos, m_RightFootPosition,
                       m_dotRightFootPosition, m_count);
    SubsamplingFootPos(m_PrevSamplingLeftFootAbsPos,
                       m_NextSamplingLeftFootAbsPos, m_LeftFootPosition,
                       m_dotLeftFootPosition, m_count);

    getAbsoluteWaistPosAttHomogeneousMatrix(m_WaistAttitudeMatrixAbsolute);
    // end added lines for test on waist

    m_count++;

    Eigen::Matrix<double, 4, 1> newRefPos, oldRefPos;
    oldRefPos(0) = m_COMRefPos(0);
    oldRefPos(1) = m_COMRefPos(1);
    oldRefPos(2) = m_COMRefPos(2);
    oldRefPos(3) = 1.0;
    newRefPos = m_MotionSinceInstanciationToThisSequence * oldRefPos;
    m_COMRefPos(0) = newRefPos(0);
    m_COMRefPos(1) = newRefPos(1);
    m_COMRefPos(2) = newRefPos(2);

    oldRefPos(0) = m_ZMPRefPos(0);
    oldRefPos(1) = m_ZMPRefPos(1);
    oldRefPos(2) = m_ZMPRefPos(2);
    oldRefPos(3) = 1.0;
    newRefPos = m_MotionSinceInstanciationToThisSequence * oldRefPos;
    m_ZMPRefPos(0) = newRefPos(0);
    m_ZMPRefPos(1) = newRefPos(1);
    m_ZMPRefPos(2) = newRefPos(2);

    /* We assume that the left foot is always the origin
       of the new frame. */
    m_LeftFootPosition =
        m_MotionSinceInstanciationToThisSequence * m_LeftFootPosition;
    m_RightFootPosition =
        m_MotionSinceInstanciationToThisSequence * m_RightFootPosition;

    sotDEBUG(25) << "lLeftFootPosition.stepType: " << lLeftFootPosition.stepType
                 << " lRightFootPosition.stepType: "
                 << lRightFootPosition.stepType << endl;

    // Find the support foot feet.
    // If stepType = -1 -> single support phase on the dedicated foot
    // If stepType = 10 -> double support phase (both feet should have
    // stepType=10) If stepType = 11 -> double support phase between 2 single
    // support phases for Kajita Algorithm
    if ((lLeftFootPosition.stepType == 10) ||
        (lRightFootPosition.stepType == 10) ||
        (lLeftFootPosition.stepType == 11) ||
        (lRightFootPosition.stepType == 11)) {
      m_leftFootContact = true;
      m_rightFootContact = true;
      m_ContactPhase = DOUBLE_SUPPORT_PHASE;
    }

    if (lLeftFootPosition.stepType == -1) {
      lSupportFoot = 1;
      m_leftFootContact = true;
      // It is almost certain that when there is single support on a foot
      // the other one cannot be in simple support (neither in double support)
      // This if is certainly always true -> to be checked
      if (lRightFootPosition.stepType != -1) {
        m_rightFootContact = false;
        m_ContactPhase = LEFT_SUPPORT_PHASE;
      }
    } else if (lRightFootPosition.stepType == -1) {
      lSupportFoot = 0;
      m_rightFootContact = true;
      // It is almost certain that when there is single support on a foot
      // the other one cannot be in simple support (neither in double support)
      // This if is certainly always true -> to be checked
      if (lLeftFootPosition.stepType != -1) {
        m_leftFootContact = false;
        m_ContactPhase = RIGHT_SUPPORT_PHASE;
      }
    } else
    /* m_LeftFootPosition.z ==m_RightFootPosition.z
       We keep the previous support foot half the time
       of the double support phase..
       */
    {
      lSupportFoot = m_SupportFoot;
    }

    /* Update the class related member. */
    m_SupportFoot = lSupportFoot;

    // Waist
    // ----------------------------------------------------------------------

    if ((m_ReferenceFrame == EGOCENTERED_FRAME) ||
        (m_ReferenceFrame == LEFT_FOOT_CENTERED_FRAME) ||
        (m_ReferenceFrame == WAIST_CENTERED_FRAME)) {
      sotDEBUG(25) << "Inside egocentered frame " << endl;
      MatrixHomogeneous PoseOrigin, iPoseOrigin, WaistPoseAbsolute;

      getAbsoluteWaistPosAttHomogeneousMatrix(WaistPoseAbsolute);

      if (m_ReferenceFrame == EGOCENTERED_FRAME) {
        if (m_SupportFoot == 1)
          PoseOrigin = m_LeftFootPosition;
        else
          PoseOrigin = m_RightFootPosition;
      } else if (m_ReferenceFrame == LEFT_FOOT_CENTERED_FRAME) {
        PoseOrigin = m_LeftFootPosition;
      } else if (m_ReferenceFrame == WAIST_CENTERED_FRAME) {
        PoseOrigin = WaistPoseAbsolute;
      }
      iPoseOrigin = PoseOrigin.inverse();

      sotDEBUG(25) << "Old ComRef:  " << m_COMRefPos << endl;
      sotDEBUG(25) << "Old LeftFootRef:  " << m_LeftFootPosition << endl;
      sotDEBUG(25) << "Old RightFootRef:  " << m_RightFootPosition << endl;
      sotDEBUG(25) << "Old PoseOrigin:  " << PoseOrigin << endl;

      Eigen::Matrix<double, 4, 1> lVZMPRefPos, lV2ZMPRefPos;
      Eigen::Matrix<double, 4, 1> lVCOMRefPos, lV2COMRefPos;

      for (unsigned int li = 0; li < 3; li++) {
        lVZMPRefPos(li) = m_ZMPRefPos(li);
        lVCOMRefPos(li) = m_COMRefPos(li);
      }
      lVZMPRefPos(3) = lVCOMRefPos(3) = 1.0;

      // We do not touch to ZMP.
      lV2ZMPRefPos = iPoseOrigin * (WaistPoseAbsolute * lVZMPRefPos);

      // Put the CoM reference pos in the Pos
      // Origin reference frame.
      lV2COMRefPos = iPoseOrigin * lVCOMRefPos;

      MatrixHomogeneous lMLeftFootPosition = m_LeftFootPosition;
      MatrixHomogeneous lMRightFootPosition = m_RightFootPosition;

      m_LeftFootPosition = iPoseOrigin * lMLeftFootPosition;
      m_RightFootPosition = iPoseOrigin * lMRightFootPosition;

      for (unsigned int i = 0; i < 3; i++) {
        m_ZMPRefPos(i) = lV2ZMPRefPos(i);
        m_COMRefPos(i) = lV2COMRefPos(i);
      }

      WaistPoseAbsolute = iPoseOrigin * WaistPoseAbsolute;

      MatrixRotation newWaistRot;
      newWaistRot = WaistPoseAbsolute.linear();
      VectorRollPitchYaw newWaistRPY;
      newWaistRPY = (newWaistRot.eulerAngles(2, 1, 0)).reverse();
      m_WaistAttitude = newWaistRPY;

      m_WaistPosition = WaistPoseAbsolute.translation();

      m_WaistAttitudeMatrixAbsolute = WaistPoseAbsolute;

      sotDEBUG(25) << "ComRef:  " << m_COMRefPos << endl;
      sotDEBUG(25) << "iPoseOrigin:  " << iPoseOrigin << endl;
    }
    sotDEBUG(25) << "After egocentered frame " << endl;

    sotDEBUG(25) << "ComRef:  " << m_COMRefPos << endl;
    sotDEBUG(25) << "LeftFootRef:  " << m_LeftFootPosition << endl;
    sotDEBUG(25) << "RightFootRef:  " << m_RightFootPosition << endl;
    sotDEBUG(25) << "ZMPRefPos:  " << m_ZMPRefPos << endl;
    sotDEBUG(25) << "m_MotionSinceInstanciationToThisSequence"
                 << m_MotionSinceInstanciationToThisSequence << std::endl;

    for (unsigned int i = 0; i < 3; i++) {
      m_ZMPPrevious[i] = m_ZMPRefPos(i);
    }

    m_dataInProcess = 1;

  } else if (!m_trigger) {
    for (unsigned int i = 0; i < 3; i++) {
      m_COMRefPos(i) = m_InitCOMRefPos(i);
      m_ZMPRefPos(i) = m_InitZMPRefPos(i);
    }
    m_RightFootPosition = m_InitRightFootPosition;
    m_LeftFootPosition = m_InitLeftFootPosition;
  } else {
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

/* --- PARAMS ------------------------------------------------ */
/* --- PARAMS ------------------------------------------------ */

void PatternGenerator::initCommands(void) {
  using namespace command;
  addCommand("setURDFpath",
             makeCommandVoid1(*this, &PatternGenerator::setURDFFile,
                              docCommandVoid1("Set URDF directory+name.",
                                              "string (path name)")));
  addCommand("setSRDFpath",
             makeCommandVoid1(*this, &PatternGenerator::setSRDFFile,
                              docCommandVoid1("Set SRDF directory+name.",
                                              "string (file name)")));

  addCommand(
      "setXmlRank",
      makeCommandVoid1(*this, &PatternGenerator::setXmlRankFile,
                       docCommandVoid1("Set XML rank file directory+name.",
                                       "string (file name)")));

  std::string docstring =
      "    \n"
      "    Set foot parameters\n"
      "      Input:\n"
      "        - a floating point number: the sole length,\n"
      "        - a floating point number: the sole width,\n"
      "    \n";
  addCommand(
      "setSoleParameters",
      makeCommandVoid2(*this, &PatternGenerator::setSoleParameters, docstring));

  addCommand("addJointMapping",
             makeCommandVoid2(*this, &PatternGenerator::addJointMapping,
                              docCommandVoid1("Map link names.",
                                              "string (link name)"
                                              "string (rep name)")));

  addCommand("setParamPreview",
             makeCommandVoid1(*this, &PatternGenerator::setParamPreviewFile,
                              docCommandVoid1("Set [guess what!] file",
                                              "string (path/filename)")));

  // for the setFiles, need to implement the makeCmdVoid5... later
  // displayfiles... later too
  addCommand(
      "buildModel",
      makeCommandVoid0(
          *this, (void(PatternGenerator::*)(void)) & PatternGenerator::buildPGI,
          docCommandVoid0("From the files, parse and build the robot model and"
                          " the Walking Pattern Generator.")));
  addCommand(
      "initState",
      makeCommandVoid0(
          *this,
          (void(PatternGenerator::*)(void)) & PatternGenerator::InitState,
          docCommandVoid0("From q and model, compute the initial geometry.")));

  addCommand(
      "frameReference",
      makeCommandVoid1(
          *this, &PatternGenerator::setReferenceFromString,
          docCommandVoid1("Set the reference.",
                          "string among "
                          "World|Egocentered|LeftFootcentered|Waistcentered")));

  addCommand("getTimeStep",
             makeDirectGetter(*this, &m_TimeStep,
                              docDirectGetter("timestep", "double")));

  addCommand("setTimeStep",
             makeDirectSetter(*this, &m_TimeStep,
                              docDirectSetter("timestep", "double")));

  addCommand("getInitByRealState",
             makeDirectGetter(*this, &m_InitPositionByRealState,
                              docDirectGetter("initByRealState", "bool")));

  addCommand("setInitByRealState",
             makeDirectSetter(*this, &m_InitPositionByRealState,
                              docDirectSetter("initByRealState", "bool")));

  addCommand(
      "addOnLineStep",
      makeCommandVoid3(*this, &PatternGenerator::addOnLineStep,
                       docCommandVoid3("Add a step on line.", "double (x)",
                                       "double (y)", "double (theta)")));

  addCommand(
      "addStep",
      makeCommandVoid3(*this, &PatternGenerator::addOnLineStep,
                       docCommandVoid3("Add a step in the stack.", "double (x)",
                                       "double (y)", "double (theta)")));

  addCommand(
      "parseCmd",
      makeCommandVoid1(
          *this, &PatternGenerator::pgCommandLine,
          docCommandVoid1("Send the command line to the internal pg object.",
                          "string (command line)")));

  addCommand(
      "feedBackControl",
      makeCommandVoid1(
          *this, &PatternGenerator::useFeedBackSignals,
          docCommandVoid1("Enable or disable the use of the CoMfullState "
                          "Signal inside the pg.",
                          "string (true or false)")));

  addCommand(
      "dynamicFilter",
      makeCommandVoid1(
          *this, &PatternGenerator::useDynamicFilter,
          docCommandVoid1("Enable or disable the use of the CoMfullState "
                          "Signal inside the pg.",
                          "string (true or false)")));

  // Change next step : todo (deal with FootAbsolutePosition...).

  addCommand(
      "debug",
      makeCommandVoid0(
          *this, (void(PatternGenerator::*)(void)) & PatternGenerator::debug,
          docCommandVoid0("Launch a debug command.")));
}

void PatternGenerator::debug(void) {
  std::cout << "t = " << dataInProcessSOUT.getTime() << std::endl;
  std::cout << "deptype = " << dataInProcessSOUT.dependencyType << std::endl;
  std::cout << "child = " << dataInProcessSOUT.updateFromAllChildren
            << std::endl;
  std::cout << "last = " << dataInProcessSOUT.lastAskForUpdate << std::endl;

  std::cout << "inprocess = " << dataInProcessSOUT.needUpdate(40) << std::endl;
  std::cout << "onestep = " << OneStepOfControlS.needUpdate(40) << std::endl;

  dataInProcessSOUT.Signal<unsigned int, int>::access(1);
}

void PatternGenerator::addOnLineStep(const double &x, const double &y,
                                     const double &th) {
  assert(m_PGI != 0);
  m_PGI->AddOnLineStep(x, y, th);
}
void PatternGenerator::addStep(const double &x, const double &y,
                               const double &th) {
  assert(m_PGI != 0);
  m_PGI->AddStepInStack(x, y, th);
}
void PatternGenerator::pgCommandLine(const std::string &cmdline) {
  assert(m_PGI != 0);
  std::istringstream cmdArgs(cmdline);
  m_PGI->ParseCmd(cmdArgs);
}

void PatternGenerator::useFeedBackSignals(const bool &feedBack) {
  m_feedBackControl = feedBack;
  string cmdBool = feedBack ? "true" : "false";
  assert(m_PGI != 0);
  std::istringstream cmdArgs(":feedBackControl " + cmdBool);
  m_PGI->ParseCmd(cmdArgs);
}

void PatternGenerator::useDynamicFilter(const bool &dynamicFilter) {
  m_feedBackControl = dynamicFilter;
  string cmdBool = dynamicFilter ? "true" : "false";
  assert(m_PGI != 0);
  std::istringstream cmdArgs(":useDynamicFilter " + cmdBool);
  m_PGI->ParseCmd(cmdArgs);
}

int PatternGenerator::stringToReferenceEnum(const std::string &FrameReference) {
  if (FrameReference == "World")
    return WORLD_FRAME;
  else if (FrameReference == "Egocentered")
    return EGOCENTERED_FRAME;
  else if (FrameReference == "LeftFootcentered")
    return LEFT_FOOT_CENTERED_FRAME;
  else if (FrameReference == "Waistcentered")
    return WAIST_CENTERED_FRAME;
  assert(false &&
         "String name should be in the list "
         "World|Egocentered|LeftFootcentered|Waistcentered");
  return 0;
}

void PatternGenerator::setReferenceFromString(const std::string &str) {
  m_ReferenceFrame = stringToReferenceEnum(str);
}

Vector &PatternGenerator::getjointWalkingErrorPosition(Vector &res, int time) {
  sotDEBUGIN(5);
  OneStepOfControlS(time);
  res = m_JointErrorValuesForWalking;
  sotDEBUGOUT(5);

  return res;
}

int &PatternGenerator::getSupportFoot(int &res, int /*time*/) {
  res = m_SupportFoot;
  return res;
}

VectorRollPitchYaw &PatternGenerator::getWaistAttitude(VectorRollPitchYaw &res,
                                                       int time) {
  sotDEBUGIN(5);
  OneStepOfControlS(time);
  for (unsigned int i = 0; i < 3; ++i) {
    res(i) = m_WaistAttitude(i);
  }
  sotDEBUG(5) << "WaistAttitude: " << m_WaistAttitude << endl;
  sotDEBUGOUT(5);
  return res;
}

dynamicgraph::Vector &PatternGenerator::getdComAttitude(
    dynamicgraph::Vector &res, int time) {
  sotDEBUGIN(5);
  OneStepOfControlS(time);
  res.resize(3);
  for (unsigned int i = 0; i < 3; ++i) {
    res(i) = m_dComAttitude(i);
  }
  sotDEBUG(5) << "ComAttitude: " << m_dComAttitude << endl;
  sotDEBUGOUT(5);
  return res;
}

dynamicgraph::Vector &PatternGenerator::getddComAttitude(
    dynamicgraph::Vector &res, int time) {
  sotDEBUGIN(5);
  OneStepOfControlS(time);
  res.resize(3);
  for (unsigned int i = 0; i < 3; ++i) {
    res(i) = m_ddComAttitude(i);
  }
  sotDEBUG(5) << "ComAttitude: " << m_ddComAttitude << endl;
  sotDEBUGOUT(5);
  return res;
}

dynamicgraph::Vector &PatternGenerator::getComAttitude(
    dynamicgraph::Vector &res, int time) {
  sotDEBUGIN(5);
  OneStepOfControlS(time);
  res.resize(3);
  for (unsigned int i = 0; i < 3; ++i) {
    res(i) = m_ComAttitude(i);
  }
  sotDEBUG(5) << "ComAttitude: " << m_ComAttitude << endl;
  sotDEBUGOUT(5);
  return res;
}

VectorRollPitchYaw &PatternGenerator::getWaistAttitudeAbsolute(
    VectorRollPitchYaw &res, int time) {
  sotDEBUGIN(5);
  OneStepOfControlS(time);
  sotDEBUG(15) << "I survived one step of control" << std::endl;
  for (unsigned int i = 0; i < 3; ++i) {
    res(i) = m_WaistAttitudeAbsolute(i);
  }
  sotDEBUG(5) << "WaistAttitude: " << m_WaistAttitudeAbsolute << endl;
  sotDEBUGOUT(5);
  return res;
}

MatrixHomogeneous &PatternGenerator::getWaistAttitudeMatrixAbsolute(
    MatrixHomogeneous &res, int time) {
  sotDEBUGIN(5);
  OneStepOfControlS(time);
  res = m_WaistAttitudeMatrixAbsolute;
  sotDEBUGOUT(5);
  return res;
}

Vector &PatternGenerator::getWaistPosition(Vector &res, int time) {
  sotDEBUGIN(5);
  OneStepOfControlS(time);
  res = m_WaistPosition;
  sotDEBUG(5) << "WaistPosition: " << m_WaistPosition << endl;
  sotDEBUGOUT(5);
  return res;
}
Vector &PatternGenerator::getWaistPositionAbsolute(Vector &res, int time) {
  sotDEBUGIN(5);
  OneStepOfControlS(time);
  res = m_WaistPositionAbsolute;
  /* ARGH ! ->  res(2) =0*/
  sotDEBUG(5) << "WaistPosition: " << m_WaistPositionAbsolute << endl;
  sotDEBUGOUT(5);
  return res;
}

unsigned &PatternGenerator::getDataInProcess(unsigned &res, int time) {
  sotDEBUGIN(5);
  OneStepOfControlS(time);
  res = m_dataInProcess;
  sotDEBUG(5) << "DataInProcess: " << m_dataInProcess << endl;
  sotDEBUGOUT(5);
  return res;
}

void PatternGenerator::setSoleParameters(const double &inSoleLength,
                                         const double &inSoleWidth) {
  m_soleLength = inSoleLength;
  m_soleWidth = inSoleWidth;
}
}  // namespace sot
}  // namespace dynamicgraph
