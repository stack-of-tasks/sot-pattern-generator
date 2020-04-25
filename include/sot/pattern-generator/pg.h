/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2008
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotDynamic.h
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

#ifndef __SOT_PATTERN_GENERATOR_H__
#define __SOT_PATTERN_GENERATOR_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* STD */
#include <string>

#include <pinocchio/fwd.hpp>

/* SOT */

#include <dynamic-graph/entity.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <pinocchio/fwd.hpp>
#include <sot/pattern-generator/exception-pg.h>
#include <sot/core/flags.hh>
#include <sot/core/matrix-geometry.hh>

/* Pattern Generator */
#include <jrl/walkgen/patterngeneratorinterface.hh>
#include <jrl/walkgen/pinocchiorobot.hh>
namespace pg = PatternGeneratorJRL;

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(pg_EXPORTS)
#define PatternGenerator_EXPORT __declspec(dllexport)
#else
#define PatternGenerator_EXPORT __declspec(dllimport)
#endif
#else
#define PatternGenerator_EXPORT
#endif

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/*! @ingroup tasks

  \brief This class provides dynamically stable CoM, ZMP, feet trajectories.
  It wraps up the algorithms implemented by the walkGenJRL library.



*/
class PatternGenerator_EXPORT PatternGenerator : public Entity {
 public:
  // overload the new[] eigen operator
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! \name Some constant to define the type
    of frame reference.
    @{
  */
  /*! \brief Specify that the frame is expressed  in
    the world reference frame. */
  static const int WORLD_FRAME = 0;

  /*! \brief Specify that the frame is expressed in
    the robot ego centered frame. */
  static const int EGOCENTERED_FRAME = 1;

  /*! \brief Specify that the frame is expressed in
    the left foot centered frame. */
  static const int LEFT_FOOT_CENTERED_FRAME = 2;

  /*! \brief Specify that the frame is expressed in
    the waist centered frame. */
  static const int WAIST_CENTERED_FRAME = 3;

  /*! @} */

  /*! Class name */
 protected:
 public:
  DYNAMIC_GRAPH_ENTITY_DECL();

 protected:
  /*! \brief The model of the robot. */
  pinocchio::Model m_robotModel;
  /*! \brief Pointer towards the robot model inside jrl-walkgen. */
  pg::PinocchioRobot *m_PR;
  /*! \brief The pointor toward the robot data. */
  pinocchio::Data *m_robotData;
  /*! \brief Pointer towards the interface of the pattern generator. */
  pg::PatternGeneratorInterface *m_PGI;

  /*! \name Fields to store name and positions of data files
    @{
  */
  /*! \brief Some information related to the preview control. */
  std::string m_PreviewControlParametersFile;

  /*! \brief Directory+Name where the URDF file of
    the robot's model is located. */
  std::string m_urdfFile;

  /*! \brief Directory+Name where the SRDF file of
    the robot's model is located. */
  std::string m_srdfFile;

  /*! \brief Directory+Name where the Rank of
    the joints are notified */
  std::string m_xmlRankFile;

  std::vector<unsigned> m_wrml2urdfIndex;

  /*! \brief Lenght of the sole */
  double m_soleLength;

  /*! \brief Width of the sole */
  double m_soleWidth;

  /* \brief Special joints map for the parser */
  std::map<std::string, std::string> specialJoints_;

  /*! @} */

  /*! \brief Boolean variable to initialize the object
    by loading an object. */
  bool m_init;

  /*! \brief Boolean variable to initialize the position:
    first through the real state of the robot,
    then through the motor command. */
  bool m_InitPositionByRealState;

  /*! \brief Integer for the support foot. */
  int m_SupportFoot;

  /*! \brief Keep the frame reference */
  int m_ReferenceFrame;

  /*! \brief Distance between ankle and soil */
  double m_AnkleSoilDistance;

  /*! \brief Time step */
  double m_TimeStep;

  /*! \brief Double support phase detected. */
  bool m_DoubleSupportPhaseState;
  int m_DSStartingTime;

  /*! \brief iteration time. */
  unsigned int m_LocalTime;

  /*! \brief count for subsampling. */
  unsigned int m_count;

 public: /* --- CONSTRUCTION --- */
  /*! \brief Default constructor. */
  PatternGenerator(const std::string &name = "PatternGenerator");
  /*! \brief Default destructor. */
  virtual ~PatternGenerator(void);

 public: /* --- MODEL CREATION --- */
  /*! \name Methods related to the data files.
    @{
  */

  /*! \brief Build the pattern generator interface from a Urdf
    and SRDF file. */
  bool buildModel(void);

  /*! \brief Initialize the state of the robot. */
  bool InitState(void);

  /*! \brief Set the directory which contains the parameters
    for the preview control. */
  void setPreviewControlParametersFile(const std::string &filename);

  /*! \brief Set the path which contains the URDF files robot's model. */
  void setURDFFile(const std::string &filename);

  /*! \brief Set the path which contains the SRDF files robot's model.
    More precisely this file describes which joints are the hands, feet.
    For more information please see the documentation of walkGenJRL.
  */
  void setSRDFFile(const std::string &filename);

  /*! \brief Set the path which contains the Joint Rank model. */
  void setXmlRankFile(const std::string &filename);

  /*! \brief Set the name of the file specifying the control parameters
    of the preview control. */
  void setParamPreviewFile(const std::string &filename);

  /*! \brief Set the foot parameters */
  void setSoleParameters(const double &inSoleLength, const double &inSoleWidth);

  /*! \brief Set mapping between a link and actual robot name */
  void addJointMapping(const std::string &link, const std::string &repName);

  /*! \brief Give access directly to the pattern generator...
    You really have to know what your are doing. */
  pg::PatternGeneratorInterface *GetPatternGeneratorInterface() {
    return m_PGI;
  };

  /*! @} */

 public: /* --- SIGNALS --- */
  typedef int Dummy;

  /*! \name Internal signals.
    @{
  */

  /*! \brief Internal signal for initialization and one shot signals. */
  SignalTimeDependent<Dummy, int> firstSINTERN;

  /*! \brief Internal signal to trigger one step of the algorithm. */
  SignalTimeDependent<Dummy, int> OneStepOfControlS;

  /*! @} */

 protected:
  /*! \name Internal methods to access reference trajectories.
    @{
  */
  /*! \brief Internal method to get the reference ZMP at a given time. */
  dynamicgraph::Vector &getZMPRef(dynamicgraph::Vector &res, int time);

  /*! \brief Internal method to get the reference CoM at a given time.*/
  dynamicgraph::Vector &getCoMRef(dynamicgraph::Vector &res, int time);

  /*! \brief Internal method to get the reference dCoM at a given time.*/
  dynamicgraph::Vector &getdCoMRef(dynamicgraph::Vector &res, int time);

  /*! \brief Internal method to get the reference ddCoM at a given time.*/
  dynamicgraph::Vector &getddCoMRef(dynamicgraph::Vector &res, int time);

  /*! \brief Internal method to get the external forces at a given time.*/
  dynamicgraph::Vector &getExternalForces(dynamicgraph::Vector &forces,
                                          int time);

  /*! \brief Internal method to get the position of the left foot. */
  MatrixHomogeneous &getLeftFootRef(MatrixHomogeneous &res, int time);

  /*! \brief Internal method to get the position of the right foot. */
  MatrixHomogeneous &getRightFootRef(MatrixHomogeneous &res, int time);

  /*! \brief Internal method to get the derivative of the left foot. */
  MatrixHomogeneous &getdotLeftFootRef(MatrixHomogeneous &res, int time);

  /*! \brief Internal method to get the derivative of the right foot. */
  MatrixHomogeneous &getdotRightFootRef(MatrixHomogeneous &res, int time);

  /*! \brief Internal method to get the position of the flying foot. */
  MatrixHomogeneous &getFlyingFootRef(MatrixHomogeneous &res, int time);

  /*! \brief Internal method to get the joint position for walking. */
  dynamicgraph::Vector &getjointWalkingErrorPosition(dynamicgraph::Vector &res,
                                                     int time);

  /*! \brief Internal method to get the derivative of the com attitude. */
  dynamicgraph::Vector &getdComAttitude(dynamicgraph::Vector &res, int time);

  /*! \brief Internal method to get the second derivative of the com attitude.
   */
  dynamicgraph::Vector &getddComAttitude(dynamicgraph::Vector &res, int time);

  /*! \brief Internal method to get the attitude of the com. */
  dynamicgraph::Vector &getComAttitude(dynamicgraph::Vector &res, int time);

  /*! \brief Internal method to get the attitude of the waist. */
  VectorRollPitchYaw &getWaistAttitude(VectorRollPitchYaw &res, int time);

  /*! \brief Internal method to get the absolute attitude of the waist. */
  VectorRollPitchYaw &getWaistAttitudeAbsolute(VectorRollPitchYaw &res,
                                               int time);

  /*! \brief Internal method to get the absolute attitude of the waist into
    an homogeneous matrix. */
  MatrixHomogeneous &getWaistAttitudeMatrixAbsolute(MatrixHomogeneous &res,
                                                    int time);

  /*! \brief Internal method to get the dataInPorcess flag */
  unsigned &getDataInProcess(unsigned &res, int time);

  /*! \brief Internal method to get the position of the waist. */
  dynamicgraph::Vector &getWaistPosition(dynamicgraph::Vector &res, int time);

  /*! \brief Internal method to get the absolute position of the waist. */
  dynamicgraph::Vector &getWaistPositionAbsolute(dynamicgraph::Vector &res,
                                                 int time);

  /*! @} */

  /*! \brief Getting the current support foot: 1 Left -1 Right. */
  unsigned int &getSupportFoot(unsigned int &res, int time);

  /*! \brief Trigger the initialization of the algorithm */
  int &InitOneStepOfControl(int &dummy, int time);
  /*! \brief Trigger one step of the algorithm. */
  int &OneStepOfControl(int &dummy, int time);

  /*! \name Keep information computed once for each time.
    @{
  */

  /*! \brief Rigit motion between two waist positions
    at the  beginning of the walking and at the end of the walking. */
  MatrixHomogeneous m_k_Waist_kp1;

  /*! \brief Absolute Position for the left and right feet. */
  MatrixHomogeneous m_LeftFootPosition, m_RightFootPosition;
  PatternGeneratorJRL::FootAbsolutePosition m_PrevSamplingRightFootAbsPos,
      m_PrevSamplingLeftFootAbsPos;
  PatternGeneratorJRL::FootAbsolutePosition m_NextSamplingRightFootAbsPos,
      m_NextSamplingLeftFootAbsPos;
  PatternGeneratorJRL::FootAbsolutePosition m_InitRightFootAbsPos,
      m_InitLeftFootAbsPos;

  /*! \brief Absolute Derivate for the left and right feet. */
  MatrixHomogeneous m_dotLeftFootPosition, m_dotRightFootPosition;

  /*! \brief Initial Absolute Starting Position
    for the left and right feet. */
  MatrixHomogeneous m_InitLeftFootPosition, m_InitRightFootPosition;

  /*! \brief Keep track of the motion between sequence of motions. */
  MatrixHomogeneous m_MotionSinceInstanciationToThisSequence;

  /*! \brief Relative Position of the flying foot. */
  MatrixHomogeneous m_FlyingFootPosition;

  /*! \brief Absolute position of the reference ZMP. */
  dynamicgraph::Vector m_ZMPRefPos;

  /*! \brief Com Attitude: does not really exist apart
    from when the robot
    is seen as an inverted pendulum*/
  dynamicgraph::Vector m_ComAttitude;

  /*! \brief dCom Attitude: does not really exist apart when the robot
    is seen as an inverted pendulum*/
  dynamicgraph::Vector m_dComAttitude;

  /*! \brief ddCom Attitude: does not really exist apart when the robot
    is seen as an inverted pendulum*/
  dynamicgraph::Vector m_ddComAttitude;

  /*! \brief Absolute position of the reference CoM. */
  dynamicgraph::Vector m_COMRefPos;
  dynamicgraph::Vector m_PrevSamplingCOMRefPos;
  dynamicgraph::Vector m_NextSamplingCOMRefPos;

  /*! \brief Absolute position of the reference dCoM. */
  dynamicgraph::Vector m_dCOMRefPos;
  dynamicgraph::Vector m_PrevSamplingdCOMRefPos;
  dynamicgraph::Vector m_NextSamplingdCOMRefPos;

  /*! \brief Absolute position of the reference ddCoM. */
  dynamicgraph::Vector m_ddCOMRefPos;
  dynamicgraph::Vector m_PrevSamplingddCOMRefPos;
  dynamicgraph::Vector m_NextSamplingddCOMRefPos;

  /*! \brief Initial Absolute position of the reference ZMP. */
  dynamicgraph::Vector m_InitZMPRefPos;

  /*! \brief Initial Absolute position and
    attitude of the reference Waist. */
  dynamicgraph::Vector m_InitWaistRefPos, m_InitWaistRefAtt;

  /*! \brief Initial Absolute position of the reference CoM. */
  dynamicgraph::Vector m_InitCOMRefPos;

  /*! \brief Waist position */
  dynamicgraph::Vector m_WaistPosition;

  /*! \brief Waist position Absolute */
  dynamicgraph::Vector m_WaistPositionAbsolute;

  /*! \brief Waist Attitude */
  dynamicgraph::Vector m_WaistAttitude;

  /*! \brief Waist Attitude Absolute */
  dynamicgraph::Vector m_WaistAttitudeAbsolute;
  dynamicgraph::Vector m_PrevSamplingWaistAttAbs;
  dynamicgraph::Vector m_NextSamplingWaistAttAbs;

  /*! \brief Waist Attitude Homogeneous Matrix */
  MatrixHomogeneous m_WaistAttitudeMatrixAbsolute;

  /*! \brief Joint values for walking. */
  dynamicgraph::Vector m_JointErrorValuesForWalking;

  /*! \brief Velocity reference for Herdt's PG */
  dynamicgraph::Vector m_VelocityReference;

  /*! \brief trigger to start walking */
  bool m_trigger;

  /*! \brief true iff the pattern if dealing with data,
    false if pg is not
    * working anymore/yet. */
  unsigned int m_dataInProcess;

  /*! \brief Booleans used to indicate if feedback
    signals shoul be used or not */
  bool m_feedBackControl;

  /*! \brief Booleans used to indicate if force
    feedback signals shoul be used or not */
  bool m_forceFeedBack;

  /*! \brief Booleans used to indicate feet contacts */
  bool m_rightFootContact, m_leftFootContact;

  /*! @} */

  /*! Parsing a file of command by the walking
    pattern generator interface.
    \par[in] The command line (optional option)
    \par[in] */
  void ParseCmdFile(std::istringstream &cmdArg, std::ostream &os);

  /*! \brief Transfert from a current absolute foot position
    to a dot homogeneous matrix. */
  void FromAbsoluteFootPosToDotHomogeneous(
      pg::FootAbsolutePosition aFootPosition, MatrixHomogeneous &aFootMH,
      MatrixHomogeneous &adotFootMH);

  /*! \brief Transfert from a current absolute foot position
    to a homogeneous matrix. */
  void FromAbsoluteFootPosToHomogeneous(pg::FootAbsolutePosition aFootPosition,
                                        MatrixHomogeneous &aFootMH);

  /*! \brief Provide an homogeneous matrix
    from the current waist position and attitude*/
  void getAbsoluteWaistPosAttHomogeneousMatrix(MatrixHomogeneous &aWaistMH);

  void SubsamplingFootPos(pg::FootAbsolutePosition &PrevFootPosition,
                          pg::FootAbsolutePosition &NextFootPosition,
                          MatrixHomogeneous &FootPositionOut,
                          MatrixHomogeneous &dotFootPositionOut,
                          unsigned int &count);

  void SubsamplingVector(dynamicgraph::Vector &PrevPosition,
                         dynamicgraph::Vector &NextPosition,
                         dynamicgraph::Vector &PositionOut,
                         unsigned int &count);

  void CopyFootPosition(pg::FootAbsolutePosition &FootPositionIn,
                        pg::FootAbsolutePosition &FootPositionOut);

  /*! \brief Internal method to get the initial
    reference ZMP at a given time. */
  dynamicgraph::Vector &getInitZMPRef(dynamicgraph::Vector &res, int time);

  /*! \brief Internal method to get the
    initial reference CoM at a given time.*/
  dynamicgraph::Vector &getInitCoMRef(dynamicgraph::Vector &res, int time);

  /*! \brief Internal method to get the initial
    reference CoM at a given time.*/
  dynamicgraph::Vector &getInitWaistPosRef(dynamicgraph::Vector &res, int time);

  /*! \brief Internal method to get the initial
    reference CoM at a given time.*/
  VectorRollPitchYaw &getInitWaistAttRef(VectorRollPitchYaw &res, int time);

  /*! \brief Internal method to get the position of the left foot. */
  MatrixHomogeneous &getInitLeftFootRef(MatrixHomogeneous &res, int time);

  /*! \brief Internal method to get the position of the right foot. */
  MatrixHomogeneous &getInitRightFootRef(MatrixHomogeneous &res, int time);

  /*! \brief Internal method to get the information of contact or not on
    the feet. */
  bool &getLeftFootContact(bool &res, int time);
  bool &getRightFootContact(bool &res, int time);

 public:
  /*! \name External signals
    @{ */
  /*! \brief Real state position values. */
  SignalPtr<dynamicgraph::Vector, int> jointPositionSIN;

  /*! \brief Motor control joint position values. */
  SignalPtr<dynamicgraph::Vector, int> motorControlJointPositionSIN;

  /*! \brief Previous ZMP value (ZMP send by the preceding controller). */
  SignalPtr<dynamicgraph::Vector, int> ZMPPreviousControllerSIN;

  /*! \brief Externalize the ZMP reference . */
  SignalTimeDependent<dynamicgraph::Vector, int> ZMPRefSOUT;

  /*! \brief Externalize the CoM reference. */
  SignalTimeDependent<dynamicgraph::Vector, int> CoMRefSOUT;

  /*! \brief Externalize the CoM reference. */
  SignalTimeDependent<dynamicgraph::Vector, int> dCoMRefSOUT;

  SignalTimeDependent<dynamicgraph::Vector, int> ddCoMRefSOUT;
  /*! \brief Take the current CoM. */
  SignalPtr<dynamicgraph::Vector, int> comSIN;

  /*! \brief Take the current CoM state (pos, vel, acc). */
  SignalPtr<dynamicgraph::Vector, int> comStateSIN;

  /*! \brief Take the current zmp (x, y, z). */
  SignalPtr<dynamicgraph::Vector, int> zmpSIN;

  /*! \brief Take the current external force applied
    to the com (fx, fy, fz). */
  SignalPtr<dynamicgraph::Vector, int> forceSIN;
  SignalTimeDependent<dynamicgraph::Vector, int> forceSOUT;
  dynamicgraph::Vector m_initForce;
  dynamicgraph::Vector m_currentForces;
  std::deque<dynamicgraph::Vector> m_bufferForce;
  std::vector<double> m_filterWindow;

  /*! \brief Take the current desired velocity. */
  SignalPtr<dynamicgraph::Vector, int> velocitydesSIN;

  /*! \brief Take the current trigger to start OneStepOfControl. */
  SignalPtr<bool, int> triggerSIN;

  /*! \brief Take the current left foot homogeneous position. */
  SignalPtr<MatrixHomogeneous, int> LeftFootCurrentPosSIN;

  /*! \brief Take the current right foot homogeneous position. */
  SignalPtr<MatrixHomogeneous, int> RightFootCurrentPosSIN;

  /*! \brief Externalize the left foot position reference. */
  SignalTimeDependent<MatrixHomogeneous, int> LeftFootRefSOUT;

  /*! \brief Externalize the right foot position reference. */
  SignalTimeDependent<MatrixHomogeneous, int> RightFootRefSOUT;

  /*! \brief Externalize the left foot position reference. */
  SignalTimeDependent<MatrixHomogeneous, int> dotLeftFootRefSOUT;

  /*! \brief Externalize the right foot position reference. */
  SignalTimeDependent<MatrixHomogeneous, int> dotRightFootRefSOUT;

  /*! \brief Externalize the foot which is not considered as support foot,
    the information are given in a relative referentiel. */
  SignalTimeDependent<MatrixHomogeneous, int> FlyingFootRefSOUT;

  /*! \brief Externalize the support foot. */
  SignalTimeDependent<unsigned int, int> SupportFootSOUT;

  /*! \brief Externalize the joint values for walking. */
  SignalTimeDependent<dynamicgraph::Vector, int> jointWalkingErrorPositionSOUT;

  /*! \brief Externalize the com attitude. */
  SignalTimeDependent<dynamicgraph::Vector, int> comattitudeSOUT;

  /*! \brief Externalize the dcom attitude. */
  SignalTimeDependent<dynamicgraph::Vector, int> dcomattitudeSOUT;

  /*! \brief Externalize the ddcom attitude. */
  SignalTimeDependent<dynamicgraph::Vector, int> ddcomattitudeSOUT;

  /*! \brief Externalize the waist attitude. */
  SignalTimeDependent<VectorRollPitchYaw, int> waistattitudeSOUT;

  /*! \brief Externalize the absolute waist attitude. */
  SignalTimeDependent<VectorRollPitchYaw, int> waistattitudeabsoluteSOUT;

  /*! \brief Externalize the absolute waist attitude into a homogeneous matrix.
   */
  SignalTimeDependent<MatrixHomogeneous, int> waistattitudematrixabsoluteSOUT;

  /*! \brief Externalize the waist position. */
  SignalTimeDependent<dynamicgraph::Vector, int> waistpositionSOUT;

  /*! \brief Externalize the absolute waist position. */
  SignalTimeDependent<dynamicgraph::Vector, int> waistpositionabsoluteSOUT;

  /*! \brief true iff PG is processing. Use it for synchronize. */
  SignalTimeDependent<unsigned int, int> dataInProcessSOUT;

  /*! \brief Externalize the initial ZMP reference . */
  SignalTimeDependent<dynamicgraph::Vector, int> InitZMPRefSOUT;

  /*! \brief Externalize the initial CoM reference. */
  SignalTimeDependent<dynamicgraph::Vector, int> InitCoMRefSOUT;

  /*! \brief Externalize the initial Waist reference. */
  SignalTimeDependent<dynamicgraph::Vector, int> InitWaistPosRefSOUT;

  /*! \brief Externalize the initial Waist reference. */
  SignalTimeDependent<VectorRollPitchYaw, int> InitWaistAttRefSOUT;

  /*! \brief Externalize the left foot position reference. */
  SignalTimeDependent<MatrixHomogeneous, int> InitLeftFootRefSOUT;

  /*! \brief Externalize the right foot position reference. */
  SignalTimeDependent<MatrixHomogeneous, int> InitRightFootRefSOUT;

  /*! \brief Booleans for contact of the feet */
  SignalTimeDependent<bool, int> leftFootContactSOUT;
  SignalTimeDependent<bool, int> rightFootContactSOUT;
  /*! @} */

  /*! \name Reimplement the interface of the plugin.
    @{ */

  /*! @} */
 protected:
  /*! Storing the previous ZMP value. */
  Eigen::VectorXd m_ZMPPrevious;

 public: /* --- PARAMS --- */
  void initCommands(void);
  int stringToReferenceEnum(const std::string &FrameReference);
  void setReferenceFromString(const std::string &str);
  void addOnLineStep(const double &x, const double &y, const double &th);
  void addStep(const double &x, const double &y, const double &th);
  void pgCommandLine(const std::string &cmdline);
  void useFeedBackSignals(const bool &feedBack);
  void useDynamicFilter(const bool &dynamicFilter);

  void debug(void);
};

}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __SOT_PATTERN_GENERATOR_H__
