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

/* SOT */

#include <sot/core/flags.hh>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot-pattern-generator/exception-pg.h>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>
#include <sot/core/matrix-rotation.hh>

/* Pattern Generator */
#include <jrl/mal/matrixabstractlayer.hh>
#include <jrl/walkgen/patterngeneratorinterface.hh>
namespace pg=PatternGeneratorJRL;

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (pg_EXPORTS)
#    define PatternGenerator_EXPORT __declspec(dllexport)
#  else
#    define PatternGenerator_EXPORT __declspec(dllimport)
#  endif
#else
#  define PatternGenerator_EXPORT
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
    class PatternGenerator_EXPORT PatternGenerator
      :public Entity
    {
    public:

      /*! \name Some constant to define the type
	of frame reference.
	@{
      */
      /*! \brief Specify that the frame is expressed  in
	the world reference frame. */
      static const int WORLD_FRAME=0;

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

      /*! \brief Pointer towards the interface of the pattern generator. */
      pg::PatternGeneratorInterface * m_PGI;

      /*! \name Fields to store name and positions of data files
	@{
      */
      /*! \brief Some information related to the preview control. */
      std::string m_PreviewControlParametersFile;

      /*! \brief Directory where the VRML file of the robot's model is located. */
      std::string m_vrmlDirectory;

      /*! \brief Name of the VRML file which containes the robot's model. */
      std::string m_vrmlMainFile;

      /*! \brief Name of the XML file which contains humanoid specific informations. */
      std::string m_xmlSpecificitiesFile;

      /*! \brief Name of the XML file which specificies the rank of the Joints in the state vector. */
      std::string m_xmlRankFile;

      /*! \brief Directory where the VRML file of the robot's model is located. */
      std::string m_urdfDirectory;

      /*! \brief Name of the VRML file which containes the robot's model. */
      std::string m_urdfMainFile; 

      /*! \brief Lenght of the sole */
      double m_soleLength;

      /*! \brief Width of the sole */
      double m_soleWidth;

      /* \brief Special joints map for the parser */
      std::map<std::string, std::string> specialJoints_;     

     /*! @} */

      /*! \brief Boolean variable to initialize the object by loading an object. */
      bool m_init;

      /*! \brief Boolean variable to initialize the position:
	first through the real state of the robot, then through the motor command. */
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

    public: /* --- CONSTRUCTION --- */

      /*! \brief Default constructor. */
      PatternGenerator( const std::string& name = "PatternGenerator" );
      /*! \brief Default destructor. */
      virtual ~PatternGenerator( void );

    public: /* --- MODEL CREATION --- */

      /*! \name Methods related to the data files.
	@{
      */

      /*! \brief Build the pattern generator interface. */
      bool buildModel( void );

     /*! \brief Build the pattern generator interface from a Urdf file. */
      bool buildModelUrdf( void );

      /*! \brief Initialize the state of the robot. */
      bool InitState( void );

      /*! \brief Set the directory which contains the parameters for the preview control. */
      void setPreviewControlParametersFile( const std::string& filename );

      /*! \brief Set the directory which contains the VRML files robot's model. */
      void setVrmlDirectory( const std::string& filename );

      /*! \brief Set the name of the VRML file. */
      void setVrmlMainFile( const std::string& filename );

      /*! \brief Set the name of the file specifying a semantic to the joints.
	More precisely this file describes which joints are the hands, feet.
	For more information please see the documentation of walkGenJRL. */
      void setXmlSpecificitiesFile( const std::string& filename );

      /*! \brief Set the name of the file specifying the rank of the joints in the state vector. */
      void setXmlRankFile( const std::string& filename );

      /*! \brief Set the name of the file specifying the control parameters
	of the preview control. */
      void setParamPreviewFile(const std::string &filename);

      /*! \brief Set the directory which contains the urdf files robot's model. */
      void setUrdfDirectory( const std::string& filename );
      
      /*! \brief Set the name of the urdf file. */
      void setUrdfMainFile( const std::string& filename );

      /*! \brief Set the foot parameters */
      void setSoleParameters(const double& inSoleLength, const double& inSoleWidth);
 
      /*! \brief Set mapping between a link and actual robot name */
       void addJointMapping(const std::string& link, const std::string& repName);
     
      /*! \brief Give access directly to the pattern generator...
	You really have to know what your are doing. */
      pg::PatternGeneratorInterface * GetPatternGeneratorInterface()
	{ return m_PGI;};

      /*! @} */

    public: /* --- SIGNALS --- */

      typedef int Dummy;

      /*! \name Internal signals.
	@{
      */

      /*! \brief Internal signal for initialization and one shot signals. */
      SignalTimeDependent<Dummy,int> firstSINTERN;

      /*! \brief Internal signal to trigger one step of the algorithm. */
      SignalTimeDependent<Dummy,int> OneStepOfControlS;

      /*! @} */

    protected:

      /*! \name Internal methods to access reference trajectories.
	@{
      */
      /*! \brief Internal method to get the reference ZMP at a given time. */
      ml::Vector & getZMPRef(ml::Vector & res, int time);

      /*! \brief Internal method to get the reference CoM at a given time.*/
      ml::Vector & getCoMRef(ml::Vector & res, int time);

      /*! \brief Internal method to get the reference dCoM at a given time.*/
      ml::Vector & getdCoMRef(ml::Vector & res, int time);

      /*! \brief Internal method to get the position of the left foot. */
      MatrixHomogeneous & getLeftFootRef(MatrixHomogeneous &res, int time);

      /*! \brief Internal method to get the position of the right foot. */
      MatrixHomogeneous & getRightFootRef(MatrixHomogeneous &res, int time);

      /*! \brief Internal method to get the derivative of the left foot. */
      MatrixHomogeneous & getdotLeftFootRef(MatrixHomogeneous &res, int time);

      /*! \brief Internal method to get the derivative of the right foot. */
      MatrixHomogeneous & getdotRightFootRef(MatrixHomogeneous &res, int time);

      /*! \brief Internal method to get the position of the flying foot. */
      MatrixHomogeneous & getFlyingFootRef(MatrixHomogeneous &res, int time);

      /*! \brief Internal method to get the joint position for walking. */
      ml::Vector & getjointWalkingErrorPosition(ml::Vector &res, int time);

      /*! \brief Internal method to get the derivative of the com attitude. */
      VectorRollPitchYaw & getdComAttitude(VectorRollPitchYaw &res, int time);

      /*! \brief Internal method to get the attitude of the com. */
      VectorRollPitchYaw & getComAttitude(VectorRollPitchYaw &res, int time);

      /*! \brief Internal method to get the attitude of the waist. */
      VectorRollPitchYaw & getWaistAttitude(VectorRollPitchYaw &res, int time);

      /*! \brief Internal method to get the absolute attitude of the waist. */
      VectorRollPitchYaw & getWaistAttitudeAbsolute(VectorRollPitchYaw &res, int time);

      /*! \brief Internal method to get the dataInPorcess flag */
      unsigned & getDataInProcess(unsigned &res, int time);

      /*! \brief Internal method to get the position of the waist. */
      ml::Vector & getWaistPosition(ml::Vector &res, int time);

      /*! \brief Internal method to get the absolute position of the waist. */
      ml::Vector & getWaistPositionAbsolute(ml::Vector &res, int time);

      /*! @} */

      /*! \brief Getting the current support foot: 1 Left -1 Right. */
      unsigned int & getSupportFoot(unsigned int &res, int time);

      /*! \brief Trigger the initialization of the algorithm */
      int & InitOneStepOfControl(int &dummy, int time);
      /*! \brief Trigger one step of the algorithm. */
      int & OneStepOfControl(int &dummy, int time);

      /*! \name Keep information computed once for each time.
	@{
      */

      /*! \brief Rigit motion between two waist positions
	at the  beginning of the walking and at the end of the walking. */
      MatrixHomogeneous m_k_Waist_kp1;

      /*! \brief Absolute Position for the left and right feet. */
      MatrixHomogeneous m_LeftFootPosition,m_RightFootPosition;

      /*! \brief Absolute Derivate for the left and right feet. */
      MatrixHomogeneous m_dotLeftFootPosition,m_dotRightFootPosition;

      /*! \brief Initial Absolute Starting Position for the left and right feet. */
      MatrixHomogeneous m_InitLeftFootPosition,m_InitRightFootPosition;

      /*! \brief Keep track of the motion between sequence of motions. */
      MatrixHomogeneous m_MotionSinceInstanciationToThisSequence;

      /*! \brief Relative Position of the flying foot. */
      MatrixHomogeneous m_FlyingFootPosition;

      /*! \brief Absolute position of the reference ZMP. */
      ml::Vector m_ZMPRefPos;

      /*! \brief Com Attitude: does not really exist apart from when the robot
	is seen as an inverted pendulum*/
      ml::Vector m_ComAttitude;

      /*! \brief Com Attitude: does not really exist apart when the robot
	is seen as an inverted pendulum*/
      ml::Vector m_dComAttitude;

      /*! \brief Absolute position of the reference CoM. */
      ml::Vector m_COMRefPos;

      /*! \brief Absolute position of the reference dCoM. */
      ml::Vector m_dCOMRefPos;

      /*! \brief Initial Absolute position of the reference ZMP. */
      ml::Vector m_InitZMPRefPos;

      /*! \brief Initial Absolute position and attitude of the reference Waist. */
      ml::Vector m_InitWaistRefPos, m_InitWaistRefAtt;

      /*! \brief Initial Absolute position of the reference CoM. */
      ml::Vector m_InitCOMRefPos;

      /*! \brief Waist position */
      ml::Vector m_WaistPosition;

      /*! \brief Waist position Absolute */
      ml::Vector m_WaistPositionAbsolute;

      /*! \brief Waist Attitude */
      ml::Vector m_WaistAttitude;

      /*! \brief Waist Attitude Absolute */
      ml::Vector m_WaistAttitudeAbsolute;

      /*! \brief Joint values for walking. */
      ml::Vector m_JointErrorValuesForWalking;

      /*! \brief Velocity reference for Herdt's PG */
      ml::Vector m_VelocityReference;
      /*! \brief true iff the pattern if dealing with data, false if pg is not
       * working anymore/yet. */
      unsigned int m_dataInProcess;

      /*! \brief Booleans used to indicate feet contacts */
      bool m_rightFootContact, m_leftFootContact;

      /*! @} */

      /*! Parsing a file of command by the walking pattern generator interface.
	\par[in] The command line (optional option)
	\par[in]

      */
      void ParseCmdFile(std::istringstream &cmdArg,
			std::ostream &os);

      /*! \brief Transfert from a current absolute foot position
	to a dot homogeneous matrix. */
      void FromAbsoluteFootPosToDotHomogeneous(pg::FootAbsolutePosition aFootPosition,
					       MatrixHomogeneous &aFootMH,
					       MatrixHomogeneous &adotFootMH);


      /*! \brief Transfert from a current absolute foot position
	to a homogeneous matrix. */
      void FromAbsoluteFootPosToHomogeneous(pg::FootAbsolutePosition aFootPosition,
					    MatrixHomogeneous &aFootMH);


      /*! \brief Provide an homogeneous matrix from the current waist position
	and attitude*/
      void getAbsoluteWaistPosAttHomogeneousMatrix(MatrixHomogeneous &aWaistMH);


      /*! \brief Internal method to get the initial reference ZMP at a given time. */
      ml::Vector & getInitZMPRef(ml::Vector & res, int time);

      /*! \brief Internal method to get the initial reference CoM at a given time.*/
      ml::Vector & getInitCoMRef(ml::Vector & res, int time);

      /*! \brief Internal method to get the initial reference CoM at a given time.*/
      ml::Vector & getInitWaistPosRef(ml::Vector & res, int time);

      /*! \brief Internal method to get the initial reference CoM at a given time.*/
      VectorRollPitchYaw & getInitWaistAttRef(VectorRollPitchYaw & res, int time);

      /*! \brief Internal method to get the position of the left foot. */
      MatrixHomogeneous & getInitLeftFootRef(MatrixHomogeneous &res, int time);

      /*! \brief Internal method to get the position of the right foot. */
      MatrixHomogeneous & getInitRightFootRef(MatrixHomogeneous &res, int time);

      /*! \brief Internal method to get the information of contact or not on
	the feet. */
      bool & getLeftFootContact(bool & res,int time);
      bool & getRightFootContact(bool & res,int time);
      
    public:

      /*! \name External signals
	@{
      */
      /*! \brief Real state position values. */
      SignalPtr<ml::Vector,int> jointPositionSIN;

      /*! \brief Motor control joint position values. */
      SignalPtr<ml::Vector,int> motorControlJointPositionSIN;

      /*! \brief Previous ZMP value (ZMP send by the preceding controller). */
      SignalPtr<ml::Vector,int> ZMPPreviousControllerSIN;

      /*! \brief Externalize the ZMP reference . */
      SignalTimeDependent<ml::Vector,int> ZMPRefSOUT;

      /*! \brief Externalize the CoM reference. */
      SignalTimeDependent<ml::Vector,int> CoMRefSOUT;

      /*! \brief Externalize the CoM reference. */
      SignalTimeDependent<ml::Vector,int> dCoMRefSOUT;

      /*! \brief Take the current CoM. */
      SignalPtr<ml::Vector,int> comSIN;

      /*! \brief Take the current desired velocity. */
      SignalPtr<ml::Vector,int> velocitydesSIN;

      /*! \brief Take the current left foot homogeneous position. */
      SignalPtr<MatrixHomogeneous,int> LeftFootCurrentPosSIN;

      /*! \brief Take the current right foot homogeneous position. */
      SignalPtr<MatrixHomogeneous,int> RightFootCurrentPosSIN;

      /*! \brief Externalize the left foot position reference. */
      SignalTimeDependent<MatrixHomogeneous,int> LeftFootRefSOUT;

      /*! \brief Externalize the right foot position reference. */
      SignalTimeDependent<MatrixHomogeneous,int> RightFootRefSOUT;

      /*! \brief Externalize the left foot position reference. */
      SignalTimeDependent<MatrixHomogeneous,int> dotLeftFootRefSOUT;

      /*! \brief Externalize the right foot position reference. */
      SignalTimeDependent<MatrixHomogeneous,int> dotRightFootRefSOUT;

      /*! \brief Externalize the foot which is not considered as support foot,
	the information are given in a relative referentiel. */
      SignalTimeDependent<MatrixHomogeneous,int> FlyingFootRefSOUT;

      /*! \brief Externalize the support foot. */
      SignalTimeDependent<unsigned int,int> SupportFootSOUT;

      /*! \brief Externalize the joint values for walking. */
      SignalTimeDependent<ml::Vector,int> jointWalkingErrorPositionSOUT;

      /*! \brief Externalize the com attitude. */
      SignalTimeDependent<VectorRollPitchYaw,int> comattitudeSOUT;

      /*! \brief Externalize the dcom attitude. */
      SignalTimeDependent<VectorRollPitchYaw,int> dcomattitudeSOUT;

      /*! \brief Externalize the waist attitude. */
      SignalTimeDependent<VectorRollPitchYaw,int> waistattitudeSOUT;

      /*! \brief Externalize the absolute waist attitude. */
      SignalTimeDependent<VectorRollPitchYaw,int> waistattitudeabsoluteSOUT;

      /*! \brief Externalize the waist position. */
      SignalTimeDependent<ml::Vector,int> waistpositionSOUT;

      /*! \brief Externalize the absolute waist position. */
      SignalTimeDependent<ml::Vector,int> waistpositionabsoluteSOUT;

      /*! \brief true iff PG is processing. Use it for synchronize. */
      SignalTimeDependent<unsigned int,int> dataInProcessSOUT;

      /*! \brief Externalize the initial ZMP reference . */
      SignalTimeDependent<ml::Vector,int> InitZMPRefSOUT;

      /*! \brief Externalize the initial CoM reference. */
      SignalTimeDependent<ml::Vector,int> InitCoMRefSOUT;

      /*! \brief Externalize the initial Waist reference. */
      SignalTimeDependent<ml::Vector,int> InitWaistPosRefSOUT;

      /*! \brief Externalize the initial Waist reference. */
      SignalTimeDependent<VectorRollPitchYaw,int> InitWaistAttRefSOUT;

      /*! \brief Externalize the left foot position reference. */
      SignalTimeDependent<MatrixHomogeneous,int> InitLeftFootRefSOUT;

      /*! \brief Externalize the right foot position reference. */
      SignalTimeDependent<MatrixHomogeneous,int> InitRightFootRefSOUT;

      /*! \brief Booleans for contact of the feet */
      SignalTimeDependent<bool,int> leftFootContactSOUT;
      SignalTimeDependent<bool,int> rightFootContactSOUT;
      /*! @} */

      /*! \name Reimplement the interface of the plugin.
	@{
      */

      /*! @} */
    protected:

      /*! Storing the previous ZMP value. */
      MAL_VECTOR(m_ZMPPrevious,double);

    public: /* --- PARAMS --- */
      void initCommands( void );
      /*! \brief This method pass on to the Pattern Generator Interface to interpret the commands.
       */
      virtual void commandLine( const std::string& cmdLine,
				std::istringstream& cmdArgs,
				std::ostream& os );
      int stringToReferenceEnum( const std::string & FrameReference );
      void setReferenceFromString( const std::string & str );
      void addOnLineStep( const double & x, const double & y, const double & th);
      void addStep( const double & x, const double & y, const double & th);
      void pgCommandLine( const std::string & cmdline );

      void debug(void);
    };



  } // namespace sot
} // namespace dynamicgraph

#endif // #ifndef __SOT_PATTERN_GENERATOR_H__

