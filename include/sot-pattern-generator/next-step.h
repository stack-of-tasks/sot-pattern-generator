/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      NextStep.h
 * Project:   SOT
 * Author:    Nicolas Mansard
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

#ifndef __SOT_SOTNEXTSTEP_H__
#define __SOT_SOTNEXTSTEP_H__


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>
#include <sot/core/matrix-rotation.hh>

/* STD */
#include <string>
#include <deque>


/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (next_step_EXPORTS)
#    define SOTNEXTSTEP_EXPORT __declspec(dllexport)
#  else
#    define SOTNEXTSTEP_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTNEXTSTEP_EXPORT
#endif

namespace dynamicgraph {
  namespace sot {
    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */


    /* --- Two Hand Observer ----------------------------------------------- */

    /*!
     * Computes a reference frame in either left or right foot
     * coordinates, based on the position of the hands. Used by the
     * stepper to compute the reference foot prints to send to
     * the Pattern Generator.
     */
    class SOTNEXTSTEP_EXPORT NextStepTwoHandObserver
    {
    public:

      SignalPtr< MatrixHomogeneous,int > referencePositionLeftSIN;
      SignalPtr< ml::Vector,int > referenceVelocityLeftSIN;
      SignalPtr< ml::Vector,int > referenceAccelerationLeftSIN;
      SignalPtr< MatrixHomogeneous,int > leftFootPositionSIN;

      SignalPtr< MatrixHomogeneous,int > referencePositionRightSIN;
      SignalPtr< ml::Vector,int > referenceVelocityRightSIN;
      SignalPtr< ml::Vector,int > referenceAccelerationRightSIN;
      SignalPtr< MatrixHomogeneous,int > rightFootPositionSIN;

      SignalTimeDependent< MatrixHomogeneous,int > referencePositionLeftSOUT;
      SignalTimeDependent< MatrixHomogeneous,int > referencePositionRightSOUT;
      SignalTimeDependent< ml::Vector,int > referenceVelocitySOUT;
      SignalTimeDependent< ml::Vector,int > referenceAccelerationSOUT;

    public:

      NextStepTwoHandObserver( const std::string & name );

      MatrixHomogeneous& computeReferencePositionLeft( MatrixHomogeneous& res,int timeCurr );
      MatrixHomogeneous& computeReferencePositionRight( MatrixHomogeneous& res,int timeCurr );
      ml::Vector& computeReferenceVelocity( const ml::Vector& right,const ml::Vector& left,ml::Vector& res );
      ml::Vector& computeReferenceAcceleration( const ml::Vector& right,const ml::Vector& left,ml::Vector& res );

      SignalArray<int> getSignals( void );
      operator SignalArray<int> ();

    private:

      MatrixHomogeneous& computeRefPos( MatrixHomogeneous& res,int timeCurr,const MatrixHomogeneous& wMsf );
    };


    /* --- Next Step (stepper) --------------------------------------------- */

    /*!
     * Generates footsteps depending on the frame computed by the
     * TwoHandObserver and the interpretation of the sensor forces.
     * A stack of 4 foot prints is maintained, and at each step, two
     * steps are computed: one step to push at the end of the stack,
     * and a step computed from the i-th step of the stack + delta,
     * which corresponds to a change in the future steps.
     */
    class SOTNEXTSTEP_EXPORT NextStep
      : public Entity
    {
    public:
      DYNAMIC_GRAPH_ENTITY_DECL();

    protected: /* --- FOOT PRINT LIST --- */

      enum ContactName
      {
	CONTACT_LEFT_FOOT
	,CONTACT_RIGHT_FOOT
      };
      class FootPrint
      {
      public:
	double x,y,theta;
	ContactName contact;
	int introductionTime;
      };
      std::deque< FootPrint > footPrintList;

    protected: /* --- INTRODUCTION PERIOD --- */

      unsigned int period;
      static const unsigned int PERIOD_DEFAULT;
      int timeLastIntroduction;

    protected: /* --- STATE --- */

      enum SteppingMode
      {
	MODE_1D
	,MODE_3D
      };
      SteppingMode mode;
      enum SteppingState
      {
	STATE_STARTING   //! Introducing 4 steps then switches to STATE_STARTED.
	,STATE_STOPING   //! Running but stop requested: introduce a last step and stop.
	,STATE_STARTED   //! Running, simply introduce steps.
	,STATE_STOPED    //! Nothing to do, cannot introduce steps in the FIFO
      };
      SteppingState state;

    protected: /* --- STEPPING --- */

      double zeroStepPosition;
      static const double ZERO_STEP_POSITION_DEFAULT; // = .19

    protected: /* --- REFERENCE FRAME --- */

      MatrixHomogeneous rfMref0;
      MatrixHomogeneous lfMref0;
      NextStepTwoHandObserver twoHandObserver;

      void thisIsZero();

    protected: /* --- DEBUG --- */

      std::ostream* verbose;


    public: /* --- CONSTRUCTION --- */
      NextStep( const std::string& name );
      virtual ~NextStep( void );

    public: /* --- Signal --- */

      SignalPtr< MatrixHomogeneous,int > referencePositionLeftSIN;
      SignalPtr< MatrixHomogeneous,int > referencePositionRightSIN;
      SignalPtr< unsigned,int > contactFootSIN;

      Signal< int,int > triggerSOUT;


    public: /* --- FUNCTIONS --- */

      virtual void nextStep( const int & timeCurr );
      virtual void starter( const int & timeCurr );
      virtual void stoper( const int & timeCurr );

      virtual void introductionCallBack( const int & timeCurr ) {};

      int& triggerCall( int& dummy,int timeCurr );

    public: /* --- PARAMS --- */

      virtual void display( std::ostream& os ) const;
      virtual void commandLine( const std::string& cmdLine,
				std::istringstream& cmdArgs,
				std::ostream& os );
    };


  } // namespace sot
} // namespace dynamicgraph

#endif // #ifndef __SOT_SOTNEXTSTEP_H__

