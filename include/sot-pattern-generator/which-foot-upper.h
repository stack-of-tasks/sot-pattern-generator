/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      WhichFootUpper.h
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



#ifndef __SOT_WhichFootUpper_H__
#define __SOT_WhichFootUpper_H__

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

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (which_foot_upper_EXPORTS)
#    define WhichFootUpper_EXPORT __declspec(dllexport)
#  else
#    define WhichFootUpper_EXPORT __declspec(dllimport)
#  endif
#else
#  define WhichFootUpper_EXPORT
#endif


namespace dynamicgraph {
  namespace sot {

    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */

    class WhichFootUpper_EXPORT WhichFootUpper
      :public Entity
    {
      DYNAMIC_GRAPH_ENTITY_DECL ();

    protected:

      static const unsigned int INDEX_LEFT_FOOT_DEFAULT;
      static const unsigned int INDEX_RIGHT_FOOT_DEFAULT;
      unsigned int indexLeftFoot, indexRightFoot;

      static const double TRIGGER_THRESHOLD_DEFAULT;
      double triggerThreshold;

      unsigned int lastFoot;

    public: /* --- CONSTRUCTION --- */

      WhichFootUpper( const std::string& name );
      virtual ~WhichFootUpper( void );

    public: /* --- SIGNAL --- */

      SignalPtr<MatrixRotation,int> waistRsensorSIN;
      SignalPtr<MatrixRotation,int> worldRsensorSIN;
      SignalPtr<MatrixHomogeneous,int> waistMlfootSIN;
      SignalPtr<MatrixHomogeneous,int> waistMrfootSIN;

      SignalTimeDependent<MatrixHomogeneous,int> worldMlfootSOUT;
      SignalTimeDependent<MatrixHomogeneous,int> worldMrfootSOUT;
      SignalTimeDependent<unsigned int,int> whichFootSOUT;

      SignalPtr<MatrixHomogeneous,int> waistMsensorSIN;
      SignalTimeDependent<MatrixRotation,int> waistRsensorSOUT;

    public: /* --- FUNCTIONS --- */

      static MatrixHomogeneous &
	computeFootPosition( const MatrixHomogeneous& waistMfoot,
			     const MatrixRotation& waistRsensor,
			     const MatrixRotation& worldRsensor,
			     MatrixHomogeneous& res );

      unsigned int & whichFoot( const MatrixHomogeneous& waistMlfoot,
				const MatrixHomogeneous& waistMrfoot,
				unsigned int& res );


    public: /* --- PARAMS --- */
      virtual void commandLine( const std::string& cmdLine,
				std::istringstream& cmdArgs,
				std::ostream& os );


    };

  } // namespace sot
} // namespace dynamicgraph

#endif // #ifndef __SOT_WhichFootUpper_H__
