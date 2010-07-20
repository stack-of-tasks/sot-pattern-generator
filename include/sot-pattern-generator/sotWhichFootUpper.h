/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotWhichFootUpper.h
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



#ifndef __SOT_SOTWHICHFOOTUPPER_H__
#define __SOT_SOTWHICHFOOTUPPER_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot-core/matrix-homogeneous.h>
#include <sot-core/vector-roll-pitch-yaw.h>
#include <sot-core/matrix-rotation.h>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (sotWhichFootUpper_EXPORTS) 
#    define SOTWHICHFOOTUPPER_EXPORT __declspec(dllexport)
#  else  
#    define SOTWHICHFOOTUPPER_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTWHICHFOOTUPPER_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTWHICHFOOTUPPER_EXPORT sotWhichFootUpper
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;

 protected:

  static const unsigned int INDEX_LEFT_FOOT_DEFAULT;
  static const unsigned int INDEX_RIGHT_FOOT_DEFAULT;
  unsigned int indexLeftFoot, indexRightFoot;

  static const double TRIGGER_THRESHOLD_DEFAULT;
  double triggerThreshold;

  unsigned int lastFoot;
  
 public: /* --- CONSTRUCTION --- */

  sotWhichFootUpper( const std::string& name );
  virtual ~sotWhichFootUpper( void );

 public: /* --- SIGNAL --- */

  dg::SignalPtr<MatrixRotation,int> waistRsensorSIN; 
  dg::SignalPtr<MatrixRotation,int> worldRsensorSIN; 
  dg::SignalPtr<MatrixHomogeneous,int> waistMlfootSIN; 
  dg::SignalPtr<MatrixHomogeneous,int> waistMrfootSIN; 
  
  dg::SignalTimeDependent<MatrixHomogeneous,int> worldMlfootSOUT;
  dg::SignalTimeDependent<MatrixHomogeneous,int> worldMrfootSOUT;
  dg::SignalTimeDependent<unsigned int,int> whichFootSOUT;

  dg::SignalPtr<MatrixHomogeneous,int> waistMsensorSIN; 
  dg::SignalTimeDependent<MatrixRotation,int> waistRsensorSOUT; 

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





#endif // #ifndef __SOT_SOTWHICHFOOTUPPER_H__

