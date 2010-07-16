/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotStepComputer.h
 * Project:   SOT
 * Author:    Paul Evrard, Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 * Computes reference signals for the stepper.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#ifndef __SOT_STEP_OBSERVER_H__
#define __SOT_STEP_OBSERVER_H__


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot-core/matrix-homogeneous.h>


/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (sotStepObserver_EXPORTS) 
#    define SOTSTEPOBSERVER_EXPORT __declspec(dllexport)
#  else
#    define SOTSTEPOBSERVER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTSTEPOBSERVER_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/// Computes a reference frame from the position of both
/// hands and feet of the robot. The coordinates of the reference
/// frames are computed both in the left and right foot frames,
/// and in the waist frame.
class SOTSTEPOBSERVER_EXPORT sotStepObserver
  : public dg::Entity
{
 public:

  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public:

  dg::SignalPtr< MatrixHomogeneous,int > leftHandPositionSIN;
  dg::SignalPtr< MatrixHomogeneous,int > rightHandPositionSIN;

  dg::SignalPtr< MatrixHomogeneous,int > leftFootPositionSIN;
  dg::SignalPtr< MatrixHomogeneous,int > rightFootPositionSIN;
  dg::SignalPtr< MatrixHomogeneous,int > waistPositionSIN;

  /// Reference frame in left foot coordinates.
  dg::SignalTimeDependent< MatrixHomogeneous,int > referencePositionLeftSOUT;

  /// Reference frame in right foot coordinates.
  dg::SignalTimeDependent< MatrixHomogeneous,int > referencePositionRightSOUT;

  /// Reference frame in the waist coordinates.
  dg::SignalTimeDependent< MatrixHomogeneous,int > referencePositionWaistSOUT;

 public: // methods

  sotStepObserver( const std::string & name );

  dg::SignalArray<int> getdg::Signals( void );
  operator dg::SignalArray<int> ();

 public: // signal callbacks

  MatrixHomogeneous& computeReferencePositionLeft( MatrixHomogeneous& res,int timeCurr );
  MatrixHomogeneous& computeReferencePositionRight( MatrixHomogeneous& res,int timeCurr );
  MatrixHomogeneous& computeReferencePositionWaist( MatrixHomogeneous& res,int timeCurr );

 public: // dg::Entity

  virtual void display( std::ostream& os ) const; 
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );

 private: // helpers

  MatrixHomogeneous& computeRefPos( MatrixHomogeneous& res,int timeCurr,const MatrixHomogeneous& wMref );
};

#endif
