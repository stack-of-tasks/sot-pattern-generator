/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      StepComputer.h
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
#include <sot/core/matrix-geometry.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(step_observer_EXPORTS)
#define StepObserver_EXPORT __declspec(dllexport)
#else
#define StepObserver_EXPORT __declspec(dllimport)
#endif
#else
#define StepObserver_EXPORT
#endif

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/// Computes a reference frame from the position of both
/// hands and feet of the robot. The coordinates of the reference
/// frames are computed both in the left and right foot frames,
/// and in the waist frame.
class StepObserver_EXPORT StepObserver : public Entity {
public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

public:
  SignalPtr<MatrixHomogeneous, int> leftHandPositionSIN;
  SignalPtr<MatrixHomogeneous, int> rightHandPositionSIN;

  SignalPtr<MatrixHomogeneous, int> leftFootPositionSIN;
  SignalPtr<MatrixHomogeneous, int> rightFootPositionSIN;
  SignalPtr<MatrixHomogeneous, int> waistPositionSIN;

  /// Reference frame in left foot coordinates.
  SignalTimeDependent<MatrixHomogeneous, int> referencePositionLeftSOUT;

  /// Reference frame in right foot coordinates.
  SignalTimeDependent<MatrixHomogeneous, int> referencePositionRightSOUT;

  /// Reference frame in the waist coordinates.
  SignalTimeDependent<MatrixHomogeneous, int> referencePositionWaistSOUT;

public: // methods
  StepObserver(const std::string &name);

  SignalArray<int> getSignals(void);
  operator SignalArray<int>();

public: // signal callbacks
  MatrixHomogeneous &computeReferencePositionLeft(MatrixHomogeneous &res,
                                                  int timeCurr);
  MatrixHomogeneous &computeReferencePositionRight(MatrixHomogeneous &res,
                                                   int timeCurr);
  MatrixHomogeneous &computeReferencePositionWaist(MatrixHomogeneous &res,
                                                   int timeCurr);

public: // Entity
  virtual void display(std::ostream &os) const;
  virtual void commandLine(const std::string &cmdLine,
                           std::istringstream &cmdArgs, std::ostream &os);

private: // helpers
  MatrixHomogeneous &computeRefPos(MatrixHomogeneous &res, int timeCurr,
                                   const MatrixHomogeneous &wMref);
};

} // namespace sot
} // namespace dynamicgraph

#endif
