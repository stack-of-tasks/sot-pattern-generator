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
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#ifndef __SOT_StepComputer_FORCE_H__
#define __SOT_StepComputer_FORCE_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot-pattern-generator/step-checker.h>
#include <sot-pattern-generator/step-computer.h>
#include <sot-pattern-generator/step-observer.h>
#include <sot/core/matrix-geometry.hh>
/* STD */
#include <deque>
#include <fstream>
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(step_computer_pos_EXPORTS)
#define StepComputerFORCE_EXPORT __declspec(dllexport)
#else
#define StepComputerFORCE_EXPORT __declspec(dllimport)
#endif
#else
#define StepComputerFORCE_EXPORT
#endif

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class StepQueue;

/// Generates footsteps.
class StepComputerFORCE_EXPORT StepComputerPos : public Entity,
                                                 public StepComputer {
public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

public: // Construction
  StepComputerPos(const std::string &name);

public: // Methods
  void changeFirstStep(StepQueue &queue, int timeCurr);
  void nextStep(StepQueue &queue, int timeCurr);

public: // Signals
  SignalPtr<MatrixHomogeneous, int> referencePositionLeftSIN;
  SignalPtr<MatrixHomogeneous, int> referencePositionRightSIN;
  SignalPtr<unsigned, int> contactFootSIN;

public: // Entity
  virtual void display(std::ostream &os) const;
  virtual void commandLine(const std::string &cmdLine,
                           std::istringstream &cmdArgs, std::ostream &os);

private: // Reference frame
  MatrixHomogeneous rfMref0;
  MatrixHomogeneous lfMref0;
  StepObserver *twoHandObserver;
  StepChecker checker;

  void thisIsZero();

private: // Debug
  std::ofstream logChanges;
  std::ofstream logPreview;
};

} // namespace sot
} // namespace dynamicgraph

#endif // #ifndef __SOT_STEPCOMPUTER_H__
