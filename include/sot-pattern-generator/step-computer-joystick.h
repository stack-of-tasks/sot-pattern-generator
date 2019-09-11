/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      StepComputer.h
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

#ifndef __SOT_StepComputer_JOYSTICK_H__
#define __SOT_StepComputer_JOYSTICK_H__

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
#if defined(step_computer_joystick_EXPORTS)
#define StepComputerJOYSTICK_EXPORT __declspec(dllexport)
#else
#define StepComputerJOYSTICK_EXPORT __declspec(dllimport)
#endif
#else
#define StepComputerJOYSTICK_EXPORT
#endif

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class StepQueue;

/// Generates footsteps.
class StepComputerJOYSTICK_EXPORT StepComputerJoystick : public Entity,
                                                         public StepComputer {
public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

public: // Construction
  StepComputerJoystick(const std::string &name);

public: // Methods
  void changeFirstStep(StepQueue &queue, int timeCurr);
  void nextStep(StepQueue &queue, int timeCurr);

public: // Signals
  /*! \brief Entry of the joystick (x,y,theta)*/
  SignalPtr<Vector, int> joystickSIN;
  /*! \brief Getting the support foot */
  SignalPtr<unsigned, int> contactFootSIN;
  /*! \brief Externalize the last step . */
  SignalTimeDependent<Vector, int> laststepSOUT;

protected:
  Vector &getlaststep(Vector &res, int time);

public: // Entity
  virtual void display(std::ostream &os) const;
  virtual void commandLine(const std::string &cmdLine,
                           std::istringstream &cmdArgs, std::ostream &os);

private: // Reference frame
  StepChecker checker;

  void thisIsZero();

private: // Debug
  std::ofstream logChanges;
  std::ofstream logPreview;

  double m_laststep[3];
};

} // namespace sot
} // namespace dynamicgraph

#endif // #ifndef __SOT_STEPCOMPUTER_H__
