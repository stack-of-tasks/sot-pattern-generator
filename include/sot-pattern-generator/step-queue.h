/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      StepQueue.h
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
 * StepQueue entity: manages a step queue (a series of future steps,
 * plus a series of changes in the future steps).
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#ifndef __SOT_STEPQUEUE_H__
#define __SOT_STEPQUEUE_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <dynamic-graph/entity.h>

/* STD */
#include <deque>
#include <string>

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(step_queue_EXPORTS)
#define StepQueue_EXPORT __declspec(dllexport)
#else
#define StepQueue_EXPORT __declspec(dllimport)
#endif
#else
#define StepQueue_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/// Support foot identifier.
enum ContactName { CONTACT_LEFT_FOOT, CONTACT_RIGHT_FOOT };

class StepQueue_EXPORT FootPrint {
public:
  FootPrint();
  FootPrint(double x, double y, double theta, ContactName contact);

  double x, y, theta;  ///< The coordinates of the step (landing position of the
                       ///< fly foot).
  ContactName contact; ///< Fly foot.
};

/// A step queue in the preview window.

/// A series of step in the preview window. The first step can be modified.
///
/// \invariant{The queue always contains 4 steps.}
/// \note{This entity class can not be instantiated in a shell since it does not
/// register any factory. This behavior is intended.}
class StepQueue_EXPORT StepQueue : public Entity {
public: // Entity name
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

private: // Parameters
  static const unsigned int QUEUE_SIZE;
  static const double ZERO_STEP_POSITION;
  static const FootPrint START_FOOT_PRINT;

public: // Construction
  /// Builds a queue containing a starting step and three steps in the preview.
  /// The steps correspond to on-place stepping: (0, +/- y, 0), where
  /// y == StepQueue::ZERO_STEP_POSITION
  StepQueue(const std::string &name);

public: // Queue manipulation
  /// Resets the queue to the initial condition (see the constructor,
  /// StepQueue::StepQueue).
  void startSequence();

  /// Adds a step at the end of the preview window. The step at the beginning
  /// of the preview window is removed. The firstStepChanged flag is reset to
  /// false.
  void pushStep(double x, double y, double theta);

  //@{
  /// Access to the step queue.
  /// \warning{No check is performed on the indices used in these accessors.}
  const FootPrint &getStep(unsigned int index) const;
  const FootPrint &getFirstStep() const;
  const FootPrint &getLastStep() const;
  //@}

  /// Changes the first step.
  void changeFirstStep(double x, double y, double dtheta);

  const FootPrint &getFirstStepChange() const;

  /// Returns true if the first step has been changed since the last call to
  /// pushStep.
  bool isFirstStepChanged() const;

public: // Queue properties
  //@{
  /// Access to the step queue properties (constants).
  unsigned int size() const;
  const FootPrint &getStartFootPrint() const;
  double getZeroStepPosition() const;
  //@}

public: // Entity
  virtual void display(std::ostream &os) const;
  virtual void commandLine(const std::string &cmdLine,
                           std::istringstream &cmdArgs, std::ostream &os);

private:
  std::deque<FootPrint> footPrintList;
  FootPrint firstStepChange;
  bool firstStepChanged;
};

} // namespace sot
} // namespace dynamicgraph

#endif
