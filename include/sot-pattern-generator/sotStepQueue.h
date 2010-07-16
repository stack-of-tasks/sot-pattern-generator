/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotStepQueue.h
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
#include <string>
#include <deque>


/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (sotStepQueue_EXPORTS) 
#    define SOTSTEPQUEUE_EXPORT __declspec(dllexport)
#  else  
#    define SOTSTEPQUEUE_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTSTEPQUEUE_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/// Support foot identifier.
enum sotContactName
{
  CONTACT_LEFT_FOOT,
  CONTACT_RIGHT_FOOT
};


class SOTSTEPQUEUE_EXPORT sotFootPrint
{
public:

  sotFootPrint();
  sotFootPrint(double x, double y, double theta, sotContactName contact);

  double x,y,theta;           ///< The coordinates of the step (landing position of the fly foot).
  sotContactName contact;     ///< Fly foot.
};


/// A step queue in the preview window.

/// A series of step in the preview window. The first step can be modified.
///
/// \invariant{The queue always contains 4 steps.}
/// \note{This entity class can not be instantiated in a shell since it does not
/// register any factory. This behavior is intended.}
class SOTSTEPQUEUE_EXPORT sotStepQueue
  : public dg::Entity
{
public: // dg::Entity name

  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

private: // Parameters

  static const unsigned int QUEUE_SIZE;
  static const double ZERO_STEP_POSITION;
  static const sotFootPrint START_FOOT_PRINT;

public: // Construction

  /// Builds a queue containing a starting step and three steps in the preview.
  /// The steps correspond to on-place stepping: (0, +/- y, 0), where
  /// y == sotStepQueue::ZERO_STEP_POSITION
  sotStepQueue( const std::string& name );

public: // Queue manipulation

  /// Resets the queue to the initial condition (see the constructor,
  /// sotStepQueue::sotStepQueue).
  void startSequence();

  /// Adds a step at the end of the preview window. The step at the beginning
  /// of the preview window is removed. The firstStepChanged flag is reset to
  /// false.
  void pushStep( double x, double y, double theta );

  //@{
  /// Access to the step queue.
  /// \warning{No check is performed on the indices used in these accessors.}
  const sotFootPrint& getStep( unsigned int index ) const;
  const sotFootPrint& getFirstStep() const;
  const sotFootPrint& getLastStep() const;
  //@}

  /// Changes the first step.
  void changeFirstStep( double x, double y, double dtheta );

  const sotFootPrint& getFirstStepChange() const;

  /// Returns true if the first step has been changed since the last call to
  /// pushStep.
  bool isFirstStepChanged() const;

public: // Queue properties
 
  //@{
  /// Access to the step queue properties (constants).
  unsigned int size() const;
  const sotFootPrint& getStartFootPrint() const;
  double getZeroStepPosition() const;
  //@}

public: // dg::Entity

  virtual void display( std::ostream& os ) const; 
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );

private:

  std::deque< sotFootPrint > footPrintList;
  sotFootPrint firstStepChange;
  bool firstStepChanged;
};

#endif

















