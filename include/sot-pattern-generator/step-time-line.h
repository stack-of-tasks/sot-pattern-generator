/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      StepTimeLine.h
 * Project:   SOT
 * Author:    Nicolas Mansard, Olivier Stasse, Paul Evrard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 * StepTimeLine entity: synchronizes a StepQueue, a StepComputer and a
 * PGManager to compute steps to send to the PG. Uses a StepChecker
 * to clip the steps.
 * Highest-level class for automatic step generation.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
 
#ifndef __SOT_STEP_TIME_LINE_H__
#define __SOT_STEP_TIME_LINE_H__


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot-pattern-generator/step-queue.h>
#include <sot-pattern-generator/step-computer.h>
#include <sot-pattern-generator/pg-manager.h>


/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (step_time_line_EXPORTS)
#    define StepTimeLine_EXPORT __declspec(dllexport)
#  else  
#    define StepTimeLine_EXPORT __declspec(dllimport)
#  endif 
#else
#  define StepTimeLine_EXPORT
#endif

namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/// Synchronizes the components for the automatic step generation.
///
/// This entity automatically generates steps to send to a PatternGenerator
/// entity. The responsibility of this entity is to synchronize all the
/// components that are used to generate and send the steps, by triggering
/// them at the appropriate time.
///
/// This entity accepts the following shell commands:
/// - state: stepper.state will print the current state of the timeline,
///          stepper.state start will start the stepping,
///          stepper.state stop will stop the stepping.
///
/// The possible states of the timeline are the following:
/// - starting: playing an initial step sequence (warmup),
/// - started: generating steps using the StepComputer,
/// - stopping: playing a final step sequence (cleanup),
/// - stopped: doing nothing (not sending steps).
///
/// \note{This entity class can not be instantiated in a shell since it does not
/// register any factory. This behavior is intended.}
class StepTimeLine_EXPORT StepTimeLine
  : public dg::Entity
{
public: // dg::Entity name

  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

private:

  static const unsigned int PERIOD_DEFAULT;

  /// Zero-based index of the first step to modify. The 0-th step
  /// is the very first step sent at the STARTING state.
  static const unsigned int FIRST_STEP_TO_MODIFY;

public: // Construction

  StepTimeLine( const std::string& name );

public: // Trigger

  /// Trigger signal, to be updated periodically to trigger a
  /// call to the synchronization method. Typically, this signal
  /// is added to the periodic calls of OpenHRP.
  dg::Signal< int,int > triggerSOUT;

  /// The trigger callback function, which implements the synchronization
  /// of all the sub-components used to generate and send the steps.
  int& triggerCall( int& dummy, int timeCurr );

private: // State

  enum SteppingState
  {
    STATE_STARTING,   ///< Introducing 4 steps then switches to STATE_STARTED.
    STATE_STOPPING,   ///< Running but stop requested: introduce a last step and stop.
    STATE_STARTED,    ///< Running, simply introduce steps.
    STATE_STOPPED     ///< Nothing to do, cannot introduce steps in the FIFO
  };

public: // dg::Entity

  virtual void display( std::ostream& os ) const; 
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );

private:

  StepQueue* stepQueue;
  StepComputer* stepComputer;
  PGManager* pgManager;

  SteppingState state;
  int timeLastIntroduction;
  int period;
  unsigned int nStartingSteps;
};





} // namespace sot




#endif






