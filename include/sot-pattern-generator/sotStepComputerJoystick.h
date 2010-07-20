/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotStepComputer.h
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

#ifndef __SOT_SOTSTEPCOMPUTER_JOYSTICK_H__
#define __SOT_SOTSTEPCOMPUTER_JOYSTICK_H__


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
#include <sot/sotStepObserver.h>
#include <sot-pattern-generator/step-checker.h>
#include <sot/sotStepComputer.h>

/* STD */
#include <string>
#include <deque>
#include <fstream>


/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (sotStepComputerJoystick_EXPORTS) 
#    define SOTSTEPCOMPUTERJOYSTICK_EXPORT __declspec(dllexport)
#  else  
#    define SOTSTEPCOMPUTERJOYSTICK_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTSTEPCOMPUTERJOYSTICK_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class StepQueue;

/// Generates footsteps.
class SOTSTEPCOMPUTERJOYSTICK_EXPORT sotStepComputerJoystick
: public dg::Entity, public sotStepComputer
{
 public:

  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public: // Construction

  sotStepComputerJoystick( const std::string& name );

 public: // Methods

  void changeFirstStep( StepQueue& queue, int timeCurr );
  void nextStep( StepQueue& queue, int timeCurr );

 public: // dg::Signals
  
  /*! \brief Entry of the joystick (x,y,theta)*/ 
  dg::SignalPtr< ml::Vector,int > joystickSIN;
  /*! \brief Getting the support foot */
  dg::SignalPtr< unsigned,int > contactFootSIN;
  /*! \brief Externalize the last step . */
  dg::SignalTimeDependent<ml::Vector,int> laststepSOUT;

 protected:
  ml::Vector& getlaststep(ml::Vector &res, int time);

 public: // dg::Entity
  
  virtual void display( std::ostream& os ) const; 
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );

 private: // Reference frame

  StepChecker checker;

  void thisIsZero();
  
 private: // Debug

  std::ofstream logChanges;
  std::ofstream logPreview;

  double m_laststep[3];
};


#endif // #ifndef __SOT_STEPCOMPUTER_H__

