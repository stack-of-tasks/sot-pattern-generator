/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2009
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotStepComputerJoystick.cpp
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

#include <cmath>

#include <time.h>
#ifndef WIN32
# include <sys/time.h>
#else
# include <MatrixAbstractLayer/boost.h>
# include <sot-core/utils-windows.h>
# include <Winsock2.h>
#endif /*WIN32*/

#include <sot/sotStepComputerJoystick.h>
#include <sot-core/debug.h>
#include <sot/sotMacrosSignal.h>
#include <sot-pattern-generator/exception-pg.h>
#include <sot/StepQueue.h>
#include <sot/sotStepChecker.h>
#include <dynamic-graph/factory.h>


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(sotStepComputerJoystick,"StepComputerJoystick");


sotStepComputerJoystick::sotStepComputerJoystick( const std::string & name )
  : Entity(name)
  , joystickSIN( NULL,"sotStepComputerJoystick("+name+")::input(vector)::joystickin" )
  , contactFootSIN( NULL,"sotStepComputerJoystick("+name+")::input(uint)::contactfoot" )
  , laststepSOUT(boost::bind(&sotStepComputerJoystick::getlaststep,this,_1,_2),
		 joystickSIN,
		 "sotStepComputerJoystick("+name+")::output(vector)::laststep")
  , checker()
  , logChanges("/tmp/stepcomp_changes.dat")
  , logPreview("/tmp/stepcomp_preview.dat")
{
  sotDEBUGIN(5);

  signalRegistration(joystickSIN);
  signalRegistration(laststepSOUT << contactFootSIN);

  sotDEBUGOUT(5);
}

void sotStepComputerJoystick::nextStep( StepQueue& queue, int timeCurr )
{
  // Introduce new step at the end of the preview window.
  if( queue.getLastStep().contact == CONTACT_LEFT_FOOT ) {
    queue.pushStep(0., -queue.getZeroStepPosition(), 0.);
    logPreview << timeCurr << " " << 0 << " "
	       << -queue.getZeroStepPosition() << " " << 0
	       << std::endl;
  }
  else {
    queue.pushStep(0., queue.getZeroStepPosition(), 0.);
    logPreview << timeCurr << " " << 0 << " "
	       << queue.getZeroStepPosition() << " " << 0
	       << std::endl;
  }
}

void sotStepComputerJoystick::changeFirstStep( StepQueue& queue, int timeCurr )
{
  logChanges << timeCurr << " changeFirstStep" <<std::endl;

  const FootPrint& step = queue.getFirstStep();
  
  ml::Vector joyin;
  joyin.resize(3);
  try 
    {
      joyin= joystickSIN(timeCurr);
      
    }
  catch(...)
    {
      joyin(0) = 0;
      joyin(1) = 0;
      joyin(2) = 0;
      std::cerr  << "No joystick input stay on the spot" << std::endl;
    }
  
  double x = step.x+joyin(0), y = step.y+joyin(1);
  double theta = step.theta+joyin(2);

  std::cout << "stopStepComputedJoystick::changeFirstStep: " <<
    x << " " << y << " " << theta << std::endl;
  const double THETA_MAX = 9.;
  if(theta < -THETA_MAX){ theta = -THETA_MAX; }
  if(theta > THETA_MAX){ theta = THETA_MAX; }

  //double nx = 0, ny = 0;

  //checker.clipStep(x, y, nx, ny);

  // Log x-y values before and after clipping

  //  logChanges << timeCurr << " " << x << " " << y << " " << nx << " " << ny << " ";

  // The coordinates must be expressed in the destination foot frame.
  // See the technical report of Olivier Stasse for more details,
  // on top of page 79.

  double theta_rad = 3.14159265 * theta / 180.;
  double ctheta = cos(theta_rad);
  double stheta = sin(theta_rad);

  x = x * ctheta + y * stheta;
  y = -x * stheta + y * ctheta;

  queue.changeFirstStep(x, y, theta);

  // Log the step
  m_laststep[0] = x; m_laststep[1] = y; m_laststep[2] = theta;

  //  logChanges << x << " " << y << " " << theta << std::endl;
}

void sotStepComputerJoystick::display( std::ostream& os ) const
{
  os << "sotStepComputer <" << getName() <<">:" << std::endl;
}


void sotStepComputerJoystick::commandLine( const std::string& cmdLine,
				   std::istringstream& cmdArgs,
				   std::ostream& os )
{
  if( cmdLine == "help" )
  {
    os << "NextStep: " << std::endl
       << " - verbose [OFF]" << std::endl
       << " - state [{start|stop}] \t get/set the stepper state. " << std::endl
       << " - yZeroStep [<value>] \t get/set the Y default position." << std::endl
       << std::endl;
  }
  else   if( cmdLine == "thisIsZero" )
  {
    
    os << "Not supported" << std::endl;
  }
  else { Entity::commandLine( cmdLine,cmdArgs,os); }
}

ml::Vector & sotStepComputerJoystick::getlaststep(ml::Vector& res, int time)
{
  if (res.size()!=4)
    res.resize(4);

  res(0) = m_laststep[0];
  res(1) = m_laststep[1];
  res(2) = m_laststep[2];
  res(3) = (double)contactFootSIN(time);
  return res;
}
