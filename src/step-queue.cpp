/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      StepQueue.cpp
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

#include <sot-pattern-generator/step-queue.h>
#include <sot-core/debug.h>
#include <dynamic-graph/factory.h>

using namespace sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(StepQueue,"StepQueue");


FootPrint::FootPrint()
: x(0.), y(0.), theta(0.), contact(CONTACT_RIGHT_FOOT)
{
}


FootPrint::FootPrint(double nx, double ny, double ntheta, ContactName ncontact)
: x(nx), y(ny), theta(ntheta), contact(ncontact)
{
}


const unsigned int StepQueue::QUEUE_SIZE = 4;
const double StepQueue::ZERO_STEP_POSITION = 0.19;
const FootPrint StepQueue::START_FOOT_PRINT(0.0, -ZERO_STEP_POSITION/2., 0.0, CONTACT_RIGHT_FOOT);


StepQueue::StepQueue( const std::string& name )
  : Entity(name)
  , footPrintList()
  , firstStepChanged(false)
{
  startSequence();
}


void StepQueue::startSequence()
{
  footPrintList.clear();
  footPrintList.push_back( START_FOOT_PRINT );
  footPrintList.push_back( FootPrint(0., ZERO_STEP_POSITION, 0., CONTACT_LEFT_FOOT) );
  footPrintList.push_back( FootPrint(0., -ZERO_STEP_POSITION, 0., CONTACT_RIGHT_FOOT) );
  footPrintList.push_back( FootPrint(0., ZERO_STEP_POSITION, 0., CONTACT_LEFT_FOOT) );

  firstStepChanged = false;
}


void StepQueue::pushStep( double x, double y, double theta )
{
  FootPrint footprint;
  footprint.x = x;
  footprint.y = y;
  footprint.theta = theta;

  const FootPrint& last = footPrintList.back();

  if(last.contact == CONTACT_LEFT_FOOT) {
    footprint.contact = CONTACT_RIGHT_FOOT;
  }
  else {
    footprint.contact = CONTACT_LEFT_FOOT;
  }

  footPrintList.push_back(footprint);
  footPrintList.pop_front();

  firstStepChanged = false;
}


void StepQueue::changeFirstStep( double x, double y, double theta )
{
  firstStepChange.x = x;
  firstStepChange.y = y;
  firstStepChange.theta = theta;

  firstStepChanged = true;
}


const FootPrint& StepQueue::getFirstStepChange() const
{
  return firstStepChange;
}


const FootPrint& StepQueue::getStep( unsigned int index ) const
{
  return footPrintList[index];
}


const FootPrint& StepQueue::getFirstStep() const
{
  return footPrintList.front();
}


const FootPrint& StepQueue::getLastStep() const
{
  return footPrintList.back();
}


bool StepQueue::isFirstStepChanged() const
{
  return firstStepChanged;
}


unsigned int StepQueue::size() const
{
  return QUEUE_SIZE;
}


const FootPrint& StepQueue::getStartFootPrint() const
{
  return START_FOOT_PRINT;
}


double StepQueue::getZeroStepPosition() const
{
  return ZERO_STEP_POSITION;
}


void StepQueue::display( std::ostream& os ) const
{
  os << "StepQueue <" << getName() << ">:" << std::endl;

  for(size_t i = 0; i < footPrintList.size(); ++i) {
    const FootPrint& fp = footPrintList[i];
    os << "step " << i << ": " << fp.contact << ", (" << fp.x << " " << fp.y << " " << fp.theta << ")" << std::endl;
  }
}


void StepQueue::commandLine( const std::string& cmdLine,
                                std::istringstream& cmdArgs,
                                std::ostream& os )
{
  if( cmdLine == "help" )
  {
    os << "StepQueue: " << std::endl
       << std::endl;
  }
  else { Entity::commandLine( cmdLine,cmdArgs,os); }
}








