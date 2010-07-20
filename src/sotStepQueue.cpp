/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotStepQueue.cpp
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

#include <sot/sotStepQueue.h>
#include <sot-core/debug.h>
#include <dynamic-graph/factory.h>


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(sotStepQueue,"StepQueue");


sotFootPrint::sotFootPrint()
: x(0.), y(0.), theta(0.), contact(CONTACT_RIGHT_FOOT)
{
}


sotFootPrint::sotFootPrint(double nx, double ny, double ntheta, ContactName ncontact)
: x(nx), y(ny), theta(ntheta), contact(ncontact)
{
}


const unsigned int sotStepQueue::QUEUE_SIZE = 4;
const double sotStepQueue::ZERO_STEP_POSITION = 0.19;
const sotFootPrint sotStepQueue::START_FOOT_PRINT(0.0, -ZERO_STEP_POSITION/2., 0.0, CONTACT_RIGHT_FOOT);


sotStepQueue::sotStepQueue( const std::string& name )
  : Entity(name)
  , footPrintList()
  , firstStepChanged(false)
{
  startSequence();
}


void sotStepQueue::startSequence()
{
  footPrintList.clear();
  footPrintList.push_back( START_FOOT_PRINT );
  footPrintList.push_back( sotFootPrint(0., ZERO_STEP_POSITION, 0., CONTACT_LEFT_FOOT) );
  footPrintList.push_back( sotFootPrint(0., -ZERO_STEP_POSITION, 0., CONTACT_RIGHT_FOOT) );
  footPrintList.push_back( sotFootPrint(0., ZERO_STEP_POSITION, 0., CONTACT_LEFT_FOOT) );

  firstStepChanged = false;
}


void sotStepQueue::pushStep( double x, double y, double theta )
{
  sotFootPrint footprint;
  footprint.x = x;
  footprint.y = y;
  footprint.theta = theta;

  const sotFootPrint& last = footPrintList.back();

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


void sotStepQueue::changeFirstStep( double x, double y, double theta )
{
  firstStepChange.x = x;
  firstStepChange.y = y;
  firstStepChange.theta = theta;

  firstStepChanged = true;
}


const sotFootPrint& sotStepQueue::getFirstStepChange() const
{
  return firstStepChange;
}


const sotFootPrint& sotStepQueue::getStep( unsigned int index ) const
{
  return footPrintList[index];
}


const sotFootPrint& sotStepQueue::getFirstStep() const
{
  return footPrintList.front();
}


const sotFootPrint& sotStepQueue::getLastStep() const
{
  return footPrintList.back();
}


bool sotStepQueue::isFirstStepChanged() const
{
  return firstStepChanged;
}


unsigned int sotStepQueue::size() const
{
  return QUEUE_SIZE;
}


const sotFootPrint& sotStepQueue::getStartFootPrint() const
{
  return START_FOOT_PRINT;
}


double sotStepQueue::getZeroStepPosition() const
{
  return ZERO_STEP_POSITION;
}


void sotStepQueue::display( std::ostream& os ) const
{
  os << "sotStepQueue <" << getName() << ">:" << std::endl;

  for(size_t i = 0; i < footPrintList.size(); ++i) {
    const sotFootPrint& fp = footPrintList[i];
    os << "step " << i << ": " << fp.contact << ", (" << fp.x << " " << fp.y << " " << fp.theta << ")" << std::endl;
  }
}


void sotStepQueue::commandLine( const std::string& cmdLine,
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








