/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      NextStepOpenHRP.h
 * Project:   SOT
 * Author:    Nicolas Mansard
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

#include "NextStepOpenHRP.h"
#include "StackOfTasks.h"

#include <sot-core/debug.h>
#include <dynamic-graph/factory.h>
#include <sot/sotMacrosSignal.h>
#include <sot-pattern-generator/exception-pg.h>
#include <dynamic-graph/pool.h>

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(NextStepOpenHRP,"NextStepOpenHRP");
 

/* --- CONSTRUCT ------------------------------------------------------------- */
/* --- CONSTRUCT ------------------------------------------------------------- */
/* --- CONSTRUCT ------------------------------------------------------------- */

NextStepOpenHRP::
NextStepOpenHRP( const std::string & name ) 
  :NextStep(name)
{
  sotDEBUGIN(5);

  Entity & ohrp = g_pool.getEntity( "OpenHRP" );
  sot_ptr = dynamic_cast< StackOfTasks* > (&ohrp);
  if(! sot_ptr ) { sotERROR << "SOT plugin not found." << std::endl; }

  sotDEBUGOUT(5);
}


/* --- FUNCTIONS ------------------------------------------------------------ */
/* --- FUNCTIONS ------------------------------------------------------------ */
/* --- FUNCTIONS ------------------------------------------------------------ */

void NextStepOpenHRP::
starter( const int & timeCurr )
{
  sotDEBUGIN(15); 

  NextStep::starter( timeCurr );
  if( (NULL!=sot_ptr)&&(NULL!=sot_ptr->walkGenpluginPTR) )
    {
      std::ostringstream cmdstd;
      cmdstd << ":StartOnLineStepSequencing ";
      for( std::deque< FootPrint >::const_iterator iter = footPrintList.begin();
	   iter!=footPrintList.end();++iter )
	{
	  cmdstd <<  iter->x << " " << iter->y << " " << iter->theta << " ";
	}

      CORBA::String_var cmdstart( cmdstd.str().c_str());
      sot_ptr->walkGenpluginPTR->ParseCmd( cmdstart );
      sotDEBUG(15) << "Cmd: " << cmdstd.str() << std::endl;
      sot_ptr->walkGenpluginPTR->startStepping();
    } else { sotERROR <<"Walk plugin not found. " << std::endl; }

  state = STATE_STARTED;

  sotDEBUGOUT(15); 
  return;
}

void NextStepOpenHRP::
stoper( const int & timeCurr )
{
  sotDEBUGIN(15); 

  CORBA::String_var csfinish(":StopOnLineStepSequencing");
  if( (NULL!=sot_ptr)&&(NULL!=sot_ptr->walkGenpluginPTR) )
    {
      sot_ptr->walkGenpluginPTR->ParseCmd(csfinish);
    } else { sotERROR << "Walk plugin not found. " <<std::endl; }

  state = STATE_STOPED;
  
  sotDEBUGOUT(15); 
  return;
}

void NextStepOpenHRP::
introductionCallBack( const int & timeCurr ) 
{
  sotDEBUGIN(15); 

  if( state==STATE_STARTED ) 
    {
      
      FootPrint & lastStep =  footPrintList.back();
      if( (NULL!=sot_ptr)&&(NULL!=sot_ptr->walkGenpluginPTR) )
	{
	  switch( lastStep.contact )
	    {
	    case CONTACT_LEFT_FOOT:
	      sot_ptr->walkGenpluginPTR->
		setLfootPosNoWait(lastStep.x,lastStep.y,lastStep.theta);
	      break;
	    case CONTACT_RIGHT_FOOT:
	      sot_ptr->walkGenpluginPTR->
		setRfootPosNoWait(lastStep.x,lastStep.y,lastStep.theta);
	      break;

	    };
	} else { sotERROR << "Walk plugin not found. " << std::endl; }
    }

  sotDEBUGOUT(15); 
  return;

}
