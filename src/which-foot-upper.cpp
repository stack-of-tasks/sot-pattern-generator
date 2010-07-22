/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      WhichFootUpper.h
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

#include <sot-pattern-generator/which-foot-upper.h>
#include <sot-core/debug.h>
#include <dynamic-graph/factory.h>
#include <sot-core/macros-signal.h>
#include <sot-pattern-generator/exception-pg.h>

using namespace sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(WhichFootUpper,"WhichFootUpper");

const unsigned int WhichFootUpper::
INDEX_LEFT_FOOT_DEFAULT = 0;
const unsigned int WhichFootUpper::
INDEX_RIGHT_FOOT_DEFAULT = 1;

const double WhichFootUpper::
TRIGGER_THRESHOLD_DEFAULT = 5e-4;


typedef MatrixRotation& (MatrixHomogeneous::*ExtractMemberType) (MatrixRotation&) const;

WhichFootUpper::
WhichFootUpper( const std::string & name ) 
  :Entity(name)
   ,indexLeftFoot( INDEX_LEFT_FOOT_DEFAULT )
   ,indexRightFoot( INDEX_RIGHT_FOOT_DEFAULT )
   ,triggerThreshold( TRIGGER_THRESHOLD_DEFAULT )
   ,lastFoot( indexLeftFoot )
  
   ,waistRsensorSIN( NULL,"WhichFootUpper("+name+")::input(matrixRotation)::waistRsensor" ) 
   ,worldRsensorSIN( NULL,"WhichFootUpper("+name+")::input(matrixRotation)::worldRsensor" ) 
   ,waistMlfootSIN( NULL,"WhichFootUpper("+name+")::input(matrixhomogeneous)::waistMlfoot" ) 
   ,waistMrfootSIN( NULL,"WhichFootUpper("+name+")::input(matrixhomogeneous)::waistMrfoot" ) 
  
  ,worldMlfootSOUT( SOT_INIT_SIGNAL_3( WhichFootUpper::computeFootPosition,
				       waistMlfootSIN,MatrixHomogeneous,
				       waistRsensorSIN,MatrixRotation,
				       worldRsensorSIN,MatrixRotation),
		    "WhichFootUpper("+name+")::output(MatrixHomogeneous)::worldMlfoot" )
  ,worldMrfootSOUT( SOT_INIT_SIGNAL_3( WhichFootUpper::computeFootPosition,
				       waistMrfootSIN,MatrixHomogeneous,
				       waistRsensorSIN,MatrixRotation,
				       worldRsensorSIN,MatrixRotation),
		    "WhichFootUpper("+name+")::output(MatrixHomogeneous)::worldMrfoot" )
   ,whichFootSOUT( SOT_MEMBER_SIGNAL_2( WhichFootUpper::whichFoot,
					waistMlfootSIN,MatrixHomogeneous,
					waistMrfootSIN,MatrixHomogeneous),
		   "WhichFootUpper("+name+")::output(uint)::whichFoot" ) 
  
  ,waistMsensorSIN( NULL,"WhichFootUpper("+name+")::input(matrixRotation)::waistMsensor" ) 
  ,waistRsensorSOUT( boost::bind( (ExtractMemberType)&MatrixHomogeneous::extract,
				  SOT_CALL_SIG(waistMsensorSIN,MatrixHomogeneous),
				  _1),
		     waistMsensorSIN,
		     "WhichFootUpper("+name+")::output(MatrixHomogeneous)::waistRsensorOUT" )
{
  sotDEBUGIN(5);
  
  signalRegistration( whichFootSOUT      << waistRsensorSIN 
		      << worldRsensorSIN << waistMlfootSIN 
		      << waistMrfootSIN  << worldMlfootSOUT 
		      << worldMrfootSOUT 
		      << waistMsensorSIN << waistRsensorSOUT);
  waistRsensorSIN.plug( &waistRsensorSOUT );
  

  sotDEBUGOUT(5);
}


WhichFootUpper::
~WhichFootUpper( void )
{
  sotDEBUGINOUT(5);
  return;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */

MatrixHomogeneous & WhichFootUpper::
computeFootPosition( const MatrixHomogeneous& waistMfoot,
		     const MatrixRotation& waistRsensor,			 
		     const MatrixRotation& worldRsensor,			 
		     MatrixHomogeneous& worldMfoot )
{
  sotDEBUGIN(15);
  
  MatrixRotation worldRwaist;
  worldRsensor.multiply( waistRsensor.transpose(),worldRwaist );

  ml::Vector trans(3); trans.fill(0);
  MatrixHomogeneous worldMwaist; worldMwaist.buildFrom(worldRwaist,trans);
  
  worldMwaist.multiply(waistMfoot,worldMfoot );

  sotDEBUGOUT(15);
  return worldMfoot;
}

unsigned int & WhichFootUpper::
whichFoot( const MatrixHomogeneous& waistMlfoot,
	   const MatrixHomogeneous& waistMrfoot,
	   unsigned int& res )
{
  sotDEBUGIN(15);

  const double & leftAltitude = waistMlfoot(2,3);
  const double & rightAltitude = waistMrfoot(2,3);

  if( lastFoot==indexRightFoot )
    {
      if( rightAltitude-triggerThreshold<leftAltitude )
	{ res = lastFoot; }
      else { res = lastFoot = indexLeftFoot; }
    } else {
      if( leftAltitude-triggerThreshold<rightAltitude ) 
	{ res = lastFoot = indexLeftFoot; }
      else { res = lastFoot = indexRightFoot; }
    }

  sotDEBUGOUT(15);
  return res;
}

/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */

void WhichFootUpper::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  

  if( cmdLine == "help" )
    {
      os << "WhichFootUpper: " << std::endl
	 << " - index {left|right} [<value>]: get/set the foot indeces." << std::endl
	 << " - trigger [<value>]: get/set the trigger threshold. " << std::endl;
    }
  else if( cmdLine == "index" )
    {
      std::string foot; cmdArgs >> foot >> std::ws;
      unsigned int * classIndex = NULL;
      if( foot=="left" ) 
	{ classIndex = & indexLeftFoot;  }
      else if( foot=="right" ) 
	{ classIndex = & indexRightFoot; }
      else 
	{ 
	  os << "Error. Usage is: index {left|right} [<value>]" << std::endl; 
	  return;
	}
	
      if( cmdArgs.good() )
	{ cmdArgs >> (*classIndex);}
      else { os << "index = " << (*classIndex) << std::endl; }

    }
  else if( cmdLine == "trigger" )
    {
      cmdArgs >> std::ws; if( cmdArgs.good() ) { cmdArgs >> triggerThreshold; }
      else { os  << "trigger = " << triggerThreshold << std::endl; }
    }
  else { Entity::commandLine( cmdLine,cmdArgs,os); }
}

