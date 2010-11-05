/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      NextStepPgSot.h
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

#include <algorithm>

#include <jrl/mal/matrixabstractlayer.hh>
#include <sot-pattern-generator/next-step-pg-sot.h>

#include <sot-core/debug.h>
#include <dynamic-graph/factory.h>
#include <sot-core/macros-signal.h>
#include <sot-pattern-generator/exception-pg.h>
#include <dynamic-graph/pool.h>
#include<cmath>
#define PI 3.1416


using namespace sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(NextStepPgSot,"NextStepPgSot");
 

/* --- CONSTRUCT ------------------------------------------------------------- */
/* --- CONSTRUCT ------------------------------------------------------------- */
/* --- CONSTRUCT ------------------------------------------------------------- */

NextStepPgSot::
NextStepPgSot( const std::string & name ) 
  :NextStep(name)
{
  sotDEBUGIN(5);
  m_StepModificationMode = NextStepPgSot::ADDING_STEP;
  m_NextStepTime = -1.0;
  m_NbOfFirstSteps = 0;
  m_PGI = 0;
  sotDEBUGOUT(5);

  stepbuf.reserve(10000);
}


/* --- FUNCTIONS ------------------------------------------------------------ */
/* --- FUNCTIONS ------------------------------------------------------------ */

void positionClipper(double x, double y, double & x_result, double & y_result) {
  const double MIN_y = 0.16;
  const double MAX_y = 0.40;
  const double MAX_x = 0.25;  
  const double EDGE_POINT_x = 0.20;
  const double EDGE_POINT_y = 0.30;
  
  double x0 = std::abs(x);
  double y0 = std::abs(y);
  double alpha = y0/x0;
  double y_inter = MIN_y;
  double x_inter = MIN_y/alpha;
  
  if(alpha < MIN_y/MAX_x) {
    x_result = MAX_x;
    y_result = MIN_y;
  }
  else if(alpha < EDGE_POINT_y / EDGE_POINT_x) {
    double delta1 = (MIN_y - EDGE_POINT_y) / (MAX_x - EDGE_POINT_x);
    double x_inter2 = (EDGE_POINT_y - delta1 * EDGE_POINT_x) / (alpha - delta1);
    double y_inter2 = alpha * x_inter2;
    if(x0 < x_inter) {
      x_result = x_inter;
      y_result = y_inter;
    } 
    else if(x0 > x_inter2) {
      x_result = x_inter2;
      y_result = y_inter2;
    }
    else {
      x_result = x0;
      y_result = y0;
    }
  }
  else {
    double delta2 = (EDGE_POINT_y - MAX_y) / (EDGE_POINT_x);
    double x_inter2 = (MAX_y) / (alpha - delta2);
    double y_inter2 = alpha * x_inter2;
    if(x0 < x_inter) {
      x_result = x_inter;
      y_result = y_inter;
    } 
    else if(x0 > x_inter2) {
      x_result = x_inter2;
      y_result = y_inter2;
    }
    else {
      x_result = x0;
      y_result = y0;
    }    
  }

  if(x<0) x_result = -x_result;
  if(y<0) y_result = -y_result;

}

/* --- FUNCTIONS ------------------------------------------------------------ */

void NextStepPgSot::
starter( const int & timeCurr )
{
  sotDEBUGIN(15);

  NextStep::starter(timeCurr);

  if(pgEntity) {
    std::ostringstream cmdstd; std::ostringstream os;
    cmdstd << ":StartOnLineStepSequencing ";
    for( std::deque< FootPrint >::const_iterator iter = footPrintList.begin();
	   iter!=footPrintList.end();++iter )
    {
      cmdstd <<  iter->x << " " << iter->y << " " << iter->theta << " ";
    }
    std::istringstream cmdArg( cmdstd.str() );
    std::istringstream emptyArg;
    pgEntity->commandLine( "initState", emptyArg, os );
    pgEntity->commandLine( "parsecmd", cmdArg, os );
    sotDEBUG(15) << "Cmd: " << cmdstd.str() << std::endl;
  } else { sotERROR <<"PG not set" << std::endl; }

  state = STATE_STARTED;
  
  sotDEBUGOUT(15); 
  return;
}

void NextStepPgSot::
stoper( const int & timeCurr )
{
  sotDEBUGIN(15);

  if(pgEntity) {
    std::ostringstream cmdstd; std::ostringstream os;
    cmdstd << ":StopOnLineStepSequencing";
    std::istringstream cmdArg( cmdstd.str() );
    pgEntity->commandLine( "parsecmd", cmdArg, os );
  } else { sotERROR <<"PG not set" << std::endl; }

  m_NbOfFirstSteps = 0;
  state = STATE_STOPED;
  
  sotDEBUGOUT(15); 
  return;
}

void NextStepPgSot::
introductionCallBack( const int & timeCurr ) 
{
  sotDEBUGIN(15); 

  if( state==STATE_STARTED ) 
    {
      
      FootPrint & lastStep =  footPrintList.back();
      if( pgEntity ) 
	{
	  if (m_StepModificationMode == NextStepPgSot::ADDING_STEP)
	    {
	      std::string cmdLine = "addStep";
	      std::ostringstream cmdArgIn; std::ostringstream os;
	      cmdArgIn << lastStep.x << " " << lastStep.y << " " << lastStep.theta;
	      std::istringstream cmdArg( cmdArgIn.str() );
	      pgEntity->commandLine(cmdLine,cmdArg,os); 
	    }
	  else if (m_StepModificationMode == NextStepPgSot::CHANGING_STEP)
	    {
	      if (m_NbOfFirstSteps<3)
		m_NbOfFirstSteps++;
	      else
	      {
		sotDEBUG(15) << "m_NextStepTime: "<< m_NextStepTime << std::endl;

		PatternGeneratorJRL::FootAbsolutePosition aFAP;

		unsigned int lSupportFoot=0;

		lSupportFoot = m_sPG->SupportFootSOUT(timeCurr);
		
		double NextFAPx = lastStep.x;
		double NextFAPy = lastStep.y;
		double NextFAPtheta = lastStep.theta;
		if (1)
		  {
		    NextFAPx = 0.80*(double)rand()/(double)RAND_MAX-0.40;
		    NextFAPy = 0.80*(double)rand()/(double)RAND_MAX-0.40;
		    NextFAPtheta = 18.0*(double)rand()/(double)RAND_MAX - 6.0;
		  }
		else 
		  {
		    NextFAPx = 0.2;
		    NextFAPy = 0.0;
		    NextFAPtheta = 45.0;			    
		  }

		/* Warning : HARDCODED VALUES */
#if 1
		if(0 && mode == MODE_1D)
		{
		  const double LIMIT_X = 0.20;
		  if (NextFAPx<-LIMIT_X) 
		    NextFAPx = -LIMIT_X;
		  if (NextFAPx>LIMIT_X) 
		    NextFAPx = LIMIT_X;
		  NextFAPtheta = 0.;
		  NextFAPy = 0.;
		}
		else
		{
		  //const double LIMIT_X = 0.10;
		  //const double LIMIT_Y_SUP = 0.05;
		  //const double LIMIT_Y_INF = 0.03;
		  const double THETA_MAX = 9.;

		  if (NextFAPtheta<-THETA_MAX) 
		    NextFAPtheta = -THETA_MAX;
		  if (NextFAPtheta>THETA_MAX) 
		    NextFAPtheta = THETA_MAX;
		
		  if (lSupportFoot==1)
		  { 
		    if(NextFAPy < 0) NextFAPy = -NextFAPy;
		    double Next_xResult;
		    double Next_yResult;
		    positionClipper(NextFAPx, NextFAPy, Next_xResult, Next_yResult);
		    NextFAPx = cos(NextFAPtheta*PI/180) * Next_xResult + sin(NextFAPtheta*PI/180) * Next_yResult;
		    NextFAPy = -sin(NextFAPtheta*PI/180) * Next_xResult + cos(NextFAPtheta*PI/180) * Next_yResult;   

		    // This is the right foot support.
		    // Make sure this is relative to the 
		    // next stepping value.
			std::cout << NextFAPx << " " << NextFAPy << " " << NextFAPtheta  << std::endl;
		    NextFAPy= NextFAPy - 0.19;
		  }
		  else 
		  {
		    if(NextFAPy > 0) NextFAPy = -NextFAPy;
		    double Next_xResult;
		    double Next_yResult;
		    positionClipper(NextFAPx, NextFAPy, Next_xResult, Next_yResult);
		    NextFAPx = cos(NextFAPtheta*PI/180) * Next_xResult + sin(NextFAPtheta*PI/180) * Next_yResult;
		    NextFAPy = -sin(NextFAPtheta*PI/180) * Next_xResult + cos(NextFAPtheta*PI/180) * Next_yResult;   

		    // This is the left foot support.
		    // Make sure this is relative to the 
		    // next stepping value.
			std::cout << NextFAPx << " " << NextFAPy << " " << NextFAPtheta << std::endl;
		    NextFAPy= NextFAPy + 0.19;
	      	    		    
		  }
		}
#endif
		/* End of Warning : HARDCODED VALUES */
		aFAP.x = NextFAPx;
		aFAP.y = NextFAPy;
		aFAP.theta= NextFAPtheta;
		//std::cout << lSupportFoot << std::endl;
		//std::cout << "FAP = ( "<< aFAP.x << " " << aFAP.y << " " << aFAP.z << " ) " << std::endl;
		stepbuf.push_back(std::make_pair(lSupportFoot, aFAP));
		m_NextStepTime = 0.0;

		m_PGI->ChangeOnLineStep(0.805,aFAP,m_NextStepTime);

		if (m_NextStepTime<0.0)
		  period =NextStep::PERIOD_DEFAULT;
		else
		  period = (unsigned int)((m_NextStepTime+0.025)*200.0);
		
	      }
	    }
	} else {
	  sotERROR << "Walk plugin not found. " << std::endl; 
	  /* TODO Error: PG not set. */ 
	}
    }

  sotDEBUGOUT(15); 
  return;

}



void NextStepPgSot::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( "help"==cmdLine )
    {
      os << "NextStepPgSot: " << std::endl 
	 << " - initPg [<pg_name>]" << std::endl
	 << " - stepmodificationmode [add|change]" <<std::endl;
      NextStep::commandLine(cmdLine, cmdArgs, os);
    }
  else if( "initPg" == cmdLine )
    {
      std::string name = "pg";
      cmdArgs >> std::ws; if( cmdArgs.good()) cmdArgs >> name;
      pgEntity = &g_pool.getEntity( name );
      m_sPG = dynamic_cast<PatternGenerator *>(pgEntity);
      if (m_sPG!=0)
	m_PGI = m_sPG->GetPatternGeneratorInterface();
    }
  else if( "stepmodificationmode" == cmdLine)
    {
      std::string stgmode;
      cmdArgs >> std::ws; 
      if( cmdArgs.good()) 
	{
	  cmdArgs >> stgmode;
	  if (stgmode=="add")
	    m_StepModificationMode=NextStepPgSot::ADDING_STEP;
	  else if (stgmode=="change")
	    m_StepModificationMode=NextStepPgSot::CHANGING_STEP;
	}
      else
	{
	  if (m_StepModificationMode==NextStepPgSot::ADDING_STEP)
	    os << "add";
	  else if (m_StepModificationMode==NextStepPgSot::CHANGING_STEP)
	    os << "change";
	}
    }
  else if( "savesteps" == cmdLine )
    {
      std::ofstream os("/tmp/steps.dat");
      for(size_t i = 0; i < stepbuf.size(); ++i)
      {
	os << stepbuf[i].first << " "
	   << stepbuf[i].second.x << " "
	   << stepbuf[i].second.y << " "
	   << stepbuf[i].second.z << "\n";
      }
      os << std::endl;
      stepbuf.clear();
    }
  else { NextStep::commandLine( cmdLine,cmdArgs,os ); }

}

