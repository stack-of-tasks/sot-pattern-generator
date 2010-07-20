/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotPGManager.cpp
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
 * PGManager entity: configures the PG and sends steps.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <sot/sotPGManager.h>
#include <sot-core/debug.h>
#include <sot/sotPatternGenerator.h>
#include <dynamic-graph/factory.h>


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(sotPGManager,"PGManager");


sotPGManager::sotPGManager( const std::string& name )
  : Entity(name)
{
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
}


void sotPGManager::startSequence( const sotStepQueue& seq )
{
  if(!pgEntity) {
    sotERROR <<"PG not set" << std::endl;
    return;
  }

  std::ostringstream cmdstd; cmdstd << ":StartOnLineStepSequencing ";
  std::ostringstream os;

  for( unsigned int i = 0; i < seq.size(); ++i ) {
    const sotFootPrint& fp = seq.getStep(i);
    cmdstd << fp.x << " " << fp.y << " " << fp.theta << " ";
  }

  std::istringstream cmdArg( cmdstd.str() );
  std::istringstream emptyArg;
  pgEntity->commandLine( "initState", emptyArg, os );
  pgEntity->commandLine( "parsecmd", cmdArg, os );

  sotDEBUG(15) << "Cmd: " << cmdstd.str() << std::endl;
}


void sotPGManager::stopSequence( const sotStepQueue& seq )
{
  if(!pgEntity) {
    sotERROR <<"PG not set" << std::endl;
    return;
  }

  std::ostringstream cmdstd; cmdstd << ":StopOnLineStepSequencing";
  std::ostringstream os;
  std::istringstream cmdArg( cmdstd.str() );
  pgEntity->commandLine( "parsecmd", cmdArg, os );
}


void sotPGManager::introduceStep( sotStepQueue& queue )
{
  if(!pgEntity) {
    sotERROR << "Walk plugin not found. " << std::endl;
    return;
  }

  const sotFootPrint& lastStep = queue.getLastStep();

  std::string cmdLine = "addStep";
  std::ostringstream cmdArgIn, os;
  cmdArgIn << lastStep.x << " " << lastStep.y << " " << lastStep.theta;
  std::istringstream cmdArg( cmdArgIn.str() );
  pgEntity->commandLine(cmdLine,cmdArg,os);
}


double sotPGManager::changeNextStep( sotStepQueue& queue )
{
  double stepTime = -1.;

  const sotFootPrint& step = queue.getFirstStep();
  stepbuf.push_back(step);

  if(queue.isFirstStepChanged()) {
    PatternGeneratorJRL::FootAbsolutePosition aFAP;
    const sotFootPrint& change = queue.getFirstStepChange();
    aFAP.x = change.x - step.x;
    aFAP.y = change.y - step.y;
    aFAP.theta = change.theta - step.theta;
    pgi->ChangeOnLineStep(0.805, aFAP, stepTime);
  }

  return stepTime;
}


void sotPGManager::display( std::ostream& os ) const
{
  os << "sotPGManager <" << getName() << ">:" << std::endl;
}


void sotPGManager::commandLine( const std::string& cmdLine,
                                   std::istringstream& cmdArgs,
                                   std::ostream& os )
{
  if( cmdLine == "help" )
  {
    os << "StepTimeLine: " << std::endl
       << std::endl;
  }
  else if( "initPg" == cmdLine )
  {
    std::string name = "pg";
    cmdArgs >> std::ws;
    if( cmdArgs.good()){ cmdArgs >> name; }
    pgEntity = &g_pool.getEntity( name );
    sotPatternGenerator* spg = dynamic_cast<sotPatternGenerator*>(pgEntity);
    if (spg){ pgi = spg->GetPatternGeneratorInterface(); }
  }
  else if( "savesteps" == cmdLine )
  {
    std::ofstream os("/tmp/steps.dat");
    for(size_t i = 0; i < stepbuf.size(); ++i)
    {
      os << stepbuf[i].contact << " "
          << stepbuf[i].x << " "
          << stepbuf[i].y << " "
          << stepbuf[i].theta << "\n";
    }
    os << std::endl;
    stepbuf.clear();
  }
  else { Entity::commandLine( cmdLine,cmdArgs,os); }
}








