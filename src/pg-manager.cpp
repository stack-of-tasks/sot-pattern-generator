/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      PGManager.cpp
 * Project:   SOT
 * Author:    Olivier Stasse, Paul Evrard
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

#include <sot-pattern-generator/pg-manager.h>
#include <sot/core/debug.hh>
#include <sot-pattern-generator/pg.h>
#include <dynamic-graph/factory.h>

namespace dynamicgraph {
  namespace sot {

    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PGManager,"PGManager");

    PGManager::PGManager( const std::string& name )
      : Entity(name)
    {
      sotDEBUGIN(5);

      sotDEBUGOUT(5);
    }


    void PGManager::startSequence( const StepQueue& seq )
    {
      if(!pgEntity) {
	sotERROR <<"PG not set" << std::endl;
	return;
      }

      std::ostringstream cmdstd; cmdstd << ":StartOnLineStepSequencing ";
      std::ostringstream os;

      for( unsigned int i = 0; i < seq.size(); ++i ) {
	const FootPrint& fp = seq.getStep(i);
	cmdstd << fp.x << " " << fp.y << " " << fp.theta << " ";
      }

      std::istringstream cmdArg( cmdstd.str() );
      std::istringstream emptyArg;

      sotDEBUG(15) << "Cmd: " << cmdstd.str() << std::endl;
    }


    void PGManager::stopSequence( const StepQueue& seq )
    {
      if(!pgEntity) {
	sotERROR <<"PG not set" << std::endl;
	return;
      }

      std::ostringstream cmdstd; cmdstd << ":StopOnLineStepSequencing";
      std::ostringstream os;
      std::istringstream cmdArg( cmdstd.str() );
    }


    void PGManager::introduceStep( StepQueue& queue )
    {
      if(!pgEntity) {
	sotERROR << "Walk plugin not found. " << std::endl;
	return;
      }

      const FootPrint& lastStep = queue.getLastStep();

      std::string cmdLine = "addStep";
      std::ostringstream cmdArgIn, os;
      cmdArgIn << lastStep.x << " " << lastStep.y << " " << lastStep.theta;
      std::istringstream cmdArg( cmdArgIn.str() );
    }


    double PGManager::changeNextStep( StepQueue& queue )
    {
      double stepTime = -1.;

      const FootPrint& step = queue.getFirstStep();
      stepbuf.push_back(step);

      if(queue.isFirstStepChanged()) {
	PatternGeneratorJRL::FootAbsolutePosition aFAP;
	const FootPrint& change = queue.getFirstStepChange();
	aFAP.x = change.x - step.x;
	aFAP.y = change.y - step.y;
	aFAP.theta = change.theta - step.theta;
	pgi->ChangeOnLineStep(0.805, aFAP, stepTime);
      }

      return stepTime;
    }


    void PGManager::display( std::ostream& os ) const
    {
      os << "PGManager <" << getName() << ">:" << std::endl;
    }
  } // namespace dg
} // namespace sot
