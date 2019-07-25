/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      StepTimeLine.cpp
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

#include <sot/core/debug.hh>
#include <sot-pattern-generator/step-time-line.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>

namespace dynamicgraph {
  namespace sot {

    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(StepTimeLine,"TimeLine");

    const unsigned int StepTimeLine::PERIOD_DEFAULT = 160;
    // 160iter=800ms
    const unsigned int StepTimeLine::FIRST_STEP_TO_MODIFY = 3;


    StepTimeLine::StepTimeLine( const std::string& name )
      : Entity(name)
      , triggerSOUT( "NextStep("+name+")::input(dummy)::trigger" )

      , stepQueue(0x0)
      , stepComputer(0x0)
      , pgManager(0x0)

      , state( STATE_STOPPED )
      , timeLastIntroduction( 0 )
      , period( PERIOD_DEFAULT )
      , nStartingSteps(0)
    {
      sotDEBUGIN(5);

      triggerSOUT.setFunction
        ( boost::bind(&StepTimeLine::triggerCall,this,_1,_2) );
      signalRegistration( triggerSOUT );

      sotDEBUGOUT(5);
    }


    int& StepTimeLine::triggerCall ( int& dummy, int timeCurrent )
    {
      sotDEBUGIN ( 45 );

      if( state == STATE_STARTED ) {
        int nextIntoductionTime = timeLastIntroduction + period;

        if( nextIntoductionTime <= timeCurrent ) { // condition (A)
          if(nStartingSteps < FIRST_STEP_TO_MODIFY) {
            ++nStartingSteps;
          }
          else {
            stepComputer->changeFirstStep( *stepQueue, timeCurrent );
            double stepTime = pgManager->changeNextStep( *stepQueue );
            if( stepTime < 0.0 ) {
              period = PERIOD_DEFAULT;
            }
            else {
              period = static_cast<unsigned int>
                ((stepTime + 0.025) * 200.0);
            }
          }

          stepComputer->nextStep( *stepQueue, timeCurrent );
          pgManager->introduceStep( *stepQueue );
          timeLastIntroduction = timeCurrent;
        }
      }
      else if( state == STATE_STARTING ) {
        stepQueue->startSequence();
        pgManager->startSequence( *stepQueue );
        timeLastIntroduction =
          timeCurrent - period + 1;
        // trick to satisfy condition (A) at next call
        nStartingSteps = 0;
        state = STATE_STARTED;
        period = PERIOD_DEFAULT;
      }
      else if( state == STATE_STOPPING ) {
        pgManager->stopSequence( *stepQueue );
        state = STATE_STOPPED;
      }
      else {
        // state == STATE_STOPPED: do nothing
      }

      sotDEBUGOUT ( 45 );
      return dummy;
    }


    void StepTimeLine::display( std::ostream& os ) const
    {
      os << "StepTimeLine <" << getName() << ">:" << std::endl;
      os << " - timeLastIntroduction: "
         << timeLastIntroduction << std::endl;
      os << " - period: " << period << std::endl;

      switch( state )
        {
        case STATE_STARTING: os << " - state: starting" << std::endl;
        case STATE_STARTED: os << " - state: started" << std::endl;
        case STATE_STOPPING: os << " - state: stopping" << std::endl;
        case STATE_STOPPED: os << " - state: stopped" << std::endl;
        }
    }

    void StepTimeLine::commandLine( const std::string& cmdLine,
                                    std::istringstream& cmdArgs,
                                    std::ostream& os )
    {
      if( cmdLine == "help" )
        {
          os << "StepTimeLine: " << std::endl
             << std::endl;
        }
      else if( cmdLine == "setComputer" )
        {
          std::string name = "stepcomp";
          cmdArgs >> std::ws;
          if( cmdArgs.good()){ cmdArgs >> name; }
          Entity* entity =
            &(PoolStorage::getInstance()->getEntity( name ));
          stepComputer = dynamic_cast<StepComputer*>(entity);
        }
      else if( cmdLine == "setQueue" )
        {
          std::string name = "stepqueue";
          cmdArgs >> std::ws;
          if( cmdArgs.good()){ cmdArgs >> name; }
          Entity* entity =
            &(PoolStorage::getInstance()->getEntity( name ));
          stepQueue = dynamic_cast<StepQueue*>(entity);
        }
      else if( cmdLine == "setPGManager" )
        {
          std::string name = "steppg";
          cmdArgs >> std::ws;
          if( cmdArgs.good()){ cmdArgs >> name; }
          Entity* entity =
            &(PoolStorage::getInstance()->getEntity( name ));
          pgManager = dynamic_cast<PGManager*>(entity);
        }
      else if( cmdLine == "state" )
        {
          cmdArgs >> std::ws;
          if( cmdArgs.good() ) {
            std::string statearg; cmdArgs >> statearg;
            if( statearg == "start" ){ state = STATE_STARTING; }
            else if( statearg == "stop" ){ state = STATE_STOPPING; }
          }
          else  {
            os << "state = ";
            switch( state )
              {
              case STATE_STARTING: os << "starting"; break;
              case STATE_STARTED: os << "started"; break;
              case STATE_STOPPING: os << "stopping"; break;
              case STATE_STOPPED: os << "stopped"; break;
              default: os << "should never happen"; break;
              }
            os << std::endl;
          }
        }
      else {  }
    }

  } // namespace dg
} // namespace sot
