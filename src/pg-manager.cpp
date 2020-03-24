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

#include <dynamic-graph/factory.h>
#include <sot/pattern-generator/pg-manager.h>
#include <sot/pattern-generator/pg.h>
#include <sot/core/debug.hh>

namespace dynamicgraph {
namespace sot {

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PGManager, "PGManager");

PGManager::PGManager(const std::string &name) : Entity(name) {
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
}

void PGManager::startSequence(const StepQueue &seq) {
  if (!spg_) {
    sotERROR << "PG not set" << std::endl;
    return;
  }

  std::ostringstream cmdstd;
  cmdstd << ":StartOnLineStepSequencing ";

  for (unsigned int i = 0; i < seq.size(); ++i) {
    const FootPrint &fp = seq.getStep(i);
    cmdstd << fp.x << " " << fp.y << " " << fp.theta << " ";
  }

  std::istringstream cmdArg(cmdstd.str());
  std::istringstream emptyArg;
  spg_->InitState();
  spg_->pgCommandLine(cmdArg.str());

  sotDEBUG(15) << "Cmd: " << cmdstd.str() << std::endl;
}

void PGManager::stopSequence(const StepQueue &seq) {
  if (!spg_) {
    sotERROR << "PG not set" << std::endl;
    return;
  }

  std::ostringstream cmdstd;
  cmdstd << ":StopOnLineStepSequencing";
  std::istringstream cmdArg(cmdstd.str());
  spg_->pgCommandLine(cmdArg.str());
}

void PGManager::introduceStep(StepQueue &queue) {
  if (!spg_) {
    sotERROR << "Walk plugin not found. " << std::endl;
    return;
  }

  const FootPrint &lastStep = queue.getLastStep();

  std::string cmdLine = "addStep";
  std::ostringstream cmdArgIn;
  cmdArgIn << lastStep.x << " " << lastStep.y << " " << lastStep.theta;
  std::istringstream cmdArg(cmdArgIn.str());
  spg_->pgCommandLine(cmdArg.str());
}

double PGManager::changeNextStep(StepQueue &queue) {
  double stepTime = -1.;

  const FootPrint &step = queue.getFirstStep();
  stepbuf_.push_back(step);

  if (queue.isFirstStepChanged()) {
    PatternGeneratorJRL::FootAbsolutePosition aFAP;
    const FootPrint &change = queue.getFirstStepChange();
    aFAP.x = change.x - step.x;
    aFAP.y = change.y - step.y;
    aFAP.theta = change.theta - step.theta;
    pgi_->ChangeOnLineStep(0.805, aFAP, stepTime);
  }

  return stepTime;
}

void PGManager::display(std::ostream &os) const {
  os << "PGManager <" << getName() << ">:" << std::endl;
}

void PGManager::commandLine(const std::string &cmdLine,
                            std::istringstream &cmdArgs, std::ostream &os) {
  if (cmdLine == "help") {
    os << "StepTimeLine: " << std::endl << std::endl;
  } else if ("initPg" == cmdLine) {
    std::string name = "pg";
    cmdArgs >> std::ws;
    if (cmdArgs.good()) {
      cmdArgs >> name;
    }
    Entity *pgEntity = &(PoolStorage::getInstance()->getEntity(name));
    spg_ = dynamic_cast<PatternGenerator *>(pgEntity);
    pgi_ = spg_->GetPatternGeneratorInterface();
  } else if ("savesteps" == cmdLine) {
    std::ofstream os("/tmp/steps.dat");
    for (size_t i = 0; i < stepbuf_.size(); ++i) {
      os << stepbuf_[i].contact << " " << stepbuf_[i].x << " " << stepbuf_[i].y
         << " " << stepbuf_[i].theta << "\n";
    }
    os << std::endl;
    stepbuf_.clear();
  } else {
  }
}

}  // namespace sot
}  // namespace dynamicgraph
