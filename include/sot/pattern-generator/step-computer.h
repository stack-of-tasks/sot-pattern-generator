/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      StepComputer.h
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
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#ifndef __SOT_StepComputer_H__
#define __SOT_StepComputer_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/pattern-generator/step-checker.h>
#include <sot/pattern-generator/step-observer.h>

#include <sot/core/matrix-geometry.hh>

/* STD */
#include <deque>
#include <fstream>
#include <string>

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class StepQueue;

/// Generates footsteps.
class StepComputer {
 public:  // Methods
  virtual void changeFirstStep(StepQueue &queue, int timeCurr) = 0;
  virtual void nextStep(StepQueue &queue, int timeCurr) = 0;
  virtual ~StepComputer(){};
};

}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __SOT_STEPCOMPUTER_H__
