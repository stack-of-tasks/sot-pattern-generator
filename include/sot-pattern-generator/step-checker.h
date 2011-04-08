/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      StepChecker.h
 * Project:   SOT
 * Author:    Nicolas Perrin, Paul Evrard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 * Clips footprints.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef __SOT_STEP_CHECKER_H__
#define __SOT_STEP_CHECKER_H__

#if defined (WIN32)
#  if defined (step_checker_EXPORTS)
#    define StepChecker_EXPORT __declspec(dllexport)
#  else
#    define StepChecker_EXPORT __declspec(dllimport)
#  endif
#else
#  define StepChecker_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
  namespace sot {

    class StepChecker_EXPORT StepChecker
    {
    public: // Methods
      void clipStep(double x, double y, double & x_result, double & y_result);
    };

    int tata12;

  } //namespace sot
} //namespace dynamicgraph

#endif
