/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotStepChecker.h
 * Project:   SOT
 * Author:    Nicolas Perrin, Paul Evrard, Nicolas Mansard
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


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (sotStepChecker_EXPORTS) 
#    define SOTSTEPCHECKER_EXPORT __declspec(dllexport)
#  else  
#    define SOTSTEPCHECKER_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTSTEPCHECKER_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- FUNCTION -------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTSTEPCHECKER_EXPORT sotStepChecker
{
public: // Methods

  void clipStep(double x, double y, double & x_result, double & y_result);
};


#endif
