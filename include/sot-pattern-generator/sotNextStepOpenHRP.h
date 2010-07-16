/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      NextStep.h
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



#ifndef __SOT_NextStep_OHRP_H__
#define __SOT_NextStep_OHRP_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* SOT */
#include <sot/NextStep.h>
#include "StackOfTasks.h"  

/* STD */
#include <string>
#include <deque>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (NextStepOpenHRP_EXPORTS) 
#    define NextStepOPENHRP_EXPORT __declspec(dllexport)
#  else  
#    define NextStepOPENHRP_EXPORT __declspec(dllimport)
#  endif 
#else
#  define NextStepOPENHRP_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


class NextStepOPENHRP_EXPORT NextStepOpenHRP
:public NextStep
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:

  StackOfTasks *sot_ptr;
  
 public: /* --- CONSTRUCTION --- */
  NextStepOpenHRP( const std::string& name );
  virtual ~NextStepOpenHRP( void ) {} 


 public: /* --- FUNCTIONS --- */

  virtual void starter( const int & timeCurr );
  virtual void stoper( const int & timeCurr );
  virtual void introductionCallBack( const int & timeCurr );


};





#endif // #ifndef __SOT_NextStep_OHRP_H__

