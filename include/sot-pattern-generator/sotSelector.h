/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotSelector.h
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



#ifndef __SOT_SOTSELECTOR_H__
#define __SOT_SOTSELECTOR_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot-core/matrix-homogeneous.h>
#include <sot-core/vector-roll-pitch-yaw.h>
#include <sot-core/matrix-rotation.h>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (sotSelector_EXPORTS) 
#    define SOTSELECTOR_EXPORT __declspec(dllexport)
#  else  
#    define SOTSELECTOR_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTSELECTOR_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTSELECTOR_EXPORT sotSelector
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;

 protected:
  /** Number of signal type. For each signal type, you have
   * one output and <nbEntries> inputs. */
  unsigned int nbdg::Signals; 
  /** Number of possible values for the selector. For each entry,
   * you have one signal of each type. */
  unsigned int nbEntries;

 public: /* --- CONSTRUCTION --- */

  sotSelector( const std::string& name );
  virtual ~sotSelector( void );

 public: /* --- SIGNAL --- */

  dg::SignalPtr<unsigned int,int> selectorSIN; 

  std::vector< std::vector< sotdg::SignalAbstract<int>* > > inputsSIN;
  std::vector< sotdg::SignalAbstract<int>* > outputsSOUT;


 public: /* --- FUNCTIONS --- */

  template< class T >
    static T& computeSelection( const unsigned int & sigNum,
				std::vector< sotdg::SignalAbstract<int>* >& entriesSIN,
				T& res,const int& time );

  template< class T >
    unsigned int createdg::Signal( const std::string& shortname,
			       const int & sigId=-1 );
  

  void resetdg::Signals( const unsigned int & nbEntries,const unsigned int & nbdg::Signals );

 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );
    

};





#endif // #ifndef __SOT_SOTSELECTOR_H__

