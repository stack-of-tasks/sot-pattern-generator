/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      Selector.h
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



#ifndef __SOT_Selector_H__
#define __SOT_Selector_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
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
#  if defined (selector_EXPORTS)
#    define Selector_EXPORT __declspec(dllexport)
#  else  
#    define Selector_EXPORT __declspec(dllimport)
#  endif 
#else
#  define Selector_EXPORT
#endif

namespace sot{
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class Selector_EXPORT Selector
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;

 protected:
  /** Number of signal type. For each signal type, you have
   * one output and <nbEntries> inputs. */
  unsigned int nbSignals;
  /** Number of possible values for the selector. For each entry,
   * you have one signal of each type. */
  unsigned int nbEntries;

 public: /* --- CONSTRUCTION --- */

  Selector( const std::string& name );
  virtual ~Selector( void );

 public: /* --- SIGNAL --- */

  dg::SignalPtr<unsigned int,int> selectorSIN; 

  std::vector< std::vector<dg::SignalBase<int>* > > inputsSIN;
  std::vector< dg::SignalBase<int>* > outputsSOUT;


 public: /* --- FUNCTIONS --- */

  template< class T >
    static T& computeSelection( const unsigned int & sigNum,
				std::vector< dg::SignalBase<int>* >& entriesSIN,
				T& res,const int& time );

  template< class T >
    unsigned int createSignal( const std::string& shortname,
			       const int & sigId=-1 );
  

  void resetSignals( const unsigned int & nbEntries,const unsigned int & nbSignals );

 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );
    

};


} // namespace sot


#endif // #ifndef __SOT_Selector_H__

