/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotPGManager.h
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
 
#ifndef __SOT_PG_MANAGER_H__
#define __SOT_PG_MANAGER_H__


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
#include <sot/sotStepQueue.h>
#include <sot/PatternGenerator.h>


/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (sotPGManager_EXPORTS) 
#    define SOTPGMANAGER_EXPORT __declspec(dllexport)
#  else  
#    define SOTPGMANAGER_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTPGMANAGER_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class sotStepQueue;
class PatternGenerator;

/// Finds the PG and sends steps to the PG.
class SOTPGMANAGER_EXPORT sotPGManager
  : public dg::Entity
{
public: // dg::Entity name

  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

public: // Construction

  sotPGManager( const std::string& name );

  void startSequence( const sotStepQueue& seq );
  void stopSequence( const sotStepQueue& seq );
  void introduceStep( sotStepQueue& queue );
  double changeNextStep( sotStepQueue& queue );

public: // dg::Entity

  virtual void display( std::ostream& os ) const; 
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );

private:

  std::vector<sotFootPrint> stepbuf;
  dg::Entity* pgdg::Entity;
  pg::PatternGeneratorInterface* pgi;
};

#endif






