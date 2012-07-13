/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      PGManager.h
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
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot-pattern-generator/step-queue.h>
#include <sot-pattern-generator/pg.h>


/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (pg_manager_EXPORTS)
#    define PGManager_EXPORT __declspec(dllexport)
#  else
#    define PGManager_EXPORT __declspec(dllimport)
#  endif
#else
#  define PGManager_EXPORT
#endif


namespace dynamicgraph {
  namespace sot {

    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */

    class StepQueue;
    class PatternGenerator;

    /// Finds the PG and sends steps to the PG.
    class PGManager_EXPORT PGManager
      : public Entity
    {
    public: // Entity name
      DYNAMIC_GRAPH_ENTITY_DECL();

    public: // Construction

      PGManager( const std::string& name );

      void startSequence( const StepQueue& seq );
      void stopSequence( const StepQueue& seq );
      void introduceStep( StepQueue& queue );
      double changeNextStep( StepQueue& queue );

    public: // Entity

      virtual void display( std::ostream& os ) const;
      virtual void commandLine( const std::string& cmdLine,
				std::istringstream& cmdArgs,
				std::ostream& os );

    private:

      std::vector<FootPrint> stepbuf;
      Entity* pgEntity;
      pg::PatternGeneratorInterface* pgi;
    };

  } // namespace sot
} // namespace dg

#endif

