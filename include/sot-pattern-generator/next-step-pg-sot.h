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

/* SOT */
#include <sot-pattern-generator/next-step.h>
#include <sot-pattern-generator/pg.h>

/* STD */
#include <string>
#include <deque>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (next_step_pg_sot_EXPORTS)
#    define NextStepPGSOT_EXPORT __declspec(dllexport)
#  else
#    define NextStepPGSOT_EXPORT __declspec(dllimport)
#  endif
#else
#  define NextStepPGSOT_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
  namespace sot {

    class NextStepPGSOT_EXPORT NextStepPgSot
      :public NextStep
    {
    public:
      DYNAMIC_GRAPH_ENTITY_DECL();
      static const unsigned int ADDING_STEP=0;
      static const unsigned int CHANGING_STEP=1;


    protected:

      typedef std::pair<unsigned int,
                        PatternGeneratorJRL::FootAbsolutePosition> FootPrint_t;
      std::vector<FootPrint_t> stepbuf;

      Entity * pgEntity;
      unsigned int m_StepModificationMode;
      double m_NextStepTime;
      unsigned int m_NbOfFirstSteps;

      /*! \brief Pointer towards the interface of the pattern generator. */
      pg::PatternGeneratorInterface * m_PGI;

      /*! \brief Pointer towards the entity 
        which handle the pattern generator. */
      PatternGenerator * m_sPG;

    public: /* --- CONSTRUCTION --- */
      NextStepPgSot( const std::string& name );
      virtual ~NextStepPgSot( void ) {}


    public: /* --- FUNCTIONS --- */

      virtual void starter( const int & timeCurr );
      virtual void stoper( const int & timeCurr );
      virtual void introductionCallBack( const int & timeCurr );

    public: /* --- ENTITY INHERITANCE --- */
      virtual void commandLine( const std::string& cmdLine,
                                std::istringstream& cmdArgs,
                                std::ostream& os );

    };


  } // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __SOT_NextStep_OHRP_H__

