/*
 * Copyright 2010,
 * Florent Lamiraux
 * Thomas Moulard,
 *
 * CNRS/AIST
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SOT_CORE_FEATURE_TOES_HANDLER_HH
#define SOT_CORE_FEATURE_TOES_HANDLER_HH

#include "sot/core/feature-abstract.hh"
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>


#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>

/* Pattern Generator */
#include <jrl/mal/matrixabstractlayer.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (toes_handler_zmp_EXPORTS)
#    define SOTTOESHANDLERZMP_EXPORT __declspec(dllexport)
#  else
#    define SOTTOESHANDLERZMP_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTTOESHANDLERZMP_EXPORT
#endif

namespace dynamicgraph {
  namespace sot {
    using command::Command;
    using command::Value;

    namespace ml = maal::boost;

    class SOTTOESHANDLERZMP_EXPORT FeatureToesHandlerZmp
    	: public FeatureAbstract
        , public ::dynamicgraph::EntityHelper<FeatureToesHandlerZmp>
    	, FeatureReferenceHelper<FeatureToesHandlerZmp>
    {
      DYNAMIC_GRAPH_ENTITY_DECL ();

      explicit FeatureToesHandlerZmp (const std::string& name);
      virtual ~FeatureToesHandlerZmp ();


      /*! \name Dealing with the reference value to be reach with this feature.
        @{
      */
      DECLARE_REFERENCE_FUNCTIONS(FeatureToesHandlerZmp);
      /*! @} */

    protected:

      virtual unsigned int& getDimension( unsigned int& res,int time );
      virtual ml::Vector& computeError( ml::Vector& res, int );
      virtual ml::Vector& computeErrorDot( ml::Vector& res, int );

      virtual ml::Matrix& computeJacobian( ml::Matrix& res, int );
      virtual ml::Vector& computeActivation( ml::Vector& res, int );

      DECLARE_SIGNAL_IN(leftToeOverlap, double);
      DECLARE_SIGNAL_IN(rightToeOverlap, double);
      DECLARE_SIGNAL_IN(state, ml::Vector);

    private:
      ml::Matrix jacobian_;
      ml::Vector error_;

      unsigned lToeIndex_;
      unsigned rToeIndex_;
    }; // class FeaturePosture
  } // namespace sot
} // namespace dynamicgraph

#endif //SOT_CORE_FEATURE_TOES_HANDLER_HH
