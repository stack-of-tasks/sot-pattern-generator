// Copyright 2010, François Bleibel, Thomas Moulard, Olivier Stasse,
// JRL, CNRS/AIST.
//
// This file is part of dynamic-graph.
// dynamic-graph is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// dynamic-graph is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

#include <string>
#include <boost/assign/list_of.hpp>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>

#include "sot-pattern-generator/toes-handler-zmp.h"

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

using ::dynamicgraph::command::Setter;
using dynamicgraph::sot::FeatureAbstract;

#define UNDEF_VALUE -100000

namespace dynamicgraph {
  namespace sot {
    using command::Command;
    using command::Value;

    FeatureToesHandlerZmp::FeatureToesHandlerZmp (const std::string& name)
      : FeatureAbstract(name)
    , CONSTRUCT_SIGNAL_IN(leftToeOverlap,  double)
    , CONSTRUCT_SIGNAL_IN(rightToeOverlap, double)
    , CONSTRUCT_SIGNAL_IN(state, ml::Vector)
    , jacobian_()
    {
      signalRegistration (leftToeOverlapSIN << rightToeOverlapSIN << stateSIN);
      errorSOUT.addDependency (leftToeOverlapSIN);
      errorSOUT.addDependency (rightToeOverlapSIN);
      errorSOUT.addDependency (stateSIN);

      // TODO: yes, this is Romeo specific
      lToeIndex_  = 31;
      rToeIndex_  = 38;

      error_.resize(2);
    }

    FeatureToesHandlerZmp::~FeatureToesHandlerZmp ()
    {
    }

    unsigned int& FeatureToesHandlerZmp::getDimension( unsigned int& res, int )
    {
    	res = 2;
    	return res;
    }

    ml::Vector& FeatureToesHandlerZmp::computeError( ml::Vector& res, int time)
    {
        // Check that dimensions of state and posture fit, otherwise, return 0
        const ml::Vector & state = stateSIN.access(time);
        unsigned int stateDim = state.size();

        if (jacobian_.nbRows() != stateDim)
        {
        	jacobian_.resize(2, stateDim);
            jacobian_.fill(0.);

			jacobian_(0, lToeIndex_)  = 1;
			jacobian_(1, rToeIndex_)  = 1;
        }


		// Check that dimensions of state and posture fit, otherwise, return 0
		double leftToeOverlap  = leftToeOverlapSIN(time);
		leftToeOverlap  = std::max(leftToeOverlap, 0.);
		double rightToeOverlap = rightToeOverlapSIN(time);
		rightToeOverlap  = std::max(rightToeOverlap, 0.);

		// TODO hard coded info (yuk)
		double minToeAngle = -0.7853;
		double toeLenght   = 0.0885;
		//end of hard coded methods

		// handling LToe
		{
		double toeAngle = std::min(std::max(0., leftToeOverlap/toeLenght), 1.) * minToeAngle *0.9;
		error_(0) = state(lToeIndex_) - toeAngle;
		}

		// handling RToe
		{
		double toeAngle = std::min(std::max(0.,rightToeOverlap/toeLenght), 1.) * minToeAngle *0.9;
		error_(1) = state(rToeIndex_) - toeAngle;
		}

		res = error_;

      return res;
    }

    ml::Vector& FeatureToesHandlerZmp::computeErrorDot( ml::Vector& res, int )
    {
    	std::cerr << " calling FeatureToesHandlerZmp::computeErrorDot unexpected "<< std::endl;
    	return res;
    }

    ml::Matrix& FeatureToesHandlerZmp::computeJacobian( ml::Matrix& res, int )
    {
      res = jacobian_;

      return res;
    }

    ml::Vector& FeatureToesHandlerZmp::computeActivation( ml::Vector& res, int )
    {
      return res;
    }
    /* --------------------------------------------------------------------- */
    /* --------------------------------------------------------------------- */
    /* --------------------------------------------------------------------- */

    void FeatureToesHandlerZmp::addDependenciesFromReference( void ){}
    void FeatureToesHandlerZmp::removeDependenciesFromReference( void ){}

    /* --------------------------------------------------------------------- */
    /* --------------------------------------------------------------------- */
    /* --------------------------------------------------------------------- */

    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureToesHandlerZmp, "FeatureToesHandlerZmp");
  } // namespace sot
} // namespace dynamicgraph

