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

#include "sot-pattern-generator/toes-handler.h"

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

using ::dynamicgraph::command::Setter;
using dynamicgraph::sot::FeatureAbstract;

#define UNDEF_VALUE -100000

namespace dynamicgraph {
  namespace sot {
    using command::Command;
    using command::Value;

    FeatureToesHandler::FeatureToesHandler (const std::string& name)
      : FeatureAbstract(name)
    , CONSTRUCT_SIGNAL_IN(onLeftToe, unsigned)
    , CONSTRUCT_SIGNAL_IN(onRightToe, unsigned)
    , CONSTRUCT_SIGNAL_IN(state, ml::Vector)
    , CONSTRUCT_SIGNAL_IN(dt,double)
    , prevLeftKnee_(UNDEF_VALUE)
    , prevRightKnee_(UNDEF_VALUE)
    , jacobian_()
    {
      signalRegistration (onLeftToeSIN << onRightToeSIN << stateSIN << dtSIN);
      errorSOUT.addDependency (onLeftToeSIN);
      errorSOUT.addDependency (onRightToeSIN);
      errorSOUT.addDependency (stateSIN);
      errorSOUT.addDependency (dtSIN);


      // TODO: yes, this is Romeo specific
      lKneeIndex_ = 28;
      lToeIndex_  = 31;

      rKneeIndex_ = 35;
      rToeIndex_  = 38;
    }

    FeatureToesHandler::~FeatureToesHandler ()
    {
    }

    unsigned int& FeatureToesHandler::getDimension( unsigned int& res,int )
    {
    	res = 2;
    	return res;
    }

    ml::Vector& FeatureToesHandler::computeError( ml::Vector& res, int )
    {
      // Check that dimensions of state and posture fit, otherwise, return 0
      const ml::Vector & state = stateSIN.accessCopy();
      unsigned onLeftToe = onLeftToeSIN.accessCopy();
      unsigned onRightToe = onRightToeSIN.accessCopy();
      const double & dt = dtSIN.accessCopy();

      unsigned int stateDim = state.size();

      if (res.size() != stateDim)
    	  res.resize(2);

      double currLeftKnee  = state(lKneeIndex_);
      if (prevLeftKnee_ == UNDEF_VALUE)
          prevLeftKnee_ = currLeftKnee;
      double dLeftKnee     = (currLeftKnee - prevLeftKnee_) / dt *0.1;
      prevLeftKnee_ = currLeftKnee;

      double currRightKnee = state(rKneeIndex_);
      if (prevRightKnee_ == UNDEF_VALUE)
    	  prevRightKnee_ = currRightKnee;
      double dRightKnee = (currRightKnee - prevRightKnee_) / dt *0.1  ;
      prevRightKnee_ = currRightKnee;


      // left foot
//     onLeftToe = 0;
//	  onRightToe = 0;
      res(0) = (onLeftToe  != 0 )? (-dLeftKnee) :  (state(lToeIndex_));
      res(1) = (onRightToe != 0 )? (-dRightKnee) : (state(rToeIndex_));

//      if (onLeftToe)
//    	  std::cout << "err " << res(0) << " real " << (currLeftKnee - prevLeftKnee_) << std::endl;
//      if (onRightToe)
//    	  std::cout << "err " << res(1) << " real " << (currLeftKnee - prevLeftKnee_) << std::endl;
      return res;
    }

    ml::Vector& FeatureToesHandler::computeErrorDot( ml::Vector& res, int )
    {
    	std::cerr << " calling FeatureToesHandler::computeErrorDot unexpected "<< std::endl;
    	return res;
    }

    ml::Matrix& FeatureToesHandler::computeJacobian( ml::Matrix& res, int time)
    {
      // Check that dimensions of state and posture fit, otherwise, return 0
        const ml::Vector & state = stateSIN.access(time);
        const unsigned & onLeftToe = onLeftToeSIN.access(time);
        const unsigned & onRightToe = onRightToeSIN.access(time);
        unsigned int stateDim = state.size();

      if (jacobian_.nbRows() != stateDim)
    	  jacobian_.resize(2, stateDim);

      jacobian_.fill(0.);

      // left toe on the ground
      //	  jacobian_(0, lKneeIndex_) = (onLeftToe != 0 )? 1 : 0; // minimize the velocity of LKneePitch_y
 	  jacobian_(0, lKneeIndex_) = (onLeftToe != 0 )? 0 : 0; // minimize the velocity of LKneePitch_y
	  jacobian_(0, lToeIndex_)  = (onLeftToe != 0 )? 0 : 1; // set LToePitch_y to 0

	  // right toe on the ground
	  //	  jacobian_(1, rKneeIndex_) = (onRightToe != 0 )? 1 : 0; // minimize the velocity of LKneePitch_y
	  jacobian_(1, rKneeIndex_) = (onRightToe != 0 )? 0 : 0; // minimize the velocity of LKneePitch_y
	  jacobian_(1, rToeIndex_)  = (onRightToe != 0 )? 0 : 1; // set LToePitch_y to 0

      res = jacobian_;

      return res;
    }

    ml::Vector& FeatureToesHandler::computeActivation( ml::Vector& res, int )
    {
      return res;
    }


    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureToesHandler, "FeatureToesHandler");
  } // namespace sot
} // namespace dynamicgraph

