#include <cmath>

#include <sot-pattern-generator/step-observer.h>
#include <sot/core/matrix-geometry.hh>
#include <dynamic-graph/factory.h>
#include <sot/core/debug.hh>

namespace dynamicgraph {
  namespace sot {

    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(StepObserver,"StepObserver");

    StepObserver::StepObserver( const std::string & name )
      :Entity(name)

      ,leftHandPositionSIN( NULL,"StepObserver("+name+")::input(vector)::lefthand" )
      ,rightHandPositionSIN( NULL,"StepObserver("+name+")::input(vector)::righthand" )

      ,leftFootPositionSIN( NULL,"StepObserver("+name+")::input(matrixhomo)::leftfoot" )
      ,rightFootPositionSIN( NULL,"StepObserver("+name+")::input(matrixhomo)::rightfoot" )
      ,waistPositionSIN( NULL,"StepObserver("+name+")::input(matrixhomo)::waist" )

      ,referencePositionLeftSOUT( boost::bind(&StepObserver::computeReferencePositionLeft,this,_1,_2),
				  leftFootPositionSIN<<leftHandPositionSIN<<rightHandPositionSIN,
				  "StepObserver("+name+")::output(vector)::position2handLeft" )
      ,referencePositionRightSOUT( boost::bind(&StepObserver::computeReferencePositionRight,this,_1,_2),
				   rightFootPositionSIN<<rightHandPositionSIN<<leftHandPositionSIN,
				   "StepObserver("+name+")::output(vector)::position2handRight" )
      ,referencePositionWaistSOUT( boost::bind(&StepObserver::computeReferencePositionWaist,this,_1,_2),
				   waistPositionSIN<<rightHandPositionSIN<<leftHandPositionSIN,
				   "StepObserver("+name+")::output(vector)::position2handWaist" )
    {
      sotDEBUGIN(25);

      signalRegistration(getSignals());

      sotDEBUGOUT(25);
    }


    SignalArray<int> StepObserver::getSignals( void )
    {
      return (leftHandPositionSIN << leftFootPositionSIN << waistPositionSIN
	      << rightHandPositionSIN << rightFootPositionSIN
	      << referencePositionLeftSOUT << referencePositionRightSOUT
	      << referencePositionWaistSOUT );
    }


    StepObserver::operator SignalArray<int> ()
    {
      return getSignals();
    }


    MatrixHomogeneous&
    StepObserver::computeRefPos( MatrixHomogeneous& res,
				 int timeCurr,
				 const MatrixHomogeneous& wMref )
    {
      sotDEBUGIN(15);

      // Set to 0 to compute a reference frame using both hands. Set to non zero
      // value to use the right hand frame as a reference frame (for debug).
#define RIGHT_HAND_REFERENCE 0

#if RIGHT_HAND_REFERENCE

      const MatrixHomogeneous& wMrh = rightHandPositionSIN( timeCurr );
      MatrixHomogeneous refMw; refMw = wMref.inverse();
      res = refMw*wMrh;

#else

      const MatrixHomogeneous& wMlh = leftHandPositionSIN( timeCurr );
      const MatrixHomogeneous& wMrh = rightHandPositionSIN( timeCurr );

      MatrixHomogeneous refMw; refMw = wMref.inverse();
      MatrixHomogeneous sfMlh; sfMlh = refMw * wMlh;
      MatrixHomogeneous sfMrh; sfMrh  = refMw * wMrh;

      VectorRollPitchYaw rpy;

      Vector prh(3); prh = sfMrh.translation();

      Vector plh(3); plh = sfMlh.translation();

      rpy.setZero();
      rpy(2) = std::atan2(prh(0) - plh(0), plh(1) - prh(1));
      Vector p(3); p = .5 * (plh + prh);

      MatrixRotation R;

      R = (Eigen::AngleAxisd(rpy(2),Eigen::Vector3d::UnitZ())*
	   Eigen::AngleAxisd(rpy(1),Eigen::Vector3d::UnitY())*
	   Eigen::AngleAxisd(rpy(0),Eigen::Vector3d::UnitX())).toRotationMatrix();

      res.translation() = p;
      res.linear() = R;

#endif

      sotDEBUGOUT(15);
      return res;
    }


    MatrixHomogeneous&
    StepObserver::computeReferencePositionLeft( MatrixHomogeneous& res,
						int timeCurr )
    {
      sotDEBUGIN(15);

      const MatrixHomogeneous& wMref = leftFootPositionSIN( timeCurr );

      sotDEBUGOUT(15);
      return computeRefPos( res,timeCurr,wMref );
    }


    MatrixHomogeneous&
    StepObserver::computeReferencePositionRight( MatrixHomogeneous& res,
						 int timeCurr )
    {
      sotDEBUGIN(15);

      const MatrixHomogeneous& wMref = rightFootPositionSIN( timeCurr );

      sotDEBUGOUT(15);
      return computeRefPos( res,timeCurr,wMref );
    }


    MatrixHomogeneous&
    StepObserver::computeReferencePositionWaist( MatrixHomogeneous& res,
						 int timeCurr )
    {
      sotDEBUGIN(15);

      const MatrixHomogeneous& wMref = waistPositionSIN( timeCurr );

      sotDEBUGOUT(15);
      return computeRefPos( res,timeCurr,wMref );
    }


    void StepObserver::display( std::ostream& os ) const
    {
      os << "StepObserver <" << getName() <<">:" << std::endl;
    }


    void StepObserver::commandLine( const std::string& cmdLine,
				    std::istringstream& cmdArgs,
				    std::ostream& os )
    {
      if( cmdLine == "help" )
	{
	  os << "StepObserver: " << std::endl
	     << std::endl;
	}
      else { Entity::commandLine( cmdLine,cmdArgs,os); }
    }

  } // namespace dg
} // namespace sot
