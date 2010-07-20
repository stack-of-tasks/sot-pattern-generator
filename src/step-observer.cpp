#include <cmath>

#include <sot-pattern-generator/step-observer.h>
#include <sot-core/vector-roll-pitch-yaw.h>
#include <sot-core/matrix-rotation.h>
#include <dynamic-graph/factory.h>
#include <sot-core/debug.h>

using namespace sot;
using namespace dynamicgraph;
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
  MatrixHomogeneous refMw; wMref.inverse(refMw);
  refMw.multiply(wMrh, res);

#else

  const MatrixHomogeneous& wMlh = leftHandPositionSIN( timeCurr );
  const MatrixHomogeneous& wMrh = rightHandPositionSIN( timeCurr );

  MatrixHomogeneous refMw; wMref.inverse(refMw);
  MatrixHomogeneous sfMlh; refMw.multiply(wMlh, sfMlh);
  MatrixHomogeneous sfMrh; refMw.multiply(wMrh, sfMrh);

  MatrixRotation R;
  VectorRollPitchYaw rpy;

  ml::Vector prh(3); sfMrh.extract(prh);
  sfMrh.extract(R);
  VectorRollPitchYaw rpy_rh; rpy_rh.fromMatrix(R);

  ml::Vector plh(3); sfMlh.extract(plh);
  sfMlh.extract(R);
  VectorRollPitchYaw rpy_lh; rpy_lh.fromMatrix(R);

  rpy.fill(0.);
  rpy(2) = std::atan2(prh(0) - plh(0), plh(1) - prh(1));
  ml::Vector p = .5 * (plh + prh);

  rpy.toMatrix(R);
  res.buildFrom(R, p);

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

