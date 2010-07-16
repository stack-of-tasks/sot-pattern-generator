/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotStepComputer.cpp
 * Project:   SOT
 * Author:    Paul Evrard, Nicolas Mansard
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

#include <cmath>

#include <time.h>
#ifndef WIN32
# include <sys/time.h>
#else
# include <MatrixAbstractLayer/boost.h>
# include <sot-core/utils-windows.h>
# include <Winsock2.h>
#endif /*WIN32*/

#include <sot/sotStepComputerForce.h>
#include <sot/sotDebug.h>
#include <sot/sotMacrosSignal.h>
#include <sot/sotExceptionPatternGenerator.h>
#include <sot/sotStepQueue.h>
#include <sot/sotStepChecker.h>
#include <sot/sotFactory.h>
#include <sot/sotPool.h>
#include <sot/sotMatrixTwist.h>


SOT_FACTORY_ENTITY_PLUGIN(sotStepComputerForce,"StepComputerForce");


sotStepComputerForce:: sotStepComputerForce( const std::string & name )
  : Entity(name)
  , waistMlhandSIN( NULL,"sotStepComputerForce("+name+")::input(vector)::waistMlhand" )
  , waistMrhandSIN( NULL,"sotStepComputerForce("+name+")::input(vector)::waistMrhand" )
  , referencePositionWaistSIN( NULL,"sotStepComputerForce("+name+")::input(vector)::posrefwaist" )
  , stiffnessSIN( NULL,"sotStepComputerForce("+name+")::input(vector)::stiffness" )
  , velocitySIN( NULL,"sotStepComputerForce("+name+")::input(vector)::velocity" )
  , contactFootSIN( NULL,"sotStepComputerForce("+name+")::input(uint)::contactfoot" )
  , displacementSOUT( boost::bind(&sotStepComputerForce::computeDisplacement,this,_1,_2),
		      referencePositionWaistSIN,
		      "sotStepComputerForce("+name+")::output(vector)::displacement" )
  , forceSOUT( boost::bind(&sotStepComputerForce::computeForce,this,_1,_2),
	       displacementSOUT,
	       "sotStepComputerForce("+name+")::output(vector)::force" )
  , forceLhandSOUT( boost::bind(&sotStepComputerForce::computeForceL,this,_1,_2),
		    waistMlhandSIN<<referencePositionWaistSIN<<forceSOUT,
		    "sotStepComputerForce("+name+")::output(vector)::forceL" )
  , forceRhandSOUT( boost::bind(&sotStepComputerForce::computeForceR,this,_1,_2),
		    waistMrhandSIN<<referencePositionWaistSIN<<forceSOUT,
		    "sotStepComputerForce("+name+")::output(vector)::forceR" )
  , waMref0()
  , twoHandObserver( 0x0 )
  , checker()
  , logChanges("/tmp/stepcomp_changes.dat")
  , logPreview("/tmp/stepcomp_preview.dat")
{
  sotDEBUGIN(5);

  signalRegistration( referencePositionWaistSIN<<contactFootSIN <<
		      waistMlhandSIN << waistMrhandSIN <<
		      stiffnessSIN << velocitySIN <<
		      displacementSOUT << forceSOUT <<
		      forceLhandSOUT << forceRhandSOUT );

  sotDEBUGOUT(5);
}

void sotStepComputerForce::nextStep( sotStepQueue& queue, int timeCurr )
{
  // Introduce new step at the end of the preview window.
  if( queue.getLastStep().contact == CONTACT_LEFT_FOOT ) {
    queue.pushStep(0., -queue.getZeroStepPosition(), 0.);
    logPreview << timeCurr << " " << 0 << " "
	       << -queue.getZeroStepPosition() << " " << 0
	       << std::endl;
  }
  else {
    queue.pushStep(0., queue.getZeroStepPosition(), 0.);
    logPreview << timeCurr << " " << 0 << " "
	       << queue.getZeroStepPosition() << " " << 0
	       << std::endl;
  }
}

ml::Vector& sotStepComputerForce::computeDisplacement( ml::Vector& res,int timeCurr )
{
  if(!twoHandObserver) {
    std::cerr << "Observer not set" << std::endl;
    res.resize(3);
    res.fill(0.);
    return res;
  }

  // transformation from ref0 to ref.

  const MatrixHomogeneous& waMref = referencePositionWaistSIN.access( timeCurr );
  MatrixHomogeneous ref0Mwa; waMref0.inverse(ref0Mwa);
  MatrixHomogeneous ref0Mref; ref0Mwa.multiply(waMref, ref0Mref);

  // extract the translation part and express it in the waist frame.

  ml::Vector t_ref0(3); ref0Mref.extract(t_ref0);
  sotMatrixRotation waRref0; waMref0.extract(waRref0);
  ml::Vector t_wa = waRref0.multiply(t_ref0);

  // compute the rotation that transforms ref0 into ref,
  // express it in the waist frame. Then get the associated
  // yaw (rot around z).

  sotMatrixRotation ref0Rwa; waRref0.transpose(ref0Rwa);
  sotMatrixRotation ref0Rref; ref0Mref.extract(ref0Rref);
  sotMatrixRotation tmp; ref0Rref.multiply(ref0Rwa, tmp);
  sotMatrixRotation Rref; waRref0.multiply(tmp, Rref);
  VectorRollPitchYaw rpy; rpy.fromMatrix(Rref);

  // store the result.

  res.resize(3);
  res = t_wa;
  res(2) = rpy(2);

  return res;
}

ml::Vector& sotStepComputerForce::computeForce( ml::Vector& res,int timeCurr )
{
  const ml::Vector& dx = displacementSOUT.access( timeCurr );
  const ml::Vector& K = stiffnessSIN.access( timeCurr );

  if((dx.size() != 3) || (K.size() != 3) || (dx.size() != K.size())) {
    res.resize(3);
    res.fill(0.);
  }
  else {
    res = K.multiply(dx);
  }

  return res;
}

ml::Vector&
sotStepComputerForce::computeHandForce( ml::Vector& res,
					const MatrixHomogeneous& waMh,
					const MatrixHomogeneous& waMref,
					const ml::Vector& F )
{
  if(F.size() != 3) {
    res.resize(6);
    res.fill(0.);
    return res;
  }

  ml::Vector pref(3); waMref.extract(pref);
  ml::Vector ph(3); waMh.extract(ph);

  ml::Vector OA(3);
  OA(0) = ph(0) - pref(0);
  OA(1) = ph(1) - pref(1);
  OA(2) = 0;

  ml::Vector tau(3); tau.fill(0.);
  tau(2) = -F(2);

  ml::Vector tauOA = tau.crossProduct(OA);
  double ntauOA = tauOA.norm();
  double nOA = OA.norm();
  double L = 2 * ntauOA * nOA;

  if(L < 1e-12) {
    tauOA.fill(0);
  }
  else {
    tauOA = tauOA.multiply(1./L);
  }

  ml::Vector tmp(6); tmp.fill(0.);
  tmp.resize(6); tmp.fill(0.);
  tmp(0) = tauOA(0) - F(0);
  tmp(1) = tauOA(1) - F(1);

  MatrixHomogeneous H; waMh.inverse(H);
  for(int i = 0; i < 3; ++i){ H(i,3) = 0; }
  sotMatrixTwist V; V.buildFrom(H);
  V.multiply(tmp, res);

  return res;
}
						    
ml::Vector& sotStepComputerForce::computeForceL( ml::Vector& res,int timeCurr )
{
  const MatrixHomogeneous& waMlh = waistMlhandSIN.access( timeCurr );
  const MatrixHomogeneous& waMref = referencePositionWaistSIN.access( timeCurr );
  const ml::Vector& F = forceSOUT.access( timeCurr );

  return computeHandForce( res,waMlh,waMref,F );
}

ml::Vector& sotStepComputerForce::computeForceR( ml::Vector& res,int timeCurr )
{
  const MatrixHomogeneous& waMrh = waistMrhandSIN.access( timeCurr );
  const MatrixHomogeneous& waMref = referencePositionWaistSIN.access( timeCurr );
  const ml::Vector& F = forceSOUT.access( timeCurr );

  return computeHandForce( res,waMrh,waMref,F );
}

void sotStepComputerForce::changeFirstStep( sotStepQueue& queue, int timeCurr )
{
  const ml::Vector& v = velocitySIN.access( timeCurr );
  unsigned sfoot = contactFootSIN.access( timeCurr );

  double y_default = 0;
  if( sfoot != 1 ) { // --- left foot support ---
    y_default = -0.19;
  }
  else { // -- right foot support ---
    y_default = 0.19;
  }

  // The clipping function expects the x-y coordinates of the
  // destination fly foot in the support foot frame.

  double x = v(0), y = v(1);
  double theta = v(2) * 180 / 3.14159265;

  if(std::abs(x) < 0.03){ x = 0; }
  if(std::abs(y) < 0.03){ y = 0; }
  if(std::abs(theta) < 2){ theta = 0; }

  y += y_default;

  const double THETA_MAX = 9.;
  if(theta < -THETA_MAX){ theta = -THETA_MAX; }
  if(theta > THETA_MAX){ theta = THETA_MAX; }

  double nx = 0, ny = 0;
  if(sfoot != 1) { // left foot support phase
    if(y > 0){ y = -0.001; }
  }
  else {
    if(y < 0){ y = 0.001; }
  }

  checker.clipStep(x, y, nx, ny);

  // Log x-y values before and after clipping

  logChanges << timeCurr << " " << x << " " << y << " " << nx << " " << ny << " ";

  // The coordinates must be expressed in the destination foot frame.
  // See the technical report of Olivier Stasse for more details,
  // on top of page 79.

  double theta_rad = 3.14159265 * theta / 180.;
  double ctheta = cos(theta_rad);
  double stheta = sin(theta_rad);

  x = nx * ctheta + ny * stheta;
  y = -nx * stheta + ny * ctheta;

  queue.changeFirstStep(x, y, theta);

  // Log the step

  logChanges << x << " " << y << " " << theta << std::endl;
}


void sotStepComputerForce::thisIsZero()
{
  sotDEBUGIN(15);

  waMref0 = referencePositionWaistSIN.accessCopy();

  sotDEBUGOUT(15);
}


void sotStepComputerForce::display( std::ostream& os ) const
{
  os << "sotStepComputerForce <" << getName() <<">:" << std::endl;
}


void sotStepComputerForce::commandLine( const std::string& cmdLine,
				   std::istringstream& cmdArgs,
				   std::ostream& os )
{
  if( cmdLine == "help" )
  {
    os << "NextStep: " << std::endl
       << " - setObserver" << std::endl
       << " - thisIsZero {record|disp}" << std::endl
       << std::endl;
  }
  else if( cmdLine == "thisIsZero" )
  {
    std::string arg; cmdArgs >> arg; 
    if( arg == "disp" ) { os << "zero = " << waMref0; }
    else if( arg == "record" ) { thisIsZero(); }
  }
  else if( cmdLine == "setObserver" )
  {
    std::string name = "stepobs";
    cmdArgs >> std::ws;
    if( cmdArgs.good()){ cmdArgs >> name; }
    Entity* entity = &sotPool.getEntity( name );
    twoHandObserver = dynamic_cast<sotStepObserver*>(entity);
  }
  else { Entity::commandLine( cmdLine,cmdArgs,os); }
}

