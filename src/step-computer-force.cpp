/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      StepComputer.cpp
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
# include <sot/core/utils-windows.hh>
# include <Winsock2.h>
#endif /*WIN32*/

#include <sot-pattern-generator/step-computer-force.h>
#include <sot/core/debug.hh>
#include <sot/core/macros-signal.hh>
#include <sot-pattern-generator/exception-pg.h>
#include <sot-pattern-generator/step-queue.h>
#include <sot-pattern-generator/step-checker.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>
#include <sot/core/matrix-twist.hh>

namespace dynamicgraph {
  namespace sot {

    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(StepComputerForce,"StepComputerForce");

    StepComputerForce:: StepComputerForce( const std::string & name )
      : Entity(name)
      , waistMlhandSIN( NULL,"StepComputerForce("+name+")::input(vector)::waistMlhand" )
      , waistMrhandSIN( NULL,"StepComputerForce("+name+")::input(vector)::waistMrhand" )
      , referencePositionWaistSIN( NULL,"StepComputerForce("+name+")::input(vector)::posrefwaist" )
      , stiffnessSIN( NULL,"StepComputerForce("+name+")::input(vector)::stiffness" )
      , velocitySIN( NULL,"StepComputerForce("+name+")::input(vector)::velocity" )
      , contactFootSIN( NULL,"StepComputerForce("+name+")::input(uint)::contactfoot" )
      , displacementSOUT( boost::bind(&StepComputerForce::computeDisplacement,this,_1,_2),
			  referencePositionWaistSIN,
			  "StepComputerForce("+name+")::output(vector)::displacement" )
      , forceSOUT( boost::bind(&StepComputerForce::computeForce,this,_1,_2),
		   displacementSOUT,
		   "StepComputerForce("+name+")::output(vector)::force" )
      , forceLhandSOUT( boost::bind(&StepComputerForce::computeForceL,this,_1,_2),
			waistMlhandSIN<<referencePositionWaistSIN<<forceSOUT,
			"StepComputerForce("+name+")::output(vector)::forceL" )
      , forceRhandSOUT( boost::bind(&StepComputerForce::computeForceR,this,_1,_2),
			waistMrhandSIN<<referencePositionWaistSIN<<forceSOUT,
			"StepComputerForce("+name+")::output(vector)::forceR" )
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

    void StepComputerForce::nextStep( StepQueue& queue, int timeCurr )
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

    dynamicgraph::Vector& StepComputerForce::computeDisplacement( dynamicgraph::Vector& res,int timeCurr )
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
      MatrixHomogeneous ref0Mref; ref0Mref = ref0Mwa*waMref;

      // extract the translation part and express it in the waist frame.

      dynamicgraph::Vector t_ref0(3); ref0Mref.extract(t_ref0);
      MatrixRotation waRref0; waMref0.extract(waRref0);
      dynamicgraph::Vector t_wa; t_wa = waRref0*t_ref0;

      // compute the rotation that transforms ref0 into ref,
      // express it in the waist frame. Then get the associated
      // yaw (rot around z).

      MatrixRotation ref0Rwa; ref0Rwa = waRref0.transpose();
      MatrixRotation ref0Rref; ref0Mref.extract(ref0Rref);
      MatrixRotation tmp; tmp = ref0Rref*ref0Rwa;
      MatrixRotation Rref; Rref = waRref0*tmp;
      VectorRollPitchYaw rpy; rpy.fromMatrix(Rref);

      // store the result.

      res.resize(3);
      res = t_wa;
      res(2) = rpy(2);

      return res;
    }

    dynamicgraph::Vector& StepComputerForce::computeForce( dynamicgraph::Vector& res,int timeCurr )
    {
      const dynamicgraph::Vector& dx = displacementSOUT.access( timeCurr );
      const dynamicgraph::Vector& K = stiffnessSIN.access( timeCurr );

      if((dx.size() != 3) || (K.size() != 3) || (dx.size() != K.size())) {
	res.resize(3);
	res.fill(0.);
      }
      else {
	res = K*dx;
      }

      return res;
    }

    dynamicgraph::Vector&
    StepComputerForce::computeHandForce( dynamicgraph::Vector& res,
					 const MatrixHomogeneous& waMh,
					 const MatrixHomogeneous& waMref,
					 const dynamicgraph::Vector& F )
    {
      if(F.size() != 3) {
	res.resize(6);
	res.fill(0.);
	return res;
      }

      dynamicgraph::Vector pref(3); waMref.extract(pref);
      dynamicgraph::Vector ph(3); waMh.extract(ph);

      dynamicgraph::Vector OA(3);
      OA(0) = ph(0) - pref(0);
      OA(1) = ph(1) - pref(1);
      OA(2) = 0;

      dynamicgraph::Vector tau(3); tau.fill(0.);
      tau(2) = -F(2);

      dynamicgraph::Vector tauOA(3);// = tau.cross(OA);
      tauOA(0) = tau(1)*OA(2) - tau(2)*OA(1);
      tauOA(1) = tau(2)*OA(0) - tau(0)*OA(2);
      tauOA(2) = tau(0)*OA(1) - tau(1)*OA(0);
      double ntauOA = tauOA.norm();
      double nOA = OA.norm();
      double L = 2 * ntauOA * nOA;

      if(L < 1e-12) {
	tauOA.fill(0);
      }
      else {
	tauOA = tauOA*(1./L);
      }

      dynamicgraph::Vector tmp(6); tmp.fill(0.);
      tmp.resize(6); tmp.fill(0.);
      tmp(0) = tauOA(0) - F(0);
      tmp(1) = tauOA(1) - F(1);

      MatrixHomogeneous H; waMh.inverse(H);
      for(int i = 0; i < 3; ++i){ H(i,3) = 0; }
      MatrixTwist V; V.buildFrom(H);
      res = V*tmp;

      return res;
    }

    dynamicgraph::Vector& StepComputerForce::computeForceL( dynamicgraph::Vector& res,int timeCurr )
    {
      const MatrixHomogeneous& waMlh = waistMlhandSIN.access( timeCurr );
      const MatrixHomogeneous& waMref = referencePositionWaistSIN.access( timeCurr );
      const dynamicgraph::Vector& F = forceSOUT.access( timeCurr );

      return computeHandForce( res,waMlh,waMref,F );
    }

    dynamicgraph::Vector& StepComputerForce::computeForceR( dynamicgraph::Vector& res,int timeCurr )
    {
      const MatrixHomogeneous& waMrh = waistMrhandSIN.access( timeCurr );
      const MatrixHomogeneous& waMref = referencePositionWaistSIN.access( timeCurr );
      const dynamicgraph::Vector& F = forceSOUT.access( timeCurr );

      return computeHandForce( res,waMrh,waMref,F );
    }

    void StepComputerForce::changeFirstStep( StepQueue& queue, int timeCurr )
    {
      const dynamicgraph::Vector& v = velocitySIN.access( timeCurr );
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


    void StepComputerForce::thisIsZero()
    {
      sotDEBUGIN(15);

      waMref0 = referencePositionWaistSIN.accessCopy();

      sotDEBUGOUT(15);
    }


    void StepComputerForce::display( std::ostream& os ) const
    {
      os << "StepComputerForce <" << getName() <<">:" << std::endl;
    }


    void StepComputerForce::commandLine( const std::string& cmdLine,
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
	  Entity* entity = &(PoolStorage::getInstance()->getEntity( name ));
	  twoHandObserver = dynamic_cast<StepObserver*>(entity);
	}
      else { Entity::commandLine( cmdLine,cmdArgs,os); }
    }

  } // namespace dg
} // namespace sot
