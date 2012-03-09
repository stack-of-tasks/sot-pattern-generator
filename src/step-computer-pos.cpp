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
# include <jrl/mal/boost.hh>
# include <sot/core/utils-windows.hh>
# include <Winsock2.h>
#endif /*WIN32*/

#include <sot-pattern-generator/step-computer-pos.h>
#include <sot/core/debug.hh>
#include <sot/core/macros-signal.hh>
#include <sot-pattern-generator/exception-pg.h>
#include <sot-pattern-generator/step-queue.h>
#include <sot-pattern-generator/step-checker.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>

namespace dynamicgraph {
  namespace sot {

    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(StepComputerPos,"StepComputerPos");

    StepComputerPos:: StepComputerPos( const std::string & name )
      : Entity(name)
      , referencePositionLeftSIN( NULL,"StepComputerPos("+name+")::input(vector)::posrefleft" )
      , referencePositionRightSIN( NULL,"StepComputerPos("+name+")::input(vector)::posrefright" )
      , contactFootSIN( NULL,"StepComputerPos("+name+")::input(uint)::contactfoot" )
      , rfMref0()
      , lfMref0()
      , twoHandObserver( 0x0 )
      , checker()
      , logChanges("/tmp/stepcomp_changes.dat")
      , logPreview("/tmp/stepcomp_preview.dat")
    {
      sotDEBUGIN(5);

      signalRegistration( referencePositionLeftSIN<<referencePositionRightSIN<<contactFootSIN );

      sotDEBUGOUT(5);
    }

    void StepComputerPos::nextStep( StepQueue& queue, int timeCurr )
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

    void StepComputerPos::changeFirstStep( StepQueue& queue, int timeCurr )
    {
      if(!twoHandObserver) {
	std::cerr << "Observer not set" << std::endl;
	return;
      }

      const unsigned& sfoot = contactFootSIN( timeCurr );
      const MatrixHomogeneous& wMlf = twoHandObserver->leftFootPositionSIN.access( timeCurr );
      const MatrixHomogeneous& wMrf = twoHandObserver->rightFootPositionSIN.access( timeCurr );

      // actual and reference position of reference frame in fly foot,
      // position of fly foot in support foot.

      MatrixHomogeneous ffMref, ffMref0;
      MatrixHomogeneous sfMff;
      if( sfoot != 1 ) // --- left foot support ---
	{
	  ffMref = referencePositionRightSIN.access( timeCurr );
	  ffMref0 = rfMref0;
	  MatrixHomogeneous sfMw; wMlf.inverse(sfMw); sfMw.multiply(wMrf, sfMff);
	}
      else // -- right foot support ---
	{
	  ffMref = referencePositionLeftSIN.access( timeCurr );
	  ffMref0 = lfMref0;
	  MatrixHomogeneous sfMw; wMrf.inverse(sfMw); sfMw.multiply(wMlf, sfMff);
	}

      // homogeneous transform from ref position of ref frame to
      // actual position of ref frame.

      MatrixHomogeneous ref0Mff; ffMref0.inverse(ref0Mff);
      MatrixHomogeneous ref0Mref; ref0Mff.multiply(ffMref, ref0Mref);

      // extract the translation part and express it in the support
      // foot frame.

      MatrixHomogeneous sfMref0; sfMff.multiply(ffMref0, sfMref0);
      ml::Vector t_ref0(3); ref0Mref.extract(t_ref0);
      MatrixRotation sfRref0; sfMref0.extract(sfRref0);
      ml::Vector t_sf = sfRref0.multiply(t_ref0);

      // add it to the position of the fly foot in support foot to
      // get the new position of fly foot in support foot.

      ml::Vector pff_sf(3); sfMff.extract(pff_sf);
      t_sf += pff_sf;

      // compute the rotation that transforms ref0 into ref,
      // express it in the support foot frame. Then get the
      // associated yaw (rot around z).

      MatrixRotation ref0Rsf; sfRref0.transpose(ref0Rsf);
      MatrixRotation ref0Rref; ref0Mref.extract(ref0Rref);
      MatrixRotation tmp; ref0Rref.multiply(ref0Rsf, tmp);
      MatrixRotation Rref; sfRref0.multiply(tmp, Rref);
      VectorRollPitchYaw rpy; rpy.fromMatrix(Rref);

      // get the yaw of the current orientation of the ff wrt sf.
      // Add it to the previously computed rpy.

      MatrixRotation sfRff; sfMff.extract(sfRff);
      VectorRollPitchYaw rpy_ff; rpy_ff.fromMatrix(sfRff);
      rpy += rpy_ff;

      // The clipping function expects the x-y coordinates of the
      // destination fly foot in the support foot frame.

      double x = t_sf(0), y = t_sf(1);
      double theta = rpy(2) * 180 / 3.14159265;

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


    void StepComputerPos::thisIsZero()
    {
      sotDEBUGIN(15);

      rfMref0 = referencePositionRightSIN.accessCopy();
      lfMref0 = referencePositionLeftSIN.accessCopy();

      sotDEBUGOUT(15);
    }


    void StepComputerPos::display( std::ostream& os ) const
    {
      os << "StepComputerPos <" << getName() <<">:" << std::endl;
    }


    void StepComputerPos::commandLine( const std::string& cmdLine,
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
	  if( arg == "disp_left" ) { os << "zero_left = " << lfMref0; }
	  else if( arg == "disp_right" ) { os << "zero_right = " << rfMref0; }
	  else if( arg == "record" ) { thisIsZero(); }
	}
      else if( cmdLine == "setObserver" )
	{
	  std::string name = "stepobs";
	  cmdArgs >> std::ws;
	  if( cmdArgs.good()){ cmdArgs >> name; }
	  Entity* entity = &g_pool().getEntity( name );
	  twoHandObserver = dynamic_cast<StepObserver*>(entity);
	}
      else { Entity::commandLine( cmdLine,cmdArgs,os); }
    }

  } // namespace dg
} // namespace sot
