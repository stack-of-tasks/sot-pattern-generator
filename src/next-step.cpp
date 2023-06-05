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

#include <sot/pattern-generator/next-step.h>
#include <time.h>

#include <cmath>
#include <sot/core/debug.hh>
#ifndef WIN32
#include <sys/time.h>
#else
#include <Winsock2.h>

#include <sot/core/utils-windows.hh>
#endif /*WIN32*/

#include <dynamic-graph/factory.h>

#include <sot/core/macros-signal.hh>

namespace dynamicgraph {
namespace sot {

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(NextStep, "NextStep");

/* --- CONSTRUCT -------------------------------------------------------- */
/* --- CONSTRUCT -------------------------------------------------------- */
/* --- CONSTRUCT -------------------------------------------------------- */

const unsigned int NextStep::PERIOD_DEFAULT = 160;  // 160iter=800ms
const double NextStep::ZERO_STEP_POSITION_DEFAULT = 0.19;

NextStep::NextStep(const std::string &name)
    : Entity(name)

      ,
      footPrintList(),
      period(PERIOD_DEFAULT),
      timeLastIntroduction(0)

      ,
      mode(MODE_3D),
      state(STATE_STOPED)

      ,
      zeroStepPosition(ZERO_STEP_POSITION_DEFAULT)

      ,
      rfMref0(),
      lfMref0(),
      twoHandObserver(name)

      ,
      verbose(0x0)

      ,
      referencePositionLeftSIN(
          NULL, "NextStep(" + name + ")::input(vector)::posrefleft"),
      referencePositionRightSIN(
          NULL, "NextStep(" + name + ")::input(vector)::posrefright")

      ,
      contactFootSIN(NULL, "NextStep(" + name + ")::input(uint)::contactfoot"),
      triggerSOUT("NextStep(" + name + ")::input(dummy)::trigger") {
  sotDEBUGIN(5);

  triggerSOUT.setFunction(boost::bind(&NextStep::triggerCall, this, _1, _2));

  signalRegistration(referencePositionLeftSIN << referencePositionRightSIN
                                              << contactFootSIN << triggerSOUT);
  signalRegistration(twoHandObserver.getSignals());

  referencePositionLeftSIN.plug(&twoHandObserver.referencePositionLeftSOUT);
  referencePositionRightSIN.plug(&twoHandObserver.referencePositionRightSOUT);

  sotDEBUGOUT(5);
}

NextStep::~NextStep(void) {
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
  return;
}

/* --- FUNCTIONS ------------------------------------------------------- */
/* --- FUNCTIONS ------------------------------------------------------- */
/* --- FUNCTIONS ------------------------------------------------------- */

void NextStep::nextStep(const int &timeCurr) {
  sotDEBUGIN(15);

  const unsigned &sfoot = contactFootSIN(timeCurr);
  const MatrixHomogeneous &wMlf =
      twoHandObserver.leftFootPositionSIN.access(timeCurr);
  const MatrixHomogeneous &wMrf =
      twoHandObserver.rightFootPositionSIN.access(timeCurr);

  // actual and reference position of reference frame in fly foot,
  // position of fly foot in support foot.

  MatrixHomogeneous ffMref, ffMref0;
  MatrixHomogeneous sfMff;
  if (sfoot != 1)  // --- left foot support ---
  {
    ffMref = referencePositionRightSIN.access(timeCurr);
    ffMref0 = rfMref0;
    MatrixHomogeneous sfMw;
    sfMw = wMlf.inverse();
    sfMff = sfMw * wMrf;
  } else  // -- right foot support ---
  {
    ffMref = referencePositionLeftSIN.access(timeCurr);
    ffMref0 = lfMref0;
    MatrixHomogeneous sfMw;
    sfMw = wMrf.inverse();
    sfMff = sfMw * wMlf;
  }

  // homogeneous transform from ref position of ref frame to
  // actual position of ref frame.

  MatrixHomogeneous ref0Mff;
  ref0Mff = ffMref0.inverse();
  MatrixHomogeneous ref0Mref;
  ref0Mref = ref0Mff * ffMref;

  // extract the translation part and express it in the support
  // foot frame.

  MatrixHomogeneous sfMref0;
  sfMref0 = sfMff * ffMref0;
  Vector t_ref0(3);
  t_ref0 = ref0Mref.translation();
  MatrixRotation sfRref0;
  sfRref0 = sfMref0.linear();
  Vector t_sf = sfRref0 * t_ref0;

  // add it to the position of the fly foot in support foot to
  // get the new position of fly foot in support foot.

  Vector pff_sf(3);
  pff_sf = sfMff.translation();
  t_sf += pff_sf;

  // compute the rotation that transforms ref0 into ref,
  // express it in the support foot frame. Then get the
  // associated yaw (rot around z).

  MatrixRotation ref0Rsf;
  ref0Rsf = sfRref0.transpose();
  MatrixRotation ref0Rref;
  ref0Rref = ref0Mref.linear();
  MatrixRotation tmp = ref0Rref * ref0Rsf;
  MatrixRotation Rref = sfRref0 * tmp;
  VectorRollPitchYaw rpy;
  rpy = (Rref.eulerAngles(2, 1, 0)).reverse();

  // get the yaw of the current orientation of the ff wrt sf.
  // Add it to the previously computed rpy.

  MatrixRotation sfRff;
  sfRff = sfMff.linear();
  VectorRollPitchYaw rpy_ff;
  rpy_ff = (sfRff.eulerAngles(2, 1, 0)).reverse();
  rpy += rpy_ff;

  // Now we can compute and insert the new step (we just need
  // to express the coordinates of the vector that joins the
  // support foot to the new fly foot in the coordinate frame of the
  // new fly foot).
  //
  // [dX;dY] = A^t [X;Y]
  //
  // where A is the planar rotation matrix of angle theta, [X;Y]
  // is the planar column-vector joining the support foot
  // to the new fly foot,
  // expressed in the support foot frame, and [dX;dY] is this same planar
  // column-vector expressed in the coordinates frame of the new fly foot.
  //
  // See the technical report of Olivier Stasse for more details,
  // on top of page 79.

  double ns_x = 0, ns_y = 0, ns_theta = 0;
  if (mode != MODE_1D) {
    ns_theta = rpy(2) * 180 / 3.14159265;
    if (fabs(ns_theta) < 10) {
      ns_theta = 0;
      rpy(2) = 0;
    }

    double x = t_sf(0);
    double y = t_sf(1);

    double ctheta = cos(rpy(2));
    double stheta = sin(rpy(2));

    ns_x = x * ctheta + y * stheta;
    ns_y = -x * stheta + y * ctheta;

    ns_theta = rpy(2) * 180 / 3.14159265;
    if (fabs(ns_theta) < 10) {
      ns_theta = 0;
    }
  } else {
    ns_x = t_sf(0);
    if (sfoot != 1) {
      ns_y = -ZERO_STEP_POSITION_DEFAULT;
    } else {
      ns_y = ZERO_STEP_POSITION_DEFAULT;
    }
    ns_theta = 0.;
  }

  FootPrint newStep;

  if (sfoot != 1) {
    newStep.contact = CONTACT_LEFT_FOOT;
  } else {
    newStep.contact = CONTACT_RIGHT_FOOT;
  }

  newStep.x = ns_x;
  newStep.y = ns_y;
  newStep.theta = ns_theta;

  newStep.introductionTime = timeCurr;

  footPrintList.push_back(newStep);
  footPrintList.pop_front();

  sotDEBUGOUT(15);
}

void NextStep::thisIsZero() {
  sotDEBUGIN(15);

  rfMref0 = referencePositionRightSIN.accessCopy();
  lfMref0 = referencePositionLeftSIN.accessCopy();

  sotDEBUGOUT(15);
}

void NextStep::starter(const int &timeCurr) {
  sotDEBUGIN(15);

  footPrintList.clear();
  FootPrint initSteps[4];

  initSteps[0].contact = CONTACT_RIGHT_FOOT;
  initSteps[0].x = 0;
  initSteps[0].y = -zeroStepPosition / 2;
  initSteps[0].theta = 0;
  initSteps[0].introductionTime = -1;
  footPrintList.push_back(initSteps[0]);
  introductionCallBack(-1);

  initSteps[1].contact = CONTACT_LEFT_FOOT;
  initSteps[1].x = 0;
  initSteps[1].y = +zeroStepPosition;
  initSteps[1].theta = 0;
  initSteps[1].introductionTime = -1;
  footPrintList.push_back(initSteps[1]);
  introductionCallBack(-1);

  initSteps[2].contact = CONTACT_RIGHT_FOOT;
  initSteps[2].x = 0;
  initSteps[2].y = -zeroStepPosition;
  initSteps[2].theta = 0;
  initSteps[2].introductionTime = -1;
  footPrintList.push_back(initSteps[2]);
  introductionCallBack(-1);

  initSteps[3].contact = CONTACT_LEFT_FOOT;
  initSteps[3].x = 0;
  initSteps[3].y = +zeroStepPosition;
  initSteps[3].theta = 0;
  initSteps[3].introductionTime = -1;
  footPrintList.push_back(initSteps[3]);
  introductionCallBack(-1);

  timeLastIntroduction = timeCurr - period + 1;
  if (verbose) (*verbose) << "NextStep started." << std::endl;

  sotDEBUGOUT(15);
  return;
}

void NextStep::stoper(const int &) {
  sotDEBUGIN(15);

  sotDEBUGOUT(15);
  return;
}

/* --- SIGNALS ---------------------------------------------------------- */
/* --- SIGNALS ---------------------------------------------------------- */
/* --- SIGNALS ---------------------------------------------------------- */

int &NextStep::triggerCall(int &dummy, int timeCurrent) {
  sotDEBUGIN(45);

  switch (state) {
    case STATE_STOPED:
      break;
    case STATE_STARTED: {
      int nextIntoductionTime = timeLastIntroduction + period;
      if (nextIntoductionTime <= timeCurrent) {
        nextStep(timeCurrent);
        if (NULL != verbose) {
          FootPrint &lastStep = footPrintList.back();
          (*verbose) << "<T=" << timeCurrent << "> Introduced a new step: ";
          switch (lastStep.contact) {
            case CONTACT_LEFT_FOOT:
              (*verbose) << "LF ";
              break;
            case CONTACT_RIGHT_FOOT:
              (*verbose) << "RF ";
              break;
          }
          (*verbose) << lastStep.x << "," << lastStep.y << "," << lastStep.theta
                     << std::endl;
        }
        introductionCallBack(timeCurrent);
        timeLastIntroduction = timeCurrent;
      }
      break;
    }
    case STATE_STARTING: {
      starter(timeCurrent);
      break;
    }
    case STATE_STOPING: {
      stoper(timeCurrent);
      break;
    }
  };

  sotDEBUGOUT(45);

  return dummy;
}

/* --- PARAMS ---------------------------------------------------------- */
/* --- PARAMS ---------------------------------------------------------- */
/* --- PARAMS ---------------------------------------------------------- */

void NextStep::display(std::ostream &os) const {
  os << "NextStep <" << getName() << ">:" << std::endl;
  for (std::deque<FootPrint>::const_iterator iter = footPrintList.begin();
       iter != footPrintList.end(); ++iter) {
    os << "<time=" << iter->introductionTime << "> ";
    switch (iter->contact) {
      case CONTACT_LEFT_FOOT:
        os << "LF ";
        break;
      case CONTACT_RIGHT_FOOT:
        os << "RF ";
        break;
    }
    os << "(" << iter->x << "," << iter->y << "," << iter->theta << ")"
       << std::endl;
  }
}

void NextStep::commandLine(const std::string &cmdLine,
                           std::istringstream &cmdArgs, std::ostream &os) {
  if (cmdLine == "help") {
    os << "NextStep: " << std::endl
       << " - verbose [OFF]" << std::endl
       << " - state [{start|stop}] \t get/set the stepper state. " << std::endl
       << " - yZeroStep [<value>] \t get/set the Y default position."
       << std::endl
       << " - thisIsZero {record|disp}" << std::endl
       << std::endl;
  } else if (cmdLine == "state") {
    cmdArgs >> std::ws;
    if (cmdArgs.good()) {
      std::string statearg;
      cmdArgs >> statearg;
      if (statearg == "start") {
        state = STATE_STARTING;
      } else if (statearg == "stop") {
        state = STATE_STOPING;
      }
    } else {
      os << "state = ";
      switch (state) {
        case STATE_STARTING:
          os << "starting";
          break;
        case STATE_STOPING:
          os << "stoping";
          break;
        case STATE_STARTED:
          os << "started";
          break;
        case STATE_STOPED:
          os << "stoped";
          break;
        default:
          os << "error";
          break;
      }
      os << std::endl;
    }
  } else if (cmdLine == "yZeroStep") {
    cmdArgs >> std::ws;
    if (cmdArgs.good()) {
      cmdArgs >> zeroStepPosition;
    } else {
      os << "yzero = " << zeroStepPosition;
    }
  } else if (cmdLine == "thisIsZero") {
    std::string arg;
    cmdArgs >> arg;
    if (arg == "disp_left") {
      os << "zero_left = " << lfMref0;
    } else if (arg == "disp_right") {
      os << "zero_right = " << rfMref0;
    } else if (arg == "record") {
      thisIsZero();
    }
  } else if (cmdLine == "verbose") {
    cmdArgs >> std::ws;
    std::string offarg;
    if ((cmdArgs.good()) && (cmdArgs >> offarg, offarg == "OFF")) {
      verbose = NULL;
    } else {
      verbose = &os;
    }
  } else if (cmdLine == "mode1d") {
    mode = MODE_1D;
  } else if (cmdLine == "mode3d") {
    mode = MODE_3D;
  } else {
  }
}

/* --- TWO HAND -------------------------------------------------------- */
/* --- TWO HAND -------------------------------------------------------- */
/* --- TWO HAND -------------------------------------------------------- */

NextStepTwoHandObserver::NextStepTwoHandObserver(const std::string &name)
    : referencePositionLeftSIN(NULL, "NextStepTwoHandObserver(" + name +
                                         ")::input(vector)::positionLeft"),
      referenceVelocityLeftSIN(NULL, "NextStepTwoHandObserver(" + name +
                                         ")::input(vector)::velocityLeft"),
      referenceAccelerationLeftSIN(NULL,
                                   "NextStepTwoHandObserver(" + name +
                                       ")::input(vector)::accelerationLeft"),
      leftFootPositionSIN(NULL, "NextStepTwoHandObserver(" + name +
                                    ")::input(matrixhomo)::leftfoot")

      ,
      referencePositionRightSIN(NULL, "NextStepTwoHandObserver(" + name +
                                          ")::input(vector)::positionRight"),
      referenceVelocityRightSIN(NULL, "NextStepTwoHandObserver(" + name +
                                          ")::input(vector)::velocityRight"),
      referenceAccelerationRightSIN(NULL,
                                    "NextStepTwoHandObserver(" + name +
                                        ")::input(vector)::accelerationRight"),
      rightFootPositionSIN(NULL, "NextStepTwoHandObserver(" + name +
                                     ")::input(matrixhomo)::rightfoot")

      ,
      referencePositionLeftSOUT(
          boost::bind(&NextStepTwoHandObserver::computeReferencePositionLeft,
                      this, _1, _2),
          leftFootPositionSIN << referencePositionLeftSIN
                              << referencePositionRightSIN,
          "NextStepTwoHandObserver(" + name +
              ")::output(vector)::position2handLeft"),
      referencePositionRightSOUT(
          boost::bind(&NextStepTwoHandObserver::computeReferencePositionRight,
                      this, _1, _2),
          rightFootPositionSIN << referencePositionLeftSIN
                               << referencePositionRightSIN,
          "NextStepTwoHandObserver(" + name +
              ")::output(vector)::position2handRight")

      ,
      referenceVelocitySOUT(
          SOT_MEMBER_SIGNAL_2(NextStepTwoHandObserver::computeReferenceVelocity,
                              referenceVelocityLeftSIN, Vector,
                              referenceVelocityRightSIN, Vector),
          "NextStepTwoHandObserver(" + name +
              ")::output(vector)::velocity2Hand"),
      referenceAccelerationSOUT(
          SOT_MEMBER_SIGNAL_2(
              NextStepTwoHandObserver::computeReferenceAcceleration,
              referenceAccelerationLeftSIN, Vector,
              referenceAccelerationRightSIN, Vector),
          "NextStepTwoHandObserver(" + name +
              ")::output(vector)::acceleration2Hand") {
  sotDEBUGINOUT(25);
}

SignalArray<int> NextStepTwoHandObserver::getSignals(void) {
  return (referencePositionLeftSIN
          << referenceVelocityLeftSIN << referenceAccelerationLeftSIN
          << leftFootPositionSIN << referencePositionRightSIN
          << referenceVelocityRightSIN << referenceAccelerationRightSIN
          << rightFootPositionSIN << referencePositionLeftSOUT
          << referencePositionRightSOUT << referenceVelocitySOUT
          << referenceAccelerationSOUT);
}
NextStepTwoHandObserver::operator SignalArray<int>() {
  return (referencePositionLeftSIN
          << referenceVelocityLeftSIN << referenceAccelerationLeftSIN
          << leftFootPositionSIN << referencePositionRightSIN
          << referenceVelocityRightSIN << referenceAccelerationRightSIN
          << rightFootPositionSIN << referencePositionLeftSOUT
          << referencePositionRightSOUT << referenceVelocitySOUT
          << referenceAccelerationSOUT);
}

MatrixHomogeneous &NextStepTwoHandObserver::computeRefPos(
    MatrixHomogeneous &res, int timeCurr, const MatrixHomogeneous &wMsf) {
  sotDEBUGIN(15);

#define RIGHT_HAND_REFERENCE 1
#if RIGHT_HAND_REFERENCE

  const MatrixHomogeneous &wMrh = referencePositionRightSIN(timeCurr);
  MatrixHomogeneous sfMw;
  sfMw = wMsf.inverse();
  res = sfMw * wMrh;

#else

  const MatrixHomogeneous &wMlh = referencePositionLeftSIN(timeCurr);
  const MatrixHomogeneous &wMrh = referencePositionRightSIN(timeCurr);

  MatrixHomogeneous sfMw;
  sfMw = wMsf.inverse();
  MatrixHomogeneous sfMlh;
  sfMlh = sfMw * wMlh;
  MatrixHomogeneous sfMrh;
  sfMrh = sfMw * wMrh;

  MatrixRotation R;
  VectorRollPitchYaw rpy;

  Vector prh(3);
  prh = sfMrh.translation();
  R = sfMrh.linear();
  VectorRollPitchYaw rpy_rh;
  rpy_rh = (R.eulerAngles(2, 1, 0)).reverse();

  Vector plh(3);
  plh = sfMlh.translation();
  R = sfMlh.linear();
  VectorRollPitchYaw rpy_lh;
  rpy_lh = (R.eulerAngles(2, 1, 0)).reverse();

  Vector p = 0.5 * (plh + prh);
  rpy = 0.5 * (rpy_rh + rpy_lh);

  R = (Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  res.linear() = R;
  res.translation() = p;

#endif

  sotDEBUGOUT(15);
  return res;
}

MatrixHomogeneous &NextStepTwoHandObserver::computeReferencePositionLeft(
    MatrixHomogeneous &res, int timeCurr) {
  sotDEBUGIN(15);

  const MatrixHomogeneous &wMsf = leftFootPositionSIN(timeCurr);

  sotDEBUGOUT(15);
  return computeRefPos(res, timeCurr, wMsf);
}

MatrixHomogeneous &NextStepTwoHandObserver::computeReferencePositionRight(
    MatrixHomogeneous &res, int timeCurr) {
  sotDEBUGIN(15);

  const MatrixHomogeneous &wMsf = rightFootPositionSIN(timeCurr);

  sotDEBUGOUT(15);
  return computeRefPos(res, timeCurr, wMsf);
}

Vector &NextStepTwoHandObserver::computeReferenceVelocity(const Vector &,
                                                          const Vector &,
                                                          Vector &res) {
  sotDEBUGIN(15);

  /* TODO */

  sotDEBUGOUT(15);
  return res;
}

Vector &NextStepTwoHandObserver::computeReferenceAcceleration(const Vector &,
                                                              const Vector &,
                                                              Vector &res) {
  sotDEBUGIN(15);

  /* TODO */

  sotDEBUGOUT(15);
  return res;
}

}  // namespace sot
}  // namespace dynamicgraph
