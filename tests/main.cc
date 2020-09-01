/*
 *  Copyright
 */
#include <algorithm>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>

#include <pinocchio/fwd.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>

#include <sot/pattern-generator/pg.h>
#include <sot/core/debug.hh>
#include <sot/core/robot-utils.hh>

using namespace std;

void setEndEffectorParameters(  dynamicgraph::sot::RobotUtilShrPtr aRobotUtil) {

  std::string lparameter_name = "/pg/remap/r_ankle";
  std::string lparameter_value = "leg_right_6_link";
  aRobotUtil->set_parameter<string>(lparameter_name,lparameter_value);

  lparameter_name = "/pg/remap/l_ankle";
  lparameter_value = "leg_left_6_link";
  aRobotUtil->set_parameter<string>(lparameter_name,lparameter_value);

  lparameter_name = "/pg/remap/r_wrist";
  lparameter_value = "arm_right_7_link";
  aRobotUtil->set_parameter<string>(lparameter_name,lparameter_value);

  lparameter_name = "/pg/remap/l_wrist";
  lparameter_value = "arm_left_7_link";
  aRobotUtil->set_parameter<string>(lparameter_name,lparameter_value);

  lparameter_name = "/pg/remap/body";
  lparameter_value = "base_link";
  aRobotUtil->set_parameter<string>(lparameter_name,lparameter_value);

  lparameter_name = "/pg/remap/torso";
  lparameter_value = "torso_2_link";
  aRobotUtil->set_parameter<string>(lparameter_name,lparameter_value);

}

void setFeetParameters(dynamicgraph::sot::RobotUtilShrPtr aRobotUtil){
  std::vector<std::string> lparameter_names_suffix =
      { "size/height",
        "size/width",
        "size/depth",
        "anklePosition/x",
        "anklePosition/y",
        "anklePosition/z"};
  std::vector<double> lparameter_values =
      { 0.122, 0.205, 0.107, 0.0, 0.0, 0.107};

  std::string lparameter_names_prefix =
      "/robot/specificities/feet/";
  for (unsigned int i=0;i<6;i++)
  {
    std::string full_parameter_name = lparameter_names_prefix + "right/" +
        lparameter_names_suffix[i];

    aRobotUtil->set_parameter<double>(full_parameter_name,
                                      lparameter_values[i]);
    full_parameter_name = lparameter_names_prefix + "left/" +
        lparameter_names_suffix[i];
    aRobotUtil->set_parameter<double>(full_parameter_name,
                                      lparameter_values[i]);
  }
}

void setParameters(const std::string &lrobot_description) {
  dynamicgraph::sot::RobotUtilShrPtr aRobotUtil;

    // Reading the parameter.
  string model_name("robot");

  // Search for the robot util related to robot_name.
  if (!dynamicgraph::sot::isNameInRobotUtil(model_name))
    aRobotUtil = dynamicgraph::sot::createRobotUtil(model_name);

  std::string lparameter_name("/robot_description");

  // Then set the complete robot model in the parameter set.
  aRobotUtil->set_parameter<string>(lparameter_name,lrobot_description);

  /// Specify the end effectors in the parameter server object.
  setEndEffectorParameters(aRobotUtil);

  /// Specify the size of the feet.
  setFeetParameters(aRobotUtil);
}

int main(int, char**) {
  using namespace std;
  dynamicgraph::sot::PatternGenerator aPG;

  // Search talos_reduced_wpg.urdf
  string filename(URDF_FULL_PATH);
  if (!boost::filesystem::exists(filename))
  {
    cerr << "Unable to find talos_reduced_wpg.urdf" << endl;
    exit(-1);
  }

  // Otherwise read the file
  ifstream talos_reduced_wpg_file(filename);
  ostringstream oss;
  oss << talos_reduced_wpg_file.rdbuf();

  // Name of the parameter
  const string lparameter_name("/robot_description");

  // Model of the robot inside a string.
  const string lrobot_description = oss.str();

  std::shared_ptr<std::vector<std::string>>
      alist_of_robots = dynamicgraph::sot::getListOfRobots();

  unsigned int idx = 0;
  for (auto an_it_of_robot : *alist_of_robots)
  {
    std::cout << __FILE__ << " "
              << idx++ << " " << an_it_of_robot << std::endl;
  }

  setParameters(lrobot_description);

  // Build the pattern generator
  aPG.buildPGI();

  //  aPG.InitState();
  aPG.pgCommandLine(":samplingperiod 0.005");
  aPG.pgCommandLine(":previewcontroltime 1.6");
  aPG.pgCommandLine(":walkmode 0");
  aPG.pgCommandLine(":omega 0.0");
  aPG.pgCommandLine(":stepheight 0.05");
  aPG.pgCommandLine(":singlesupporttime 0.78");
  aPG.pgCommandLine(":doublesupporttime 0.02");
  aPG.pgCommandLine(":armparameters 0.5");
  aPG.pgCommandLine(":LimitsFeasibility 0.0");
  aPG.pgCommandLine(":ZMPShiftParameters 0.015 0.015 0.015 0.015");
  aPG.pgCommandLine(":TimeDistributeParameters 2.0 3.5 1.0 3.0");
  aPG.pgCommandLine(":UpperBodyMotionParameters -0.1 -1.0 0.0");

  dynamicgraph::Vector val_jointPosition(38);
  double val_jointPositiond[38] = {
      0.0,       0.0,       1.018213,  0.0,       0.0,       0.0,      0.0,
      0.0,       -0.411354, 0.859395,  -0.448041, -0.001708, 0.0,      0.0,
      -0.411354, 0.859395,  -0.448041, -0.001708, 0.0,       0.006761, 0.25847,
      0.173046,  -0.0002,   -0.525366, 0.0,       -0.0,      0.1,      -0.005,
      -0.25847,  -0.173046, 0.0002,    -0.525366, 0.0,       0.0,      0.1,
      -0.005,    0.,        0.};

  for (unsigned int i = 0; i < val_jointPosition.size(); i++)
    val_jointPosition[i] = val_jointPositiond[i];
  aPG.jointPositionSIN.setConstant(val_jointPosition);
}
