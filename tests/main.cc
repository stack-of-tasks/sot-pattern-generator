/*
 *  Copyright
 */
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>

#ifdef BOOST_MPL_LIMIT_VECTOR_SIZE
#pragma push_macro("BOOST_MPL_LIMIT_VECTOR_SIZE")
#undef BOOST_MPL_LIMIT_VECTOR_SIZE
#define BOOST_MPL_LIMIT_VECTOR_SIZE_PUSH
#endif

#ifdef BOOST_MPL_LIMIT_LIST_SIZE
#pragma push_macro("BOOST_MPL_LIMIT_LIST_SIZE")
#undef BOOST_MPL_LIMIT_LIST_SIZE
#define BOOST_MPL_LIMIT_LIST_SIZE_PUSH
#endif

#include <boost/property_tree/ptree.hpp>

#ifdef BOOST_MPL_LIMIT_VECTOR_SIZE_PUSH
#pragma pop_macro("BOOST_MPL_LIMIT_VECTOR_SIZE")
#endif

#ifdef BOOST_MPL_LIMIT_LIST_SIZE_PUSH
#pragma pop_macro("BOOST_MPL_LIMIT_LIST_SIZE")
#endif

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <sot/pattern-generator/pg.h>
#include <sot/core/debug.hh>
#include <sot/core/robot-utils.hh>

int main(int, char**) {
  using namespace std;
  dynamicgraph::sot::PatternGenerator aPG;

  // Get environment variable CMAKE_PREFIX_PATH
  const string s_cmake_prefix_path = getenv( "CMAKE_PREFIX_PATH" );

  // Read the various paths
  vector<string> paths;
  boost::split(paths, s_cmake_prefix_path, boost::is_any_of(":;"));

  // Search talos_reduced_wpg.urdf
  string filename="";
  for (auto test_path : paths)
  {
    filename = test_path +
        string("/share/talos_data/urdf/talos_reduced_wpg.urdf");
    if ( boost::filesystem::exists(filename))
      break;

  }

  // If not found fails
  if (filename.size()==0)
  {
    cerr << "Unable to find talos_reduced_wpg.urdf" << endl;
    exit(-1);
  }

  // Otherwise read the file
  ifstream talos_reduced_wpg_file(filename);
  ostringstream oss;
  oss << talos_reduced_wpg_file.rdbuf();

  // Name of the parameter
  string lparameter_name("/robot_description");

  // Model of the robot inside a string.
  string lrobot_description = oss.str();

  std::shared_ptr<std::vector<std::string>>
      alist_of_robots = dynamicgraph::sot::getListOfRobots();

  unsigned int idx = 0;
  for (auto an_it_of_robot : *alist_of_robots)
  {
    std::cout << idx++ << " " << an_it_of_robot << std::endl;
  }

  // Reading the parameter.
  string model_name("robot");

  // Search for the robot util related to robot_name.
  dynamicgraph::sot::RobotUtilShrPtr aRobotUtil =
      dynamicgraph::sot::getRobotUtil(model_name);

  std::cout << dynamicgraph::sot::RefVoidRobotUtil() << std::endl;
  std::cout << aRobotUtil.get() << std::endl;
  if (aRobotUtil == dynamicgraph::sot::RefVoidRobotUtil())
    aRobotUtil = dynamicgraph::sot::createRobotUtil(model_name);
  std::cout << aRobotUtil.get() << std::endl;

  // Then build the complete robot model.
  aRobotUtil->set_parameter<string>(lparameter_name,lrobot_description);

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
