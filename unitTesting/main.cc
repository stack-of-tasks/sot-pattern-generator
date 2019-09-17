/*
 *  Copyright
 */

#include <sot-pattern-generator/pg.h>

int main (int , char** )
{
  dynamicgraph::sot::PatternGenerator aPG;

  std::string aRobotURDF
    ("/opt/openrobots/share/talos_data/urdf/talos_reduced_wpg.urdf");
  std::string aRobotSRDF
    ("/opt/openrobots/share/talos_data/srdf/talos_wpg.srdf");

  aPG.setURDFFile(aRobotURDF);
  aPG.setSRDFFile(aRobotSRDF);

  aPG.buildModel();
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
  double val_jointPositiond[38] =
      { 0.0, 0.0,  1.018213,  0.0 ,  0.0, 0.0,
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,
        0.0 ,  0.006761,  0.25847 ,  0.173046, -0.0002,
        -0.525366, 0.0, -0.0,  0.1, -0.005, -0.25847 ,
        -0.173046, 0.0002  , -0.525366, 0.0,  0.0,
       0.1,-0.005, 0.,  0. };

  for(unsigned int i=0;i<val_jointPosition.size();i++)
    val_jointPosition[i]=val_jointPositiond[i];
  aPG.jointPositionSIN.setConstant(val_jointPosition);


}
