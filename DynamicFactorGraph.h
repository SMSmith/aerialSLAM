#ifndef DYNAMICFACTORGRAPH_H
#define DYNAMICFACTORGRAPH_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/BetweenFactor.h>
// #include <gtsam/config.h>
// #include <gtsam/base/types.h>
#include <vector>
#include <iostream>
#include <fstream>

using namespace gtsam;

class DynamicFactorGraph {
public:
  DynamicFactorGraph();
  //-- Begin factor graph creation
  void                      initializeFactorGraph();
  void                      loadInitialPoses(std::string initialPoseFile);
  void                      loadLandmarks(std::string landmarkFile);
  void                      solve();
  void                      loadIMU(std::string imuFile);

private:
  NonlinearFactorGraph      graph;
  Values                    initialEstimate;

  // Camera Calibration
  static const double fx = 164.255034407511;
  static const double fy = 164.255034407511;
  static const double s = 0.0;
  static const double u0 = 214.523999214172;
  static const double v0 = 119.433252334595;
  static const double b = 0.1621;

};

#endif // DYNAMICFACTORGRAPH_H
