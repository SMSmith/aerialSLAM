#ifndef DYNAMICFACTORGRAPH_H
#define DYNAMICFACTORGRAPH_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/config.h>
#include <gtsam/base/types.h>
#include <vector>
#include <iostream>

using namespace gtsam;

class DynamicFactorGraph {
public:
  DynamicFactorGraph();
  //-- Begin factor graph creation
  void                      initializeFactorGraph();

private:
  NonlinearFactorGraph      graph;

};

#endif // DYNAMICFACTORGRAPH_H
