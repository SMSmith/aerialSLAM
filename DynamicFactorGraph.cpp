#include "DynamicFactorGraph.h"

DynamicFactorGraph::DynamicFactorGraph() {
	
}

void DynamicFactorGraph::initializeFactorGraph() {

  // A unique symbol is needed for each factor, here is the prior
  static Symbol x1('x',1);

  // Initialize the prior with a standard noise model (.3m translations, .1 radian rotations)
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.3, 0.3, 0.3, 0.1, 0.1, 0.1));

  // The graph is global and adding factors is as easy as the following:
  graph.add(PriorFactor<Pose3>(x1, Pose3(), priorNoise)); // Initialized to zero

}