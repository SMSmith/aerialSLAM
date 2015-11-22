#include "DynamicFactorGraph.h"

DynamicFactorGraph::DynamicFactorGraph() {
	
}

/********************************************************************************
* If we load from file, we probably shouldn't do this 							*
********************************************************************************/
void DynamicFactorGraph::initializeFactorGraph() {

  // A unique symbol is needed for each factor, here is the prior
  static Symbol x1('x',1);

  // Initialize the prior with a standard noise model (.3m translations, .1 radian rotations)
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.3, 0.3, 0.3, 0.1, 0.1, 0.1));

  // The graph is global and adding factors is as easy as the following:
  graph.add(PriorFactor<Pose3>(x1, Pose3(), priorNoise)); // Initialized to zero

  // The camera calibration matrix
  // kx, ky, s, u0, v0, b
  // k11, k22, k12 or k21, k13, k23, baseline
  const Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(fx,fy,s,u0,v0,b));

}

/********************************************************************************
* This reads in the poses from the stereo vision. File format is just:			*
* ID P00 P01 P02 P03 P10 P11 P12 P13 P20 P21 P22 P23 P30 P31 P32 P33			*
* Where ID is the unique integer id of the pose  								*
********************************************************************************/
void DynamicFactorGraph::loadInitialPoses(std::string initialPoseFile) {
	std::ifstream poseFile(initialPoseFile.c_str());

	int poseID;
	MatrixRowMajor m(4,4);

	while(poseFile >> poseID) {
		for(int i=0; i<16; i++) {
			poseFile >> m.data()[i];
		}

		// Keep track of the initial estimate based on stereo vision
		initialEstimate.insert(Symbol('x', poseID), Pose3(m));
	}
}

/********************************************************************************
* This reads in the landmarks from file.  File format is:						*
* xID lID uL uR v X Y zero 														*
* where xID is the state where the landmark was seen, lID is the landmark lID, 	*
* uL and uR are the horizontal pixel coordinates of the landmarks, v is the 	*
* vertical pixel coordinate (they align, so it should be the same?), and X,Y,Z 	*
* is the world point location in the camera frame.								*
********************************************************************************/
void DynamicFactorGraph::loadLandmarks(std::string landmarkFile) {
	double uL, uR, v, X, Y, Z;
	size_t x, l;

	// Noise model for camera
	const noiseModel::Isotropic::shared_ptr cameraNoise = noiseModel::Isotropic::Sigma(3,1);
	// Camera calibration matrix, params are in *.h
	const Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(fx,fy,s,u0,v0,b));

	std::ifstream factorFile(landmarkFile.c_str());

	while(factorFile >> x >> l >> uL >> uR >> v >> X >> Y >> Z) {
		graph.push_back(
			GenericStereoFactor<Pose3,Point3>(StereoPoint2(uL,uR,v), cameraNoise,
				Symbol('x', x), Symbol('l', l), K));
		if (!initialEstimate.exists(Symbol('l',l))) {
			Pose3 camPose = initialEstimate.at<Pose3>(Symbol('x',x));
			Point3 worldPoint = camPose.transform_from(Point3(X,Y,Z));

			initialEstimate.insert(Symbol('l',l), worldPoint);
		}
	}
}

void DynamicFactorGraph::solve() {
	// The first pose will probably be identity at (0,0,0), as such, 
	// we don't want it to move during optimaziotn, so the following is done:
	Pose3 firstPose = initialEstimate.at<Pose3>(Symbol('x',1));
	graph.push_back(NonlinearEquality<Pose3>(Symbol('x',1),firstPose));

	// Now we just solve the optimization algorithm
	LevenbergMarquardtOptimizer optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
	Values result = optimizer.optimize();

	// Make the output usable
	Values poseValues = result.filter<Pose3>();
	poseValues.print("some std::string that gets printed before the data, maybe do csv style headings?");
}