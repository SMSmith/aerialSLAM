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
		initialEstimate.insert(getKey('x',poseID),Pose3(m));
	}
}

/********************************************************************************
* Load the IMU data as factors from matlab preprocessed data 					*
********************************************************************************/
void DynamicFactorGraph::loadIMUFactors(std::string imuFile) {
	std::ifstream imu(imuFile.c_str());

	noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.0001, 0.0001, 0.0001));

	int poseID;
	MatrixRowMajor odom(4,4);
	Matrix currentPose = eye(4);
	// std::cout << currentPose << std::endl;

	initialEstimate.insert(getKey('x',0),Pose3(currentPose));
	// std::cout << getKey('x',0) << std::endl;

	while(imu >> poseID) {
		for(int i=0; i<16; i++) {
			imu >> odom.data()[i];
		}
		currentPose = currentPose*odom;

		// IMU could be in the graph or the initial
		// graph.add(BetweenFactor<Pose3>(getKey('x',poseID), getKey('x',poseID+1), Pose3(odom), odometryNoise));
		initialEstimate.insert(getKey('x',poseID+1),Pose3(currentPose));
	}
}

// Not used - IMU calculations in c++
void DynamicFactorGraph::loadIMUData(std::string accel, std::string angular, std::string imuTime, std::string imageTime) {
	std::string value;

	std::ifstream imuFile(imuTime.c_str());
	while(imuFile.good()) {
		getline(imuFile,value,',');
		imuTimestamps.push_back(std::stof(value));
	}

	int imuLength = imuTimestamps.size();

	std::ifstream accelFile(accel.c_str());
	int i=0;
	std::vector<float> a;
	accelMeasurements.push_back(a);
	while(accelFile.good()) {
		if(accelMeasurements[i].size()==imuLength) {
			std::vector<float> b;
			accelMeasurements.push_back(b);
			i++;
		}
		getline(accelFile,value,',');
		accelMeasurements[i].push_back(std::stof(value));
	}

	std::ifstream angularFile(angular.c_str());
	i=0;
	std::vector<float> c;
	angularMeasurements.push_back(c);
	while(angularFile.good()) {
		if(angularMeasurements[i].size()==imuLength) {
			std::vector<float> d;
			angularMeasurements.push_back(d);
			i++;
		}
		getline(angularFile,value,',');
		angularMeasurements[i].push_back(std::stof(value));
	}

	std::ifstream imageFile(imageTime.c_str());
	while(imageFile.good()) {
		getline(imageFile,value,',');
		imageTimestamps.push_back(std::stof(value));
	}

	// std::cout << accelMeasurements[0].size() << " " << angularMeasurements.size() << " " << imuTimestamps.size() << " " << imageTimestamps.size() << " " << std::endl;
	// std::cout << accelMeasurements[0].size() << " " << accelMeasurements[1].size() << " " << accelMeasurements[2].size() << std::endl;
	// std::cout << accelMeasurements[0][0] << " " << accelMeasurements[1][0] << " " << accelMeasurements[2][0] << std::endl;
	// std::cout << accelMeasurements[0][accelMeasurements[0].size()-1] << " " << accelMeasurements[1][accelMeasurements[1].size()-1] << " " << accelMeasurements[2][accelMeasurements[2].size()-1] << std::endl;
}	

/********************************************************************************
* This reads in the landmarks from file.  File format is:						*
* xID lID uL uR v X Y Z 														*
* where xID is the state where the landmark was seen, lID is the landmark lID, 	*
* uL and uR are the horizontal pixel coordinates of the landmarks, v is the 	*
* vertical pixel coordinate (they align, so it should be the same?), and X,Y,Z 	*
* is the world point location in the camera frame.								*
********************************************************************************/
void DynamicFactorGraph::loadLandmarks(std::string landmarkFile) {
	double uL, uR, v, X, Y, Z;
	size_t x, l;

	// Noise model for camera
	const noiseModel::Isotropic::shared_ptr cameraNoise = noiseModel::Isotropic::Sigma(3,0.1);
	// Camera calibration matrix, params are in *.h
	const Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(fx,fy,s,u0,v0,b));

	std::ifstream factorFile(landmarkFile.c_str());

	while(factorFile >> x >> l >> uL >> uR >> v >> X >> Y >> Z) {
		graph.push_back(
			GenericStereoFactor<Pose3,Point3>(StereoPoint2(uL,uR,v), cameraNoise,
				getKey('x',x), getKey('l',l), K));
		if (!initialEstimate.exists(getKey('l',l))) {
			Pose3 camPose = initialEstimate.at<Pose3>(getKey('x',x));
			// Point3 worldPoint = camPose.transform_from(Point3(X,Y,Z));
			Point3 worldPoint = camPose.transform_from(Point3(X,Y,Z));

			initialEstimate.insert(getKey('l',l), worldPoint);
		}
	}
}
Key DynamicFactorGraph::getKey(char type, int index) {
	return Symbol(type,index+2039287402);
}

void DynamicFactorGraph::solve() {
	// The first pose will probably be identity at (0,0,0), as such, 
	// we don't want it to move during optimaziotn, so the following is done:
	Pose3 firstPose = initialEstimate.at<Pose3>(getKey('x',0));
	// graph.push_back(NonlinearEqua	lity<Pose3>(getKey('x',0),firstPose));

	Values poseValues;

	// std::cout << initialEstimate.at<Pose3>(getKey('x',0)).x() << " " << initialEstimate.at<Pose3>(getKey('x',0)).y() << std::endl;

	// Now we just solve the optimization algorithm
	// try {
		// LevenbergMarquardtOptimizer optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
		// Values result = optimizer.optimize();

		// Make the output usable
		// poseValues = result.filter<Pose3>();
		// poseValues.print("some std::string that gets printed before the data, maybe do csv style headings?");
	// }
	// catch(...) {
	// 	std::cout << "Using Gauss Newton because LM failed" << std::endl;
		GaussNewtonParams params;
		params.linearSolverType = NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY;
		// params.linearSolverType = NonlinearOptimizerParams::MULTIFRONTAL_QR;
		GaussNewtonOptimizer optimizer2 = GaussNewtonOptimizer(graph,initialEstimate,params);
		Values result2 = optimizer2.optimize();

	// 	// Make the output usable
		poseValues = result2.filter<Pose3>();
	// 	// poseValues.print("some std::string that gets printed before the data, maybe do csv style headings?");
	// }

	// graph.print();
	// initialEstimate.print("");

	// The resulting poses after solve
	std::cout << "x,y,z" << std::endl;
	for(int i=1; i<poseValues.size(); i++) {
		std::cout << poseValues.at<Pose3>(getKey('x',i)).x() << "," << poseValues.at<Pose3>(getKey('x',i)).y() << "," << poseValues.at<Pose3>(getKey('x',i)).z() << std::endl;
	}

	// The resulting landmarks after solve
	// Values landmarkValues = result.filter<Point3>();
	// // landmarkValues.print("");
	// std::cout << "x,y,z" << std::endl;
	// for(int i=1; i<landmarkValues.size(); i++) {
	// 	try {
	// 		std::cout << landmarkValues.at<Point3>(getKey('l',i)).x() << "," << landmarkValues.at<Point3>(getKey('l',i)).y() << "," << landmarkValues.at<Point3>(getKey('l',i)).z() << std::endl;
	// 	}
	// 	catch (...){}
	// }

	// The initial landmarks
	// Values landmarkInitial = initialEstimate.filter<Point3>();
	// std::cout << "x,y,z" << std::endl;
	// for(int i=1; i<landmarkInitial.size(); i++) {
	// 	try {
	// 		std::cout << landmarkInitial.at<Point3>(getKey('l',i)).x() << "," << landmarkInitial.at<Point3>(getKey('l',i)).y() << "," << landmarkInitial.at<Point3>(getKey('l',i)).z() << std::endl;
	// 	}
	// 	catch (...) {}
	// }

	// The initial pose
	// Values initialPose = initialEstimate.filter<Pose3>();
	// std::cout << "x,y,z" << std::endl;
	// for(int i=1; i<initialPose.size(); i++) {
	// 	std::cout << initialPose.at<Pose3>(getKey('x',i)).x() << "," << initialPose.at<Pose3>(getKey('x',i)).y() << "," << initialPose.at<Pose3>(getKey('x',i)).z() << std::endl;
	// }
}