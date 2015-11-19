#include "featuredpa.h"

FeatureDPA::FeatureDPA::FeatureDPA()
{
  featureDetector = FastFeatureDetector::create();
  extractor = ORB::create();
}

void FeatureDPA::initializeFactorGraph() {
  // The factor graph
  NonlinearFactorGraph graph;

  // A unique symbol is needed for each factor, here is the prior
  static Symbol x1('x',1);

  // Initialize the prior and add it to the graph
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.3, 0.3, 0.3, 0.1, 0.1, 0.1));
  graph.add(PriorFactor<Pose3>(x1, Pose3(), priorNoise)); // Initialized to zero

  // Add more factors
}

Feature FeatureDPA::findMatches(const Mat& img_1, const Mat& img_2, bool print)
{
  const static bool DEBUG = false;
  Feature feature;
  
   //-- Step 1: Detect the keypoints using FAST Detector
  featureDetector->detect( img_1, feature.keypoints_1 );
  featureDetector->detect( img_2, feature.keypoints_2 );
  
  //-- Step 2: Calculate descriptors (feature vectors)
  Mat descriptors_1, descriptors_2;
  extractor->compute( img_1, feature.keypoints_1, descriptors_1 );
  extractor->compute( img_2, feature.keypoints_2, descriptors_2 );
  
  //-- Step 3: Matching descriptor vectors with a brute force matcher
  std::vector< DMatch > matches;
  matcher.match( descriptors_1, descriptors_2, matches );
  double max_dist = 0; double min_dist = 200;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_1.rows; i++ ) { 
    double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }
  
  if(DEBUG) {
    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );
  }

  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)here.
  for( int i = 0; i < descriptors_1.rows; i++ ) { 
    if( matches[i].distance <= max(10*min_dist, 0.02) ) { 
      feature.matches.push_back( matches[i]); 
    }
  }

  //-- Draw only "good" matches
  if(print) {
   displayMatches(img_1,img_2,feature.keypoints_1,feature.keypoints_2,feature.matches);
   waitKey(0);
  }

  return feature;
}

void FeatureDPA::displayMatches(const Mat& img_1, const Mat& img_2, 
                                const std::vector<KeyPoint>& keypoints_1, const std::vector<KeyPoint>& keypoints_2,
                                const std::vector<DMatch>& matches)
{
  Mat img_matches;
  drawMatches( img_1, keypoints_1, img_2, keypoints_2,
               matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  imshow("Matches", img_matches);
}

/**
  * features: The set of matched features between two images
  * projMat1: Projection matrix for camera 1
  * projMat2: Projection matrix for camera 2
  * print: boolean value indicating whether to display image
  *
  * Returns features with actual world points
  */
Feature FeatureDPA::getWorldPoints(Feature features, const Matx34d& projMat1, const Matx34d& projMat2)
{
  //-- Define vectors containing the points
  std::vector<Point2f> pts1;
  std::vector<Point2f> pts2;

  // Get the matched points in order of matching
  int numMatches = get2DPointfs(features,pts1,pts2);

  //-- Define matrix that will be 4xN homogeneous world points
  Mat hWorldPoints;

  //-- Use the OpenCV triangulation function to get 3D points for each matched feature
  triangulatePoints(projMat1, projMat2,
                        pts1, pts2,
                        hWorldPoints);

  // hWorldPoints is a 4xN homogeneous matrix, and it needs to be converted to
  // Nx3 3D Euclidean space coordinates
  Mat worldPoints;
  convertPointsFromHomogeneous(hWorldPoints.t(), worldPoints);

  // Assign the world points to convient format -- Is there a better way?
  features.worldPoint.resize(numMatches);
  for(int i=0; i<numMatches; i++) {
    features.worldPoint[i] = worldPoints.at<Point3f>(i,0);
  }
  return features;
}

/**
  * f1 - features from frame i between two images
  * f2 - features from frame i+1 between two images
  * (both must contain world points)
  *
  * Returns the transform from the world points
  */
Mat FeatureDPA::estimatePose(Feature f1, Feature f2) {
  Mat tf, inliers;

  std::vector<Point3f> matches1;
  std::vector<Point3f> matches2;

  // Kind of like ICP, except not really...
  int matchCount = matchWorldPoints(f1.worldPoint,f2.worldPoint,matches1,matches2);

  // Gets the affine transform (just SVD) between two sets of world points
  // Requires that they are in order in comparison with each other
  estimateAffine3D(matches1,matches2,tf,inliers);

  // Do we care about inliers?
  return tf;
}

/**
  * features: the paired feature points
  * pts1 and pts2:  sorted vectors of the aligned feature points
  * returns the number of matched points
  */
int FeatureDPA::get2DPointfs(Feature features, std::vector<Point2f> &pts1, std::vector<Point2f> &pts2) {
  //-- Get the smaller number of matched features
  int numMatches = features.matches.size();

  //--Convert all matching keypoints into the points that go into each vector
  for (int i = 0; i < numMatches; i++) {
    Point2f pt1 = features.keypoints_1[features.matches[i].queryIdx].pt;
    Point2f pt2 = features.keypoints_2[features.matches[i].trainIdx].pt;
    pts1.push_back(pt1);
    pts2.push_back(pt2);
  }

  return numMatches;
}

/**
  *
  * w1 and w2: the 3D Points from two separate frames
  * m1 and m2: the list of paired 3D points that will be returned
  * also returns the number of paired 3D points (minimum of lengths of w1 and w2)
  */
int FeatureDPA::matchWorldPoints(std::vector<Point3f> w1,  std::vector<Point3f> w2, 
                                 std::vector<Point3f> &m1, std::vector<Point3f> &m2) {
  int matchCount = min(w1.size(),w2.size());
  m1.resize(matchCount);
  m2.resize(matchCount);

  std::vector<Point3f> shortList;
  std::vector<Point3f> longList;
  if (matchCount == w1.size()) {
    shortList = w1;
    longList = w2;
  }
  else {
    shortList = w2;
    longList = w1;
  }

  // ICP???  LOL
  for(int i=0; i<shortList.size(); i++) {
    // For each element in the shorter list:
    // - Find the closest element in the longer list
    // - Assign the first element and the closest to the output
    // - Pop the closest point from the long list
    double minDist = INFINITY;
    int minIndex;
    for(int j=0; j<longList.size(); j++) {
      double dist = norm(Mat(shortList[i]),Mat(longList[i]));
      if(dist<minDist) {
        minDist = dist;
        minIndex = j;
      }
    }
    if(matchCount == w1.size()) {
      m1[i] = shortList[i];
      m2[i] = longList[minIndex];
    }
    else {
      m1[i] = longList[minIndex];
      m2[i] = shortList[i];
    }
    longList.erase(longList.begin()+minIndex);
  }

  return matchCount;
}