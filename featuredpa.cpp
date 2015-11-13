#include "featuredpa.h"

FeatureDPA::FeatureDPA::FeatureDPA()
{
  featureDetector = FastFeatureDetector::create();
  extractor = ORB::create();
}


Feature FeatureDPA::findMatches(const Mat& img_1, const Mat& img_2, bool print)
{
  const static bool DEBUG = true;
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
  *
  *
  * img_1: Image from camera 1
  * img_2: Image from camera 2
  * projMat1: Projection matrix for camera 1
  * projMat2: Projection matrix for camera 2
  * print: boolean value indicating whether to display image
  *
  * Returns Nx3 Euclidean coordinates
  */
Mat FeatureDPA::getWorldPoints(const Mat& img_1, const Mat& img_2,
                               const Matx34d& projMat1, const Matx34d& projMat2,
                               bool print)
{
  //-- Get matching features from both images
  Feature features = findMatches(img_1, img_2, print);

  //-- Get the smaller number of matched features
  int numMatches = features.matches.size();

  //-- Define vectors containing the points
  std::vector<Point2f> pts1;
  std::vector<Point2f> pts2;

  //--Convert all matching keypoints into the points that go into each vector
  for (int i = 0; i < numMatches; i++) {
    Point2f pt1 = features.keypoints_1[features.matches[i].queryIdx].pt;
    Point2f pt2 = features.keypoints_2[features.matches[i].trainIdx].pt;
    pts1.push_back(pt1);
    pts2.push_back(pt2);
  }

  //-- Define matrix that will be 4xN homogeneous world points
  Mat hWorldPoints;

  //-- Use the OpenCV triangulation function to get 3D points for each matched feature
  cv::triangulatePoints(projMat1, projMat2,
                        pts1, pts2,
                        hWorldPoints);

  // hWorldPoints is a 4xN homogeneous matrix, and it needs to be converted to
  // Nx3 3D Euclidean space coordinates
  Mat worldPoints;
  convertPointsFromHomogeneous(hWorldPoints.t(), worldPoints);

  // Return the Euclidean coordinates
  return worldPoints.reshape(1, numMatches);
}
