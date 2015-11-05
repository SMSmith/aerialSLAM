#include "featuredpa.h"

FeatureDPA::FeatureDPA::FeatureDPA()
{
  featureDetector = FastFeatureDetector::create();
  extractor = ORB::create();
}


std::vector< DMatch > FeatureDPA::findMatches(const Mat& img_1, const Mat& img_2, bool print)
{
  const static bool DEBUG = true;
   //-- Step 1: Detect the keypoints using FAST Detector
  std::vector<KeyPoint> keypoints_1, keypoints_2;
  featureDetector->detect( img_1, keypoints_1 );
  featureDetector->detect( img_2, keypoints_2 );
  
  //-- Step 2: Calculate descriptors (feature vectors)
  Mat descriptors_1, descriptors_2;
  extractor->compute( img_1, keypoints_1, descriptors_1 );
  extractor->compute( img_2, keypoints_2, descriptors_2 );
  
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
  std::vector< DMatch > good_matches;
  for( int i = 0; i < descriptors_1.rows; i++ ) { 
    if( matches[i].distance <= max(10*min_dist, 0.02) ) { 
      good_matches.push_back( matches[i]); 
    }
  }

  //-- Draw only "good" matches
  if(print) {
   displayMatches(img_1,img_2,keypoints_1,keypoints_2,good_matches); 
   waitKey(0);
  }
  
  return good_matches;
}

void FeatureDPA::displayMatches(const Mat& img_1, const Mat& img_2, 
                                std::vector<KeyPoint> keypoints_1, std::vector<KeyPoint> keypoints_2,
                                const std::vector< DMatch >& matches)
{
  Mat img_matches;
  drawMatches( img_1, keypoints_1, img_2, keypoints_2,
               matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  imshow("Matches", img_matches);
}

