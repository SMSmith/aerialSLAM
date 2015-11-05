#ifndef FEATUREDPA_H
#define FEATUREDPA_H

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

using namespace cv;

class FeatureDPA
{
public:
  FeatureDPA();
  
  //-- Finds the matching features for two images
  std::vector< DMatch > findMatches(const Mat& img_1, const Mat& img_2, bool print=false);
  
private:
  //-- Display matches side-by-side in image. Access by print=true in @findMatches
  void displayMatches(const Mat& img_1, const Mat& img_2,
                      std::vector<KeyPoint> keypoints_1, std::vector<KeyPoint> keypoints_2,
                      const std::vector< DMatch >& matches);
  
  Ptr<FastFeatureDetector> featureDetector;
  Ptr<ORB> extractor;
  BFMatcher matcher;
};

#endif // FEATUREDPA_H
