#ifndef FEATUREDPA_H
#define FEATUREDPA_H

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <vector>
#include <iostream>

using namespace cv;

struct Feature {
 std::vector<KeyPoint> keypoints_1; 
 std::vector<KeyPoint> keypoints_2;
 std::vector<DMatch> matches;
 std::vector<Point3f> worldPoint;
};

class FeatureDPA {
public:
  FeatureDPA();
  
  //-- Finds the matching features for two images
  Feature findMatches(const Mat& img_1, const Mat& img_2, bool print=false);
  //-- Display matches side-by-side in image. Access by print=true in @findMatches
  void displayMatches(const Mat& img_1, const Mat& img_2,
                      const std::vector<KeyPoint>& keypoints_1, const std::vector<KeyPoint>& keypoints_2,
                      const std::vector<DMatch>& matches);
  //-- Gives 3D points of two given features
  Feature getWorldPoints(Feature features, const Matx34d& projMat1, const Matx34d& projMat2);

  //-- 
  Mat estimatePose(Feature f1, Feature f2, const Matx34d& projMat1, const Matx34d& projMat2);

private:
  Ptr<FastFeatureDetector> featureDetector;
  Ptr<ORB> extractor;
  BFMatcher matcher;

  int get2DPointfs(Feature features, std::vector<Point2f> &pts1, std::vector<Point2f> &pts2);
};

#endif // FEATUREDPA_H
