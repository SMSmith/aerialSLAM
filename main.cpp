#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "featuredpa.h"

using namespace cv;
using namespace std;

/** @function main */
int main( int argc, char** argv )
{

  //-- This is just a demo for showing how to read two images and then get the matched features
  Mat img_1L = imread( "datasets/cmu_16662_p2/sensor_data/left000.jpg", CV_LOAD_IMAGE_GRAYSCALE );
  Mat img_1R = imread( "datasets/cmu_16662_p2/sensor_data/right000.jpg", CV_LOAD_IMAGE_GRAYSCALE );
  Mat img_2L = imread( "datasets/cmu_16662_p2/sensor_data/left001.jpg", CV_LOAD_IMAGE_GRAYSCALE );
  Mat img_2R = imread( "datasets/cmu_16662_p2/sensor_data/right001.jpg", CV_LOAD_IMAGE_GRAYSCALE );

  //-- These are the camera projection matrices
  Matx34d projMat1(164.255034407511, 0.0, 214.523999214172, 0.0,
                   0.0, 164.255034407511, 119.433252334595, 0.0,
                   0.0, 0.0, 1.0, 0.0);
  Matx34d projMat2(164.255034407511, 0.0, 214.523999214172, 26.62574107745753,
                   0.0, 164.255034407511, 119.433252334595, 0.0,
                   0.0, 0.0, 1.0, 0.0);

  if( !img_1L.data || !img_2R.data ){
    cout<<"Images failed!"<<endl;
    return -1;
  }

  //-- Create @featureFinder (Detection,Pairing,Alignment)
  FeatureDPA featureFinder;
  //-- Give it two images and (true/false) if you want to see the matches
  //-- @feature has keypoints for both images and the good matches between images

  Feature f1 = featureFinder.findMatches(img_1L,img_1R,true);
  Feature f2 = featureFinder.findMatches(img_2L,img_2R,true);

  Feature f1WithWorlds = featureFinder.getWorldPoints(f1,projMat1,projMat2);
  Feature f2WithWorlds = featureFinder.getWorldPoints(f2,projMat1,projMat2);

  Mat rotation = featureFinder.estimatePose(f1,f2,projMat1,projMat2);

  return 0;
}


