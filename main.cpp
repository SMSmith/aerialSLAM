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
  Mat img_1 = imread( "../cmu_16662_p2/sensor_data/left000.jpg", CV_LOAD_IMAGE_GRAYSCALE );
  Mat img_2 = imread( "../cmu_16662_p2/sensor_data/right000.jpg", CV_LOAD_IMAGE_GRAYSCALE );

  if( !img_1.data || !img_2.data ){ 
    cout<<"Images failed!"<<endl;
    return -1; 
  }
  
  //-- Create @featureFinder (Detection,Pairing,Alignment) 
  FeatureDPA featureFinder;
  //-- Give it two images and (true/false) if you want to see the matches
  //-- @feature has keypoints for both images and the good matches between images
  Feature feature = featureFinder.findMatches(img_1,img_2,true);
  
  
  return 0;
}
