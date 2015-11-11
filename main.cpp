#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "featuredpa.h"

using namespace cv;
using namespace std;

/** @function main */
int main( int argc, char** argv )
{
  const static bool OLD = false;
  if(OLD) {
    //-- This is just a demo for showing how to read two images and then get the matched features
    Mat img_1 = imread( "../datasets/cmu_16662_p2/sensor_data/left000.jpg", CV_LOAD_IMAGE_GRAYSCALE );
    Mat img_2 = imread( "../datasets/cmu_16662_p2/sensor_data/right000.jpg", CV_LOAD_IMAGE_GRAYSCALE );

    if( !img_1.data || !img_2.data ){
      cout<<"Images failed!"<<endl;
      return -1;
    }

    //-- Create @featureFinder (Detection,Pairing,Alignment)
    FeatureDPA featureFinder;
    //-- Give it two images and (true/false) if you want to see the matches
    //-- @feature has keypoints for both images and the good matches between images
    Feature feature = featureFinder.findMatches(img_1,img_2,true);
    printf("Ran once\n");
  }
  else {
    //-- This is just a demo for showing how to read through stereo images and then get the matched features
    int i = 0;
    Mat img_1 = imread( "../datasets/cmu_16662_p2/sensor_data/left000.jpg", CV_LOAD_IMAGE_GRAYSCALE );
    Mat img_2 = imread( "../datasets/cmu_16662_p2/sensor_data/right000.jpg", CV_LOAD_IMAGE_GRAYSCALE );
    char file1 [50];
    char file2 [50];
    char num [4];
    clock_t begin_time = clock();
    //float avg_time;
    float total_time = 0.0;
    //-- Perform Analysis on the current files
    //-- Create @featureFinder (Detection,Pairing,Alignment)
    FeatureDPA featureFinder;
    //-- Give it two images and (true/false) if you want to see the matches
    //-- @feature has keypoints for both images and the good matches between images
    Feature feature = featureFinder.findMatches(img_1,img_2,false);
  
    while(img_1.data && img_2.data) {
      printf("Frame %i\n", i);
      begin_time = clock();
      //-- Attempt to read in the next set of files
      i++;
      if(i < 10) {
	sprintf(num, "00%i", i);
      }
      else if(i < 100) {
	sprintf(num, "0%i", i);
      }
      else {
	sprintf(num, "%i", i);
      }
      sprintf(file1, "../datasets/cmu_16662_p2/sensor_data/left%s.jpg", num);
      sprintf(file2, "../datasets/cmu_16662_p2/sensor_data/right%s.jpg", num);
      img_1 = imread( file1, CV_LOAD_IMAGE_GRAYSCALE );
      img_2 = imread( file2, CV_LOAD_IMAGE_GRAYSCALE );
      
      //-- Perform Analysis on the current files
      //-- Give it two images and (true/false) if you want to see the matches
      //-- @feature has keypoints for both images and the good matches between images
      feature = featureFinder.findMatches(img_1,img_2,false);
      total_time += float( clock () - begin_time ) /  CLOCKS_PER_SEC;
      printf("Delta Time: %.5f\n", float( clock () - begin_time ) /  CLOCKS_PER_SEC);
    }
    printf("Average Run Time: %.5f\n", total_time/i);
    cout<<"End image stream"<<endl;
  }
  
  return 0;
}
