% Get the shared features between two stereo images
% INPUT: leftFile - path to the left camera's image for this frame
% INPUT: rightFile - path to the right camera's image for this frame
% OUTPUT: features - structure containing:
%                       leftPoints - SURF points in left image
%                       rightPoints - SURF points in right image
%                       leftFeatures - Features in left image
%                       rightFeatures - Features in right image
%                       matchMetric - how well the feature match
function [features] = getStereoFeatures(leftFile, rightFile)
% Load stereo images 
leftImage = rgb2gray(imread(leftFile));
rightImage = rgb2gray(imread(rightFile));
% Get features for each image
leftPoints = detectSURFFeatures(leftImage);
rightPoints = detectSURFFeatures(rightImage);
% Extract features
[leftFeat, vptsLeft] = extractFeatures(leftImage, leftPoints);
[rightFeat, vptsRight] = extractFeatures(rightImage, rightPoints);
% Get matching features' indexes
[matchIndex, matchMetric] = matchFeatures(leftFeat, rightFeat);
% Save shared features
features.leftPoints = vptsLeft(matchIndex(:, 1), :);
features.rightPoints = vptsRight(matchIndex(:, 2), :);
features.leftFeatures = leftFeat(matchIndex(:, 1), :);
features.rightFeatures = rightFeat(matchIndex(:, 2), :);
features.matchMetric = matchMetric;
end