% Get the shared features between two stereo images
% INPUT: leftFile - path to the left camera's image for this frame
% INPUT: rightFile - path to the right camera's image for this frame
% INPUT: stereoParams - camera parameters
% IN
% OUTPUT: features - structure containing:
%                       leftPoints - SURF points in left image
%                       rightPoints - SURF points in right image
%                       leftFeatures - Features in left image
%                       rightFeatures - Features in right image
%                       matchMetric - how well the feature match
%                       distances - how far each feature is in this frame
function [features] = getStereoFeatures(leftFile, rightFile) %, stereoParams)
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
% Calculate the distance to each feature
% numPoints = size(matchIndex, 1);
% distances = zeros(numPoints, 3);
% for i = 1:numPoints
%     center1 = features.leftPoints(i);
%     center2 = features.rightPoints(i);
%     distances(i,:) = triangulate(center1, center2, stereoParams);
% end
% features.distances = distances;
end