function [features, matchMetric] = getStereoFeatures(leftFile, rightFile)
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
% Get similar features
matchedPointsLeft = vptsLeft(matchIndex(:, 1), :);
matchedPointsRight = vptsRight(matchIndex(:, 2), :);
% Save shared features
features.leftPoints = matchedPointsLeft;
features.rightPoints = matchedPointsRight;
features.leftFeatures = leftFeat(matchIndex(:, 1), :);
features.rightFeatures = rightFeat(matchIndex(:, 2), :);
end