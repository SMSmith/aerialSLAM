% TODO: Write a description of this file
function [matchFeat] = getFrameTF(features_0, features_1)
% Get features from each left image of the respective frames
feat_0 = features_0.leftFeatures;
feat_1 = features_1.leftFeatures;
% Compare features with previous Image
[matchIndex, matchMetric] = matchFeatures(feat_0, feat_1);
% Save matches to return
features_0.leftPoints = features_0.leftPoints(matchIndex(:, 1), :);
features_0.rightPoints = features_0.rightPoints(matchIndex(:, 1), :);
features_0.leftFeatures = features_0.leftFeatures(matchIndex(:, 1), :);
features_0.rightFeatures = features_0.rightFeatures(matchIndex(:, 1), :);
features_0.matchMetric = features_0.matchMetric(matchIndex(:, 1), :);
% features_0.distances = features_0.distances(matchIndex(:, 1), :);
features_1.leftPoints = features_1.leftPoints(matchIndex(:, 2), :);
features_1.rightPoints = features_1.rightPoints(matchIndex(:, 2), :);
features_1.leftFeatures = features_1.leftFeatures(matchIndex(:, 2), :);
features_1.rightFeatures = features_1.rightFeatures(matchIndex(:, 2), :);
features_1.matchMetric = features_1.matchMetric(matchIndex(:, 2), :);
% features_1.distances = features_1.distances(matchIndex(:, 2), :);
matchFeat.frame0 = features_0;
matchFeat.frame1 = features_1;
matchFeat.matchMetric = matchMetric;
end