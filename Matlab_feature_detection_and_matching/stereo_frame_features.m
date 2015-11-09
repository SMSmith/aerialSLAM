function [features] = stereo_frame_features(left_path, right_path)
I1 = rgb2gray(imread(left_path));
I2 = rgb2gray(imread(right_path));
points1 = detectSURFFeatures(I1);
points2 = detectSURFFeatures(I2);
[f1, vpts1] = extractFeatures(I1, points1);
[f2, vpts2] = extractFeatures(I2, points2);
[indexPoints, matchmetric] = matchFeatures(f1, f2);
matchedPoints1 = vpts1(indexPoints(:, 1), :);
matchedPoints2 = vpts2(indexPoints(:, 2), :);
end