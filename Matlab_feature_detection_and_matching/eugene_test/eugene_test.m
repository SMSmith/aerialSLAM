clc; clear all; close all;
addpath('estimateRigidTransform');

intrinsic_matrix = [164.255034407511, 0, 0;...
                    0, 164.255034407511, 0;...
                    214.523999214172, 119.433252334595, 1];
camera_params = cameraParameters('IntrinsicMatrix', intrinsic_matrix);
cam1_matrix = cameraMatrix(camera_params, eye(3), [0, 0, 0]);
cam2_matrix = cameraMatrix(camera_params, eye(3), [0.1621, 0, 0]);

path = [eye(4)];
landmarks = [];
landmark_output = [];
pose_output = [];
for i=0:180
% for i=0:633
    ind1 = sprintf('%03d', i)
    ind2 = sprintf('%03d', i+1);
    
    % Find first frame 3d points
    I1 = rgb2gray(imread(strcat('../../datasets/cmu_16662_p2/sensor_data/left', ind1, '.jpg')));
    I2 = rgb2gray(imread(strcat('../../datasets/cmu_16662_p2/sensor_data/right', ind1, '.jpg')));
%     figure;
%     imshow(I1);
%     figure;
%     imshow(I2);

    % feature matching
    points1 = detectSURFFeatures(I1);
    points2 = detectSURFFeatures(I2);
    [features1, valid_points1] = extractFeatures(I1, points1);
    [features2, valid_points2] = extractFeatures(I2, points2);
    indexPairs = matchFeatures(features1, features2);
    matchedPoints1 = valid_points1(indexPairs(:, 1), :);
    matchedPoints2 = valid_points2(indexPairs(:, 2), :);
%     figure;
%     showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);

    world_points = triangulate(matchedPoints1, matchedPoints2, cam1_matrix, cam2_matrix);
%     figure;
%     hold on;
%     axis equal;
%     scatter3(world_points(:, 1), world_points(:, 2), world_points(:, 3));
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');

    %% Landmakrs
    if i == 0
        for j=1:size(indexPairs, 1)
            landmarks = [landmarks; features1(indexPairs(j, 1), :)];
        end
    else
        [landmark_matches, landmark_match_metric] = matchFeatures(landmarks, features1(indexPairs(:, 1), :));
        landmarks_and_metric = [landmark_match_metric, single(landmark_matches)];
        landmarks_and_metric = sort(landmarks_and_metric, 1);
        size(landmarks_and_metric)
        for j=1:min(10, size(landmarks_and_metric, 1))  % use at most top 10 landmarks each frame
            landmark_id = landmarks_and_metric(j, 2);
            feature_id = landmarks_and_metric(j, 3);
            uL = matchedPoints1.Location(feature_id, 1);
            v = matchedPoints1.Location(feature_id, 2);
            uR = matchedPoints2.Location(feature_id, 1);
%             matchedPoints1.Location(feature_id, 2) - matchedPoints2.Location(feature_id, 2)
            X = world_points(j, 1);
            Y = world_points(j, 2);
            Z = world_points(j, 3);
            landmark_output = [landmark_output; [i, landmark_id, uL, uR, v, X, Y, Z]];
        end
%         break;
    end
    
    %% Find second frame 3d points
    I1B = rgb2gray(imread(strcat('../../datasets/cmu_16662_p2/sensor_data/left', ind2, '.jpg')));
    I2B = rgb2gray(imread(strcat('../../datasets/cmu_16662_p2/sensor_data/right', ind2, '.jpg')));
%     figure;
%     imshow(I1B);
%     figure;
%     imshow(I2B);

    % feature matching
    points1B = detectSURFFeatures(I1B);
    points2B = detectSURFFeatures(I2B);
    [features1B, valid_points1B] = extractFeatures(I1B, points1B);
    [features2B, valid_points2B] = extractFeatures(I2B, points2B);
    indexPairsB = matchFeatures(features1B, features2B);
    matchedPoints1B = valid_points1B(indexPairsB(:, 1), :);
    matchedPoints2B = valid_points2B(indexPairsB(:, 2), :);
%     figure;
%     showMatchedFeatures(I1B, I2B, matchedPoints1B, matchedPoints2B);

    world_pointsB = triangulate(matchedPoints1B, matchedPoints2B, cam1_matrix, cam2_matrix);
%     figure;
%     hold on;
%     axis equal;
%     scatter3(world_pointsB(:, 1), world_pointsB(:, 2), world_pointsB(:, 3));
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');

    %% Matching 3d points between frames
    features1_partial = features1(indexPairs(:, 1), :);
    valid_points1_partial = valid_points1(indexPairs(:, 1), :);
    features1B_partial = features1B(indexPairsB(:, 1), :);
    valid_points1B_partial = valid_points1B(indexPairsB(:, 1), :);

    cross_matches = matchFeatures(features1_partial, features1B_partial);
    matched1 = valid_points1_partial(cross_matches(:, 1), :);
    matched2 = valid_points1B_partial(cross_matches(:, 2), :);
%     figure;
%     showMatchedFeatures(I1, I1B, matched1, matched2);

    world_points_partial = world_points(cross_matches(:, 1), :);
    world_pointsB_partial = world_pointsB(cross_matches(:, 2), :);
%     figure;
%     hold on;
%     axis equal;
%     scatter3(world_points_partial(:, 1), world_points_partial(:, 2), world_points_partial(:, 3));
%     scatter3(world_pointsB_partial(:, 1), world_pointsB_partial(:, 2), world_pointsB_partial(:, 3));
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');

    %% estimate transform between frames
    n_iters = 100;
    inlier_thresh = 0.1;
    num_points = size(world_points_partial, 1);
    % RANSAC
    best_inlier_indices = [];
    best_outlier_indices = [];
    for UNUSED=1:n_iters
        random_indices = datasample(1:num_points, 3, 'Replace', false)';
        [tr, ~] = estimateRigidTransform(world_points_partial(random_indices, :)', world_pointsB_partial(random_indices, :)');
        
        % count inliers
        inlier_indices = [];
        outlier_indices = [];
        for j=1:num_points
            a = eye(4);
            a(1:3, 4) = world_points_partial(j, :)';
            b = eye(4);
            b(1:3, 4) = world_pointsB_partial(j, :)';
            c = b*tr;
            d = norm(a(1:2, 4) - c(1:2, 4));
            
            if d < inlier_thresh
                inlier_indices = [inlier_indices, j];
            else
                outlier_indices = [outlier_indices, j];
            end
        end
        if size(inlier_indices, 2) > size(best_inlier_indices, 2)
            best_inlier_indices = inlier_indices;
            best_outlier_indices = outlier_indices;
        end
    end
    size(best_inlier_indices, 2)
    [T, EPS] = estimateRigidTransform(world_points_partial(best_inlier_indices, :)', world_pointsB_partial(best_inlier_indices, :)')
    
    cur_pos = path(:, :, end) * T;
    path(:, :, i+1) = cur_pos;
    cur_pos_2 = cur_pos';
    pose_output = [pose_output; [i, cur_pos_2(:)']];
    
%     world_pointsB_partial_inliers_transformed = [];
%     world_points_partial_inliners = world_points_partial(best_inlier_indices, :);
%     world_pointsB_partial_inliners = world_pointsB_partial(best_inlier_indices, :);
%     for j=1:size(world_pointsB_partial_inliners, 1)
%         t = eye(4);
%         t(1:3, 4) = world_pointsB_partial_inliners(j, :);
%         transformed = t * T;
%         points = transformed(1:3, 4)';
%         world_pointsB_partial_inliers_transformed = [world_pointsB_partial_inliers_transformed; points];
%     end
%     [T_TEMP, EPS] = estimateRigidTransform(world_points_partial_inliners', world_pointsB_partial_inliers_transformed')
%     figure;
%     hold on;
%     axis equal;
%     scatter3(world_points_partial_inliners(:, 1), world_points_partial_inliners(:, 2), world_points_partial_inliners(:, 3));
%     scatter3(world_pointsB_partial_inliers_transformed(:, 1), world_pointsB_partial_inliers_transformed(:, 2), world_pointsB_partial_inliers_transformed(:, 3));
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');
end

%% Write output
dlmwrite('landmark_output.txt', landmark_output, 'delimiter', ' ');
dlmwrite('pose_output.txt', pose_output, 'delimiter', ' ');

%% Plot
% Forward is +x, left is +y, up is +z
figure;
hold on;
axis equal;
xs = -squeeze(path(3, 4, :));
ys = squeeze(path(1, 4, :));
zs = squeeze(path(2, 4, :));
plot3(xs, ys, zs);
xlabel('x');
ylabel('y');
zlabel('z');
