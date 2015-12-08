clc; clear all; close all;
addpath('estimateRigidTransform');

intrinsic_matrix = [164.255034407511, 0, 0;...
                    0, 164.255034407511, 0;...
                    214.523999214172, 119.433252334595, 1];
camera_params = cameraParameters('IntrinsicMatrix', intrinsic_matrix);
cam1_matrix = cameraMatrix(camera_params, eye(3), [0, 0, 0]);
cam2_matrix = cameraMatrix(camera_params, eye(3), [0.1621, 0, 0]);

path = [eye(4)];
landmarks = {};
frame_features = {};
frame_landmark_ids = {};
frame_uvXYZs = {};
landmark_idx_count = [];
landmark_locations = [];
landmark_output = [];
pose_output = [0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1];
end_idx = 633;
Ts = zeros(4, 4, end_idx+1);
parfor i=0:end_idx
    ind1 = sprintf('%03d', i)
    ind2 = sprintf('%03d', i+1);
%     ind1 = sprintf('%04d', i)
%     ind2 = sprintf('%04d', i+1);
    
    
    % Find first frame 3d points
    I1 = rgb2gray(imread(strcat('../../datasets/cmu_16662_p2/sensor_data/left', ind1, '.jpg')));
    I2 = rgb2gray(imread(strcat('../../datasets/cmu_16662_p2/sensor_data/right', ind1, '.jpg')));
%     I1 = rgb2gray(imread(strcat('../../datasets/cmu_16662_p3/NSHLevel2_Images/NSHLevel2_Images/NSHLevel2_Images/left', ind1, '.jpg')));
%     I2 = rgb2gray(imread(strcat('../../datasets/cmu_16662_p3/NSHLevel2_Images/NSHLevel2_Images/NSHLevel2_Images/right', ind1, '.jpg')));
%     figure;
%     imshow(I1);
%     figure;
%     imshow(I2);

    % feature matching
    points1 = detectSURFFeatures(I1);
    points2 = detectSURFFeatures(I2);
    [features1, valid_points1] = extractFeatures(I1, points1);
    [features2, valid_points2] = extractFeatures(I2, points2);
    indexPairs = matchFeatures(features1, features2, 'Unique', true);
    matchedPoints1 = valid_points1(indexPairs(:, 1), :);
    matchedPoints2 = valid_points2(indexPairs(:, 2), :);
%     figure;
%     showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);

    world_points = triangulate(matchedPoints1, matchedPoints2, cam1_matrix, cam2_matrix);
    world_points = [-world_points(:, 1), -world_points(:, 2), -world_points(:, 3)];
%     world_points = [-world_points(:, 3), world_points(:, 1), world_points(:, 2)];

%     figure;
%     hold on;
%     axis equal;
%     scatter3(world_points(:, 1), world_points(:, 2), world_points(:, 3));
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');
%     pause;
    
    %% Find second frame 3d points
    I1B = rgb2gray(imread(strcat('../../datasets/cmu_16662_p2/sensor_data/left', ind2, '.jpg')));
    I2B = rgb2gray(imread(strcat('../../datasets/cmu_16662_p2/sensor_data/right', ind2, '.jpg')));
%     figure;
%     I1B = rgb2gray(imread(strcat('../../datasets/cmu_16662_p3/NSHLevel2_Images/NSHLevel2_Images/NSHLevel2_Images/left', ind2, '.jpg')));
%     I2B = rgb2gray(imread(strcat('../../datasets/cmu_16662_p3/NSHLevel2_Images/NSHLevel2_Images/NSHLevel2_Images/right', ind2, '.jpg')));
%     imshow(I1B);
%     figure;
%     imshow(I2B);

    % feature matching
    points1B = detectSURFFeatures(I1B);
    points2B = detectSURFFeatures(I2B);
    [features1B, valid_points1B] = extractFeatures(I1B, points1B);
    [features2B, valid_points2B] = extractFeatures(I2B, points2B);
    indexPairsB = matchFeatures(features1B, features2B, 'Unique', true);
    matchedPoints1B = valid_points1B(indexPairsB(:, 1), :);
    matchedPoints2B = valid_points2B(indexPairsB(:, 2), :);
%     figure;
%     showMatchedFeatures(I1B, I2B, matchedPoints1B, matchedPoints2B);

    world_pointsB = triangulate(matchedPoints1B, matchedPoints2B, cam1_matrix, cam2_matrix);
    world_pointsB = [-world_pointsB(:, 1), -world_pointsB(:, 2), -world_pointsB(:, 3)];
%     world_pointsB = [-world_pointsB(:, 3), world_pointsB(:, 1), world_pointsB(:, 2)];
%     figure;
%     hold on;
%     axis equal;
%     scatter3(world_pointsB(:, 1), world_pointsB(:, 2), world_pointsB(:, 3));
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');

    %% Matching 3d points between frames
    features1_partial = features1(indexPairs(:, 1), :);
%     valid_points1_partial = valid_points1(indexPairs(:, 1), :);
    features1B_partial = features1B(indexPairsB(:, 1), :);
%     valid_points1B_partial = valid_points1B(indexPairsB(:, 1), :);

    cross_matches = matchFeatures(features1_partial, features1B_partial, 'Unique', true);
    matched1 = matchedPoints1(cross_matches(:, 1), :);
    matched2 = matchedPoints2(cross_matches(:, 1), :);
%     matched1B = matchedPoints1B(cross_matches(:, 2), :);
%     figure;
%     showMatchedFeatures(I1, I1B, matched1, matched1B);

    world_points_partial = world_points(cross_matches(:, 1), :);
    world_pointsB_partial = world_pointsB(cross_matches(:, 2), :);
    cross_matched_features = features1_partial(cross_matches(:, 1), :);
%     figure;
%     hold on;
%     axis equal;
%     scatter3(world_points_partial(:, 1), world_points_partial(:, 2), world_points_partial(:, 3));
%     scatter3(world_pointsB_partial(:, 1), world_pointsB_partial(:, 2), world_pointsB_partial(:, 3));
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');

    %% estimate transform between frames
    n_iters = 200;
    inlier_thresh = 0.1;
    num_points = size(world_points_partial, 1);
    % RANSAC
    best_inlier_indices = [];
    best_outlier_indices = [];
    for UNUSED=1:n_iters
        random_indices = datasample(1:num_points, 5, 'Replace', false)';
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
    
    Ts(:, :, i+1) = T;

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

    %% Landmarks
    matched1_inliers = matched1.Location(best_inlier_indices, :); 
    matched2_inliers = matched2.Location(best_inlier_indices, :);
    world_points_inliers = world_points_partial(best_inlier_indices, :);
    
    cur_frame_features = cross_matched_features(best_inlier_indices, :);
    cur_frame_num_features = size(cur_frame_features, 1);
    
    cur_frame_feature_indices = [];
    cur_frame_uvXYZs = [];
    for feature_id=1:cur_frame_num_features
       
        uL = matched1_inliers(feature_id, 1);
        uR = matched2_inliers(feature_id, 1);
        vL = matched1_inliers(feature_id, 2);
        vR = matched2_inliers(feature_id, 2);

        X = world_points_inliers(feature_id, 1);
        Y = world_points_inliers(feature_id, 2);
        Z = world_points_inliers(feature_id, 3);       
        
        cur_frame_uvXYZs = [cur_frame_uvXYZs; [uL, uR, (vL + vR) / 2, X, Y, Z]];
    end
    
    frame_features{i+1} = cur_frame_features;
    frame_landmark_ids{i+1} = zeros(cur_frame_num_features, 1) - 1;
    frame_uvXYZs{i+1} = cur_frame_uvXYZs;
end

%% Compute poses
cur_pos = eye(4);
for i=1:end_idx+1
    cur_pos = cur_pos * Ts(:, :, i);
    path(:, :, i+1) = cur_pos;
    cur_pos_2 = cur_pos';
    pose_output = [pose_output; [i, cur_pos_2(:)']];
end

%% Generate landmarks
LANDMARKS_PER_FRAME_PAIR = 3;
SLIDING_WINDOW_SIZE = 5;

landmark_idx_count = 0;
landmark_output = [];
for frame1_idx=0:end_idx-1
    frame1_idx
    frame1_features = frame_features{frame1_idx+1};
    for frame2_idx=frame1_idx+1:min(frame1_idx+SLIDING_WINDOW_SIZE, end_idx)
        frame2_features = frame_features{frame2_idx+1};
        
        [feature_matches, match_metric] = matchFeatures(frame1_features, frame2_features, 'Unique', true);
        
%         % Skip frame pair if not enough feature matches
%         if size(feature_matches, 1) < 50
%             continue
%         end
        
        metric_and_matches = [single(feature_matches), match_metric];
        metric_and_matches = sortrows(metric_and_matches, 3);
        
        for i=1:LANDMARKS_PER_FRAME_PAIR
            if i > size(metric_and_matches, 1)
                break;
            end
            frame1_feature_id = metric_and_matches(i, 1);
            frame2_feature_id = metric_and_matches(i, 2);

            frame1_landmark_id = frame_landmark_ids{frame1_idx+1}(frame1_feature_id);
            if frame1_landmark_id == -1  % We haven't dealt with this feature yet
                cur_landmark_idx = landmark_idx_count;
                landmark_idx_count = landmark_idx_count + 1;                

                % Set frame1 landmark index
                frame_landmark_ids{frame1_idx+1}(frame1_feature_id) = cur_landmark_idx;
                % Add frame1 landmark
                frame1_uvXYZ = frame_uvXYZs{frame1_idx+1}(frame1_feature_id, :);
                landmark_output = [landmark_output; [frame1_idx, cur_landmark_idx, frame1_uvXYZ]];
            else  % We have already taken care of this feature in a previous iteration
                cur_landmark_idx = frame1_landmark_id;
            end
            % Set frame2 landmark index
            frame_landmark_ids{frame2_idx+1}(frame2_feature_id) = cur_landmark_idx;
            % Add frame2 landmark
            frame2_uvXYZ = frame_uvXYZs{frame2_idx+1}(frame2_feature_id, :);
            landmark_output = [landmark_output; [frame2_idx, cur_landmark_idx, frame2_uvXYZ]];
        end
    end
end

%% Write output
dlmwrite('landmark_output.txt', landmark_output, 'delimiter', ' ');
dlmwrite('pose_output.txt', pose_output(1:end-1, :), 'delimiter', ' ');

%% Plot
% Forward is +x, left is +y, up is +z
figure;
hold on;
axis equal;
xs = squeeze(path(1, 4, :));
ys = squeeze(path(2, 4, :));
zs = squeeze(path(3, 4, :));
plot3(xs, ys, zs);
xlabel('x');
ylabel('y');
zlabel('z');
