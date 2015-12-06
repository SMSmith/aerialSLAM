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
landmark_locations = [];
landmark_output = [];
pose_output = [0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1];
% for i=0:180
% for i=0:633
for i=0:200
% for i=0:4067
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
    indexPairs = matchFeatures(features1, features2);
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
    indexPairsB = matchFeatures(features1B, features2B);
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

    cross_matches = matchFeatures(features1_partial, features1B_partial);
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
    
    cur_pos = path(:, :, end) * T;
    path(:, :, i+1) = cur_pos;
    cur_pos_2 = cur_pos';
    pose_output = [pose_output; [i+1, cur_pos_2(:)']];
    
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
                
%     if i == 0
% %         for j=1:size(best_inlier_indices, 2)
% %             j
% %             landmarks = [landmarks; cross_matched_features(best_inlier_indices(j), :)];
% %         end
%         landmarks = cross_matched_features(best_inlier_indices, :);
%         landmark_locations = matched1_inliers;  % for plotting
%     end

    if mod(i, 200) == 0
        landmarks = [landmarks; cross_matched_features(best_inlier_indices, :)];
        landmark_locations = [landmark_locations; matched1_inliers];  % for plotting
        size(landmarks)
    end
    
    [landmark_matches, landmark_match_metric] = matchFeatures(landmarks, cross_matched_features(best_inlier_indices, :));
    landmarks_and_metric = [landmark_match_metric, single(landmark_matches)];
    landmarks_and_metric = sortrows(landmarks_and_metric, 1);
    size(landmarks_and_metric)
    landmark_ids = [];
    feature_ids = [];
    for j=1:min(5000, size(landmarks_and_metric, 1))  % use at most top 50 landmarks each frame
        landmark_id = landmarks_and_metric(j, 2);
        if landmark_id ~= 5
            continue
        end
        feature_id = landmarks_and_metric(j, 3);
        landmark_ids = [landmark_ids; landmark_id];
        feature_ids = [feature_ids; feature_id];

        if feature_id <= size(matched1_inliers, 1)
            uL = matched1_inliers(feature_id, 1)
            uR = matched2_inliers(feature_id, 1)
            v = matched1_inliers(feature_id, 2)

%                 matched1_inliers(feature_id, 2) - matched2_inliers(feature_id, 2)

%             X = world_points(j, 1);
%             Y = world_points(j, 2);
%             Z = world_points(j, 3);
            X = world_points_inliers(feature_id, 1);
            Y = world_points_inliers(feature_id, 2);
            Z = world_points_inliers(feature_id, 3);
            landmark_output = [landmark_output; [i, landmark_id, uL, uR, v, X, Y, Z]];
        end
    end
%         if i > 33
% %             figure(1);
% %             imshow('../../datasets/cmu_16662_p2/sensor_data/left000.jpg');
% %             hold on;
% %             scatter(landmark_locations(landmark_ids, 1), landmark_locations(landmark_ids, 2), 'ro');
% 
%             figure(2);
%             imshow(I1);
%             hold on;
%             scatter(matched1_inliers(feature_ids, 1), matched1_inliers(feature_ids, 2), 'ro');
%             waitforbuttonpress;
%         end
    
end

%% Write output
dlmwrite('landmark_output.txt', landmark_output, 'delimiter', ' ');
dlmwrite('pose_output.txt', pose_output, 'delimiter', ' ');

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

% scatter3(landmark_output(:, 6), landmark_output(:, 7), landmark_output(:, 8), '.');

% 
% %% Factor Graph
% close all;
% clear all;
% clc;
% addpath('../../gtsam-toolbox-3.2.0-win64/gtsam_toolbox');
% import gtsam.*
% 
% % calib = dlmread('VO_calibration.txt');
% % pose_output = csvread('VO_camera_poses_large.txt');
% % landmark_output = csvread('VO_stereo_factors_large.txt');
% pose_output = csvread('pose_output.txt');
% landmark_output = csvread('landmark_output.txt');
% imu = csvread('preIntegratedIMUTEMP2.csv');
% 
% % init
% graph = NonlinearFactorGraph;
% initial = Values;
% % stereo_model = noiseModel.Diagonal.Sigmas([1.0; 1.0; 1.0]);
% stereo_model = noiseModel.Isotropic.Sigma(3,1);
% % format: fx fy skew cx cy baseline
% K = Cal3_S2Stereo(...
%     164.255034407511, 164.255034407511, 0,...
%     214.523999214172, 119.433252334595, 0.1621);
% % K = Cal3_S2Stereo(calib(1), calib(2), calib(3), calib(4), calib(5), calib(6));
% 
% % add initial poses
% % for i=1:size(pose_output, 1)
% %     pose = Pose3(reshape(pose_output(i,2:17),4,4)');
% %     initial.insert(symbol('x', pose_output(i, 1)), pose);
% % end
% % 
% % % load stereo measurements and initialize landmarks
% % % camera_id landmark_id uL uR v X Y Z
% % for i=1:size(landmark_output,1)
% %     sf = landmark_output(i,:);
% %     graph.add(GenericStereoFactor3D(StereoPoint2(sf(3),sf(4),sf(5)), stereo_model, ...
% %         symbol('x', sf(1)), symbol('l', sf(2)), K));
% %     
% %     if ~initial.exists(symbol('l',sf(2)))
% %         % 3D landmarks are stored in camera coordinates: transform
% %         % to world coordinates using the respective initial pose
% %         pose = initial.at(symbol('x', sf(1)));
% %         world_point = pose.transform_from(Point3(sf(6),sf(7),sf(8)));
% %         initial.insert(symbol('l',sf(2)), world_point);
% %     end
% % end
% 
% % for i=1:size(pose_output, 1)-1
% for i=1:size(imu, 1)
%     odometry = Pose3(reshape(imu(i,2:17),4,4)');
%     if i ~= 1
%         odometry = last_odometry.compose(odometry);
%     end
%     initial.insert(symbol('x', i-1), odometry);
%     last_odometry = odometry;
% %     waitforbuttonpress;
% %     covariance = noiseModel.Diagonal.Sigmas([5*pi/180; 5*pi/180; 5*pi/180; 0.5; 0.5; 0.5]);
% % %     covariance = noiseModel.Diagonal.Sigmas([0; 0; 0; 0; 0; 0;]);
% %     graph.add(BetweenFactorPose3(symbol('x', i-1), symbol('x', i), odometry, covariance));
% end
% 
% % add a constraint on the starting pose
% key = symbol('x', 0);
% first_pose = initial.at(key);
% graph.add(NonlinearEqualityPose3(key, first_pose));
% 
% % optimize
% fprintf(1,'Optimizing\n');
% tic
% optimizer = LevenbergMarquardtOptimizer(graph, initial);
% % result = optimizer.optimizeSafely();
% result = optimizer.optimize();
% toc
% 
% % visualize initial trajectory, final trajectory, and final points
% figure;
% hold on;
% axis equal;
% plot3DTrajectory(initial, 'r', 1, 1);
% plot3DPoints(initial);
% % figure;
% % hold on;
% % axis equal;
% % plot3DTrajectory(result, 'g', 1, 0.1);
% % plot3DPoints(result);
% xlabel('x');
% ylabel('y');
% zlabel('z');
%     
% % %% load the initial poses from VO
% % % row format: camera_id 4x4 pose (row, major)
% % fprintf(1,'Reading data\n');
% % cameras = dlmread(findExampleDataFile('VO_camera_poses_large.txt'));
% % for i=1:size(cameras,1)
% %     pose = Pose3(reshape(cameras(i,2:17),4,4)');
% %     initial.insert(symbol('x',cameras(i,1)),pose);
% % end
% % 
