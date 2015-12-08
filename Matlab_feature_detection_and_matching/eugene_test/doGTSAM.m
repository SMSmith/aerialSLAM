close all;
clear all;
clc;

load('../../datasets/cmu_16662_p3/NSHLevel2_data.mat');
load('../../datasets/cmu_16662_p2/sensor_data/hand_carry.mat');

%% GTSAM Setup
addpath('../../gtsam-toolbox-3.2.0-win64/gtsam_toolbox');
import gtsam.*
graph = NonlinearFactorGraph;
initial = Values;

%% Add VO Poses and Landmarks 
pose_output = csvread('pose_output.txt');
landmark_output = csvread('landmark_output.txt');

% stereo_model = noiseModel.Diagonal.Sigmas([2.0; 2.0; 2.0]);
stereo_model = noiseModel.Isotropic.Sigma(3,1);
% format: fx fy skew cx cy baseline
K = Cal3_S2Stereo(...
    164.255034407511, 164.255034407511, 0,...
    214.523999214172, 119.433252334595, 0.1621);

% Init poses
for i=1:size(pose_output, 1)
    pose = Pose3(reshape(pose_output(i,2:17),4,4)');
    initial.insert(symbol('x', pose_output(i, 1)), pose);
end

% Add stereo factors and init landmarks
% camera_id landmark_id uL uR v X Y Z
for i=1:size(landmark_output,1)
    sf = landmark_output(i,:);
    graph.add(GenericStereoFactor3D(StereoPoint2(sf(3),sf(4),sf(5)), stereo_model, ...
        symbol('x', sf(1)), symbol('l', sf(2)), K));
    
    if ~initial.exists(symbol('l',sf(2)))
        % 3D landmarks are stored in camera coordinates: transform
        % to world coordinates using the respective initial pose
        pose = initial.at(symbol('x', sf(1)));
        world_point = pose.transform_from(Point3(sf(6),sf(7),sf(8)));
        initial.insert(symbol('l',sf(2)), world_point);
    end
end

%% IMU Setup
nIMU = size(imu_timestamps, 2);

[x, y, z] = dcm2angle(rotation_imu_to_leftcam, 'XYZ');
imu_leftcam_R = [0, 0, 1; 0, 1 0; -1, 0, 0] * rotation_imu_to_leftcam;

pT = eye(4,4);
pT(1:3, 1:3) = imu_leftcam_R;
% pT(1:3, 4) = translation_imu_to_leftcam;

counter_IMU = 1;
counter_time = 1;
time = imu_timestamps(1,counter_time);
while time < image_timestamps(1,29)
    counter_time = counter_time + 1;
    time = imu_timestamps(1,counter_time);
end
bias = [1.0*mean(body_accel(1,2900:end));1.0*mean(body_accel(2,2900:end));1.0*mean(body_accel(3,2900:end))];
imustd = [std(body_accel(1,2900:end));std(body_accel(2,2900:end));std(body_accel(3,2900:end))];
fprintf('Bias Accel\n');
fprintf('%.7f %.7f %.7f\n',bias);

bias_velw = [mean(body_angvel(1,2900:end));mean(body_angvel(2,2900:end));1*mean(body_angvel(3,2900:end))];
imustd_velw = [std(body_angvel(1,2900:end));std(body_angvel(2,2900:end));std(body_angvel(3,2900:end))];
fprintf('Bias Gyro\n');
fprintf('%.7f %.7f %.7f\n',bias_velw);

%% Add IMU Factors
dV = [0;0;0];
last_pose = eye(4);
for i=2:size(pose_output, 1)
    dP = [0;0;0];
    dR = eye(3,3);
    dt = 0;
    while imu_timestamps(1,counter_IMU) < image_timestamps(1,i)
        if dt == 0
            dt = imu_timestamps(1,counter_IMU) - image_timestamps(1,i - 1);
        else
            dt = imu_timestamps(1,counter_IMU) - imu_timestamps(1,counter_IMU - 1);
        end

        dV = dV + (body_accel(:,counter_IMU)-bias)*dt;
        dP = dP + dV*dt;

        w = zeros(3,3);
        w(1,2) = -1*(body_angvel(3,counter_IMU)-bias_velw(3))*dt;
        w(1,3) = (body_angvel(2,counter_IMU)-bias_velw(2))*dt;
        w(2,1) = (body_angvel(3,counter_IMU)-bias_velw(3))*dt;
        w(2,3) = -1*(body_angvel(1,counter_IMU)-bias_velw(1))*dt;
        w(3,1) = -1*(body_angvel(2,counter_IMU)-bias_velw(2))*dt;
        w(3,2) = (body_angvel(1,counter_IMU)-bias_velw(1))*dt;
        dR = dR*expm(w);

        counter_IMU = counter_IMU + 1;
    end
    dP = dP;
    dTr = [dR,dP;0,0,0,1];
    pT = pT*dTr;
    
    pose = eye(4);
    pose(1:3, 4) = pT(1:3, 4);
    pose(1:3, 1:3) = pT(1:3, 1:3) * inv(imu_leftcam_R);
    T = inv(last_pose) * pose;
    last_pose = pose;
    
    odometry = Pose3(T);
    covariance = noiseModel.Diagonal.Sigmas([3*pi/180; 3*pi/180; 3*pi/180; 0.005*i; 0.005*i; 0.005*i]);
    graph.add(BetweenFactorPose3(symbol('x', i-2), symbol('x', i-1), odometry, covariance));
end

% % %% Summarize IMU data between the previous GPS measurement and now
% %     IMUindices = find(IMUtimes >= t_previous & IMUtimes <= t);
% %     
% %     currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
% %       currentBias, IMU_metadata.AccelerometerSigma.^2 * eye(3), ...
% %       IMU_metadata.GyroscopeSigma.^2 * eye(3), IMU_metadata.IntegrationSigma.^2 * eye(3));
% %     
% %     for imuIndex = IMUindices
% %       accMeas = [ IMU_data(imuIndex).accelX; IMU_data(imuIndex).accelY; IMU_data(imuIndex).accelZ ];
% %       omegaMeas = [ IMU_data(imuIndex).omegaX; IMU_data(imuIndex).omegaY; IMU_data(imuIndex).omegaZ ];
% %       deltaT = IMU_data(imuIndex).dt;
% %       currentSummarizedMeasurement.integrateMeasurement(accMeas, omegaMeas, deltaT);
% %     end
% %     
% %     % Create IMU factor
% %     newFactors.add(ImuFactor( ...
% %       currentPoseKey-1, currentVelKey-1, ...
% %       currentPoseKey, currentVelKey, ...
% %       currentBiasKey, currentSummarizedMeasurement, g, w_coriolis));
% %     
% %     % Bias evolution as given in the IMU metadata
% %     newFactors.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), ...
% %       noiseModel.Diagonal.Sigmas(sqrt(numel(IMUindices)) * sigma_between_b)));

%% Add a constraint on the starting pose
key = symbol('x', pose_output(1, 1));
first_pose = initial.at(key);
graph.add(NonlinearEqualityPose3(key, first_pose));

%% Optimize
fprintf(1,'Optimizing\n');
tic
optimizer = LevenbergMarquardtOptimizer(graph, initial);
% result = optimizer.optimizeSafely();
result = optimizer.optimize();
toc

%% Plot
figure;
hold on;
axis equal;
plot3DTrajectory(initial, 'r', 1, 0.1);
% plot3DPoints(initial);
xlabel('x');
ylabel('y');
zlabel('z');

figure;
hold on;
axis equal;
plot3DTrajectory(result, 'g', 1, 0.1);
% plot3DPoints(result);
xlabel('x');
ylabel('y');
zlabel('z');
