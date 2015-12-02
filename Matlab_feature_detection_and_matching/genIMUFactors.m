fileID = fopen('preIntegratedIMU.csv','w');
formatSpec = '%i %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n';
nImages = size(image_timestamps,2);
nIMU = size(imu_timestamps,2);
pT = eye(4,4);
P = zeros(3,nImages);
dV = [0;0;0];
counter_IMU = 1;
counter_time = 1;
time = imu_timestamps(1,counter_time);
while time < image_timestamps(1,29)
    counter_time = counter_time + 1;
    time = imu_timestamps(1,counter_time);
end
bias = [mean(body_accel(1,1:counter_time));mean(body_accel(2,1:counter_time));mean(body_accel(3,1:counter_time))];
imustd = [std(body_accel(1,1:counter_time));std(body_accel(2,1:counter_time));std(body_accel(3,1:counter_time))];
fprintf('Bias Accel\n');
disp(bias)
fprintf('Accel std\n');
disp(imustd)

bias_velw = [mean(body_angvel(1,1:counter_time));mean(body_angvel(2,1:counter_time));mean(body_angvel(3,1:counter_time))];
imustd_velw = [std(body_angvel(1,1:counter_time));std(body_angvel(2,1:counter_time));std(body_angvel(3,1:counter_time))];
fprintf('Bias Gyro\n');
disp(bias_velw)
fprintf('Gyro std\n');
disp(imustd_velw)

for label = 2:nImages
    dP = [0;0;0];  
    dR = eye(3,3);
    dt = 0;
    while imu_timestamps(1,counter_IMU) < image_timestamps(1,label)
        if dt == 0
            dt = imu_timestamps(1,counter_IMU) - image_timestamps(1,label - 1);
        else
            dt = imu_timestamps(1,counter_IMU) - imu_timestamps(1,counter_IMU - 1);
        end
        dP = dP + dV*dt;        
        dV = dV + dR*(body_accel(:,counter_IMU)-bias)*dt;
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
    Tn = pT*[dR,dP/10;0,0,0,1];
    P(:,label) = Tn(1:3,4);
    %fprintf(formatSpec,label-2,Tn');
    fprintf(fileID,formatSpec,label-2,Tn');
    pT = Tn;
end
fclose(fileID);
%plot3(P(1,:),P(2,:),P(3,:));