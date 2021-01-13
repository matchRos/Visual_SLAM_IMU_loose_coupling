%% Error_cal_orb.m
% This part is to calculate absolute trajectory error(ATE) of visual SLAM
% by using root-mean-square error(RMSE), in order to evaluate the accuracy
% of visual SLAM. The value of variable 'ATE' is the ATE of visual SLAM.
% Author: Chen, Li
% Email: hustchenli617@gmail.com
% Date: 13-Jan-2021
%% Select dataset
% This part is to select data from experiment, 'cpose09114.bag' is
% experiment data with faster movement, 'cpose09118.bag' is experiment data
% with slower movement.
clear;
slam_name = 'cpose09114.bag';
% slam_name = 'cpose09118.bag';
%% Set beginning and end
% Since slam and FARO measurement cannot be started and ended exactly at
% the same time. There are preparation periods by the beginning and the
% end of slam and FARO measurement, which are supposed to be removed before
% the calculation of ATE.
if slam_name == 'cpose09114.bag'
    meas_name = 'test09114.xlsx';
elseif slam_name == 'cpose09118.bag'
    meas_name = 'test09118.xlsx';
else
    error('Please enter the correct file name.');
end
bag = rosbag(slam_name);
bSel = select(bag,'Topic','/camera_pose');
msgStructs = readMessages(bSel,'DataFormat','struct');
T = readtable(meas_name);
[slam_len,~] = size(msgStructs);
[meas_len,~] = size(T);
if slam_name == 'cpose09114.bag'
    slam_start = 80;
    slam_end = 450;
    meas_start = 850;
    meas_end = 6520;
else
    slam_start = 120;
    slam_end = 750;
    meas_start = 2000;
    meas_end = 8770;
end
x_slam = zeros(slam_len,1);
y_slam = zeros(slam_len,1);
z_slam = zeros(slam_len,1);
for i = 1:slam_len
    pose = msgStructs{i}.('Pose');
    position = pose.('Position');
    x_slam(i) = position.('X');
    y_slam(i) = position.('Y');
    z_slam(i) = position.('Z');
end
x_meas = (T{:,'x'} - T{1,'x'})/1000 + x_slam(1);
y_meas = (T{:,'y'} - T{1,'y'})/1000 + y_slam(1);
z_meas = (T{:,'z'} - T{1,'z'})/1000 + z_slam(1);
x_slam_m = x_slam(slam_start:slam_end);
y_slam_m = y_slam(slam_start:slam_end);
z_slam_m = z_slam(slam_start:slam_end);
x_meas_m = x_meas(meas_start:meas_end);
y_meas_m = y_meas(meas_start:meas_end);
z_meas_m = z_meas(meas_start:meas_end);
%% Transformation from camera to SMR
% Spherically mounted retroreflector(SMR) is stuck on camera, which could
% be regarded as tracking target of FARO vantage laser tracker. Since
% coordinate systems of camera and SMR are not the same. Transformation
% matrix between FARO and camera should be considered.
v1=[mean(x_slam_m),mean(y_slam_m),mean(z_slam_m)];
v2=[mean(x_meas_m),mean(y_meas_m),mean(z_meas_m)];
nv1 = v1/norm(v1);
nv2 = v2/norm(v2);
alpha = norm(v2)/norm(v1);
if norm(nv1+nv2)==0
    q = [0 0 0 0];
else
    u = cross(nv1,nv2);         
    u = u/norm(u);
    theta = acos(sum(nv1.*nv2))/2;
    q = [cos(theta) sin(theta)*u];
end
R = [2*q(1).^2-1+2*q(2)^2  2*(q(2)*q(3)+q(1)*q(4)) 2*(q(2)*q(4)-q(1)*q(3));
    2*(q(2)*q(3)-q(1)*q(4)) 2*q(1)^2-1+2*q(3)^2 2*(q(3)*q(4)+q(1)*q(2));
    2*(q(2)*q(4)+q(1)*q(3)) 2*(q(3)*q(4)-q(1)*q(2)) 2*q(1)^2-1+2*q(4)^2];% transformation matrix between FARO and camera
x_slam_T = zeros(size(x_slam_m));
y_slam_T = zeros(size(y_slam_m));
z_slam_T = zeros(size(z_slam_m));
coordinate_slam = [0, 0, 0];
[x_slam_m_len,~] = size(x_slam_m);
for j = 1:x_slam_m_len
    coordinate_slam(1) = x_slam_m(j);
    coordinate_slam(2) = y_slam_m(j);
    coordinate_slam(3) = z_slam_m(j);
    coordinate_meas = coordinate_slam * R * alpha;
    x_slam_T(j) = coordinate_meas(1);
    y_slam_T(j) = coordinate_meas(2);
    z_slam_T(j) = coordinate_meas(3);
end
%% Calculate ATE
Er = zeros(size(x_slam_T));
[x_slam_T_len,~] = size(x_slam_T);
[x_meas_m_len,~] = size(x_meas_m);
for k = 1:x_slam_T_len
    k_m = round((k-1)/x_slam_T_len*x_meas_m_len)+1;
    Er(k) = norm([x_slam_T(k)-x_meas_m(k_m) y_slam_T(k)-y_meas_m(k_m) z_slam_T(k)-z_meas_m(k_m)]);
end
ATE = sqrt(sum(Er.^2)/x_slam_T_len);% absolute trajectory error [unit:m]
%% Plot the results
% This part is to plot results of visual SLAM as well as FARO measurement
% and compare them.
figure;
plot3(x_slam_T,y_slam_T,z_slam_T);
title('ORB-SLAM');
hold on;
plot3(x_meas_m,y_meas_m,z_meas_m);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
legend('ORB-SLAM','FARO');
figure;
plot([1:x_slam_T_len],x_slam_T,'--');
hold on;
plot([1:x_meas_m_len]*(x_slam_T_len/x_meas_m_len),x_meas_m);
legend('ORB-SLAM','FARO');
xlabel('number of points');
ylabel('x [m]')
title('ORB-SLAM x axis');
figure;
plot([1:x_slam_T_len],y_slam_T,'--');
hold on;
plot([1:x_meas_m_len]*(x_slam_T_len/x_meas_m_len),y_meas_m);
legend('ORB-SLAM','FARO');
xlabel('number of points');
ylabel('y [m]')
title('ORB-SLAM y axis');
figure;
plot([1:x_slam_T_len],z_slam_T,'--');
hold on;
plot([1:x_meas_m_len]*(x_slam_T_len/x_meas_m_len),z_meas_m);
legend('ORB-SLAM','FARO');
xlabel('number of points');
ylabel('z [m]')
title('ORB-SLAM z axis');