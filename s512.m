clear all
close all 
clc

%% initial paramter
etaship = [-0.690; -1.25; 1.78];
nuship = [0.1; 0; 0];
tau = [1;0;0];
predict_velocity = 0.5;

%% trace
% WAY POINT
x=[0.372, -0.628, 0.372, 1.872, 6.872, 8.372, 9.372, 8.372];
y=[-0.181, 1.320, 2.820, 3.320, -0.681, -0.181, 1.320, 2.820];
% ExSpline - Cubic Hermite and spline interpolation of way-points 
wpt.pos.x =[0.372, -0.628, 0.372, 1.872, 6.872, 8.372, 9.372, 8.372];
wpt.pos.y =[-0.181, 1.320, 2.820, 3.320, -0.681, -0.181, 1.320, 2.820];
kpos = wpt.pos.y./wpt.pos.x;
wpt.time = [0 8 16 24 30 36 42 48]; 
t = 0:1:max(wpt.time); % time 
x_p = pchip(wpt.time,wpt.pos.x,t); % cubic Hermite interpolation 
y_p = pchip(wpt.time,wpt.pos.y,t); 
k_p = atan(y_p./x_p);
x_s = spline(wpt.time,wpt.pos.x,t); % spline interpolation 
y_s = spline(wpt.time,wpt.pos.y,t); 
k_s = atan(y_s./x_s);
% subplot(411), plot(wpt.time,wpt.pos.x,'o',t, [x_p; x_s])
% subplot(412), plot(wpt.time,wpt.pos.y, 'o' ,t, [y_p; y_s])
% subplot(413), plot(wpt.pos.y,wpt.pos.x, 'o' ,y_p,x_p,y_s,x_s)
% subplot(414), plot(wpt.time,kpos, 'o' ,t, [k_p; k_s])
x = x_s;
y = y_s;

len = length(x);
trace = [x',y'];
point_storage = [etaship(1), etaship(2)];
angle_storage = etaship(3);
expect_angle_storage = 0;

%los paramter
delta = 3;

%% loop
i=1;
for j=1:20000
    %los
    err_y = trace(i+1,1) - trace(i,1);
    err_x = trace(i+1,2) - trace(i,2);
    whole_angle = atan2(err_y,err_x);
    trans = [cos(whole_angle), -sin(whole_angle); sin(whole_angle), cos(whole_angle)];   
    appendage_coordinate = trans'* [etaship(2)-trace(i,2);etaship(1)-trace(i,1)];
    rang = abs(trans'* [etaship(2) - trace(i+1,2); etaship(1) - trace(i+1,1)]);
    predict_path_angle = whole_angle - atan(appendage_coordinate(2) / delta);
    predict_path_angle = pi/2 - predict_path_angle;
    if appendage_coordinate(1) > 0
        i = i + 1;
        err_integral = [0; 0; 0];
    end
    if i == 49
        break;
    end
    
%     %% pid-->force
%     % PID parameters
%     k_p = 0.18;
%     k_i = 0;%0.0012
%     k_d = 0.41;%0.00068
% 
%     err_current = 0;
%     err_integral = 0;
%     
%     % PID control
%     err_angle = (predict_path_angle - etaship(3)) / pi * 180;
%     err_velocity = predict_velocity - nuship(1);
%     err_last = err_current;
%     err_current = err_angle;
%     err_integral = err_integral + err_current;
% 
%     tau(3) = k_p * err_current + k_i * err_integral + k_d * (err_current - err_last);
%     
%     % Condition & etaship
%     [etaship, nuship, nushipp_dot] = shipp_model(nuship, etaship, tau);
%     point_storage = [point_storage; etaship(1), etaship(2)];
%     angle_storage = [angle_storage; etaship(3)];
%     expect_angle_storage = [expect_angle_storage; predict_path_angle];

%% filter
% Define PID parameters
k_p = 0.18;
k_i = 0; % 0.0012
k_d = 0.41; % 0.00068

% % Define filter parameters
% alpha = 0.1;  % Smoothing factor for the filter

% Initialize variables
err_current = 0;
err_integral = 0;
err_last = 0;
filtered_tau = 0;
filtered_etaship = [0; 0; 0];  % Assuming 3 states [x, y, angle]
filtered_point_storage = [];
filtered_angle_storage = [];

    
% Error calculations
err_angle = (predict_path_angle - etaship(3)) / pi * 180;
err_velocity = predict_velocity - nuship(1);

% PID control
err_last = err_current;
err_current = err_angle;
err_integral = err_integral + err_current;
tau(3) = k_p * err_current + k_i * err_integral + k_d * (err_current - err_last);

% % Apply low-pass filter to control effort
% filtered_tau = alpha * tau(3) + (1 - alpha) * filtered_tau;
% 
% % Apply low-pass filter to state variables
% filtered_etaship = alpha * etaship + (1 - alpha) * filtered_etaship;
% 
% % Update ship state using the filtered control effort
% [etaship, nuship, nushipp_dot] = shipp_model(nuship, etaship, filtered_tau);

% Condition & etaship
[etaship, nuship, nushipp_dot] = shipp_model(nuship, etaship, tau)
point_storage = [point_storage; etaship(1), etaship(2)];
angle_storage = [angle_storage; etaship(3)];
expect_angle_storage = [expect_angle_storage; predict_path_angle];
end




%% draw
figure(1)
plot(trace(:,1),trace(:,2),'b.-');
hold on;
plot(point_storage(:,1),point_storage(:,2),'r');
figure(2)
len_angle = length(angle_storage);
angle_2 = 0:len_angle-1;
angle_storage = [angle_storage,angle_2'];
expect_angle_storage = [expect_angle_storage,angle_2'];
plot(angle_storage(:,2) , angle_storage(:,1),'r');
hold on;
plot(expect_angle_storage(:,2), expect_angle_storage(:,1) ,'b');
%% discussion
% Calculate deviation, stability, and smoothness metrics
deviation = norm(trace(end, :) - point_storage(end, :));
stability = mean(sqrt(sum(diff(point_storage).^2, 2))); % Average Euclidean distance between consecutive points
smoothness = mean(abs(diff(angle_storage(:, 1)))); % Average absolute change in angle

% Display the calculated metrics
disp(['Deviation: ', num2str(deviation)]);
disp(['Stability: ', num2str(stability)]);
disp(['Smoothness: ', num2str(smoothness)]);

%% ship mathematical model
function[etaship, nuship, nushipp_dot] = shipp_model(nuship, etaship, tau)
b = [0;0;0];
dt = 0.02; 
M = [25.8,0,0;0,33.8,1.0115;0,1.0115,2.76];
Minv = inv(M);
Nv = [2,0,0;0,7,0.1;0,0.1,0.5];
aship = etaship(3); 
Rship = [cos(aship) -sin(aship) 0; sin(aship) cos(aship) 0; 0 0 1];
nushipp_dot = Minv * (tau + Rship * b - Nv * nuship);
nushipp = nushipp_dot * dt + nuship;

etashipp_dot_dot = Rship * nushipp_dot;
etashipp_dot = Rship * nuship;
etashipp = etaship + etashipp_dot * dt + 0.5 * etashipp_dot_dot *dt*dt;

nuship = nushipp;
etaship = etashipp;
nushipp_dot = nushipp_dot;
etaship(3) = rem(etaship(3), 2*pi);
% if etaship(3) > pi 
%      etaship(3) = etaship(3)-2 * pi;
% end
% if etaship(3) < - pi 
%      etaship(3) = 2 * pi + etaship(3);
% end

end