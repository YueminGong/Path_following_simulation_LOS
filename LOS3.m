%% initial paramter
etaship = [0;5;pi/4];
nuship = [0.5;0;0];
tau = [2;0;0];
predict_velocity = 0.5;

%% trace
% WAY POINT
x=[0, 10, 20, 30, 40];
y=[0, 10, 10, 10, 10];
len = length(x);
trace = [x',y'];
point_storage = [etaship(1), etaship(2)];
angle_storage = etaship(3);
expect_angle_storage = 0;
%los paramter
delta = 1.1;

%% loop
i=1;
for j=1:2050
    %los
    err_y = trace(i+1,1) - trace(i,1);
    err_x = trace(i+1,2) - trace(i,2);
    whole_angle = atan2(err_y,err_x);
    trans = [cos(whole_angle), -sin(whole_angle); sin(whole_angle), cos(whole_angle)];   
    appendage_coordinate = trans'* [etaship(2)-trace(i,2);etaship(1)-trace(i,1)];
    rang = abs(trans'* [etaship(2) - trace(i+1,2); etaship(1) - trace(i+1,1)]);
    predict_path_angle = whole_angle - atan(appendage_coordinate(2) / delta);
    predict_path_angle = pi/2 - predict_path_angle;
    if rang(1) < delta
        i = i + 1;
        err_integral = [0; 0; 0];
    end
    
    %% pid-->force
    % PID parameters
    k_p = 0.0006;
    k_i = 0.000001;
    k_d = 0;

    err_current = 0;
    err_integral = 0;
    
    % PID control
    err_angle = (predict_path_angle - etaship(3)) / pi * 180;
    err_velocity = predict_velocity - nuship(1);
    err_last = err_current;
    err_current = err_angle;
    err_integral = err_integral + err_current;

    tau(3) = k_p * err_current + k_i * err_integral + k_d * (err_current - err_last);
    
    % Condition & etaship
    [etaship, nuship, nushipp_dot] = shipp_model(nuship, etaship, tau);
    point_storage = [point_storage; etaship(1), etaship(2)];
    angle_storage = [angle_storage; etaship(3)];
    expect_angle_storage = [expect_angle_storage; predict_path_angle];
end

%% draw
figure(1)
plot(trace(:,1),trace(:,2),'b.-');
hold on;
plot(point_storage(:,1),point_storage(:,2),'r');
% figure(2)
% len_angle = length(angle_storage);
% angle_2 = 0:len_angle-1;
% angle_storage = [angle_storage,angle_2'];
% expect_angle_storage = [expect_angle_storage,angle_2'];
% plot(angle_storage(:,2) , angle_storage(:,1),'r');
% hold on;
% plot(expect_angle_storage(:,2), expect_angle_storage(:,1) ,'b');
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

if etaship(3) > pi 
     etaship(3) = etaship(3)-2 * pi;
end
if etaship(3) < - pi 
     etaship(3) = 2 * pi + etaship(3);
end

end