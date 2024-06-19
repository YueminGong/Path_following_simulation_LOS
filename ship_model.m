%% intial parameter
etaship = [0;0;pi/4];
nuship = [0.1;0;0];
tau = [2;0;1];
position = [etaship(1),etaship(2)];
heading =[etaship(3)];
%predict_velocity=2;
%% loop
for i= 1:500
%[etaship,nuship,nushipp_dot] = shipp_model(etaship, nuship, tau); 
%the order is not correct 
[etaship, nuship, nushipp_dot] = shipp_model(nuship, etaship, tau);
position = [position; etaship(1),etaship(2)];
heading = [heading; etaship(3)];
end
figure(1)
plot(position(1:500,1), position(1:500,2),'r.')
xlabel('East m');
ylabel('North m');
figure(2)
plot(1:500,heading(1:500,1),'b.')
xlabel('time s');
ylabel('heading angle');

%% ship dynamic model 
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

if etaship(3)>pi 
     etaship(3)=etaship(3)-2*pi;
end
if etaship(3)<-pi 
     etaship(3)=2*pi+etaship(3);
end

disp(nuship);
end