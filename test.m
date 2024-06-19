etaship = [0;0;0];
nuship = [0.1;0;0];
tau = [2;0;0];
a = ship(etaship,nuship,tau);

function[a] = ship(etaship,nuship,tau)
b =[0;0;0];
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

a = nuship;

disp(nuship)
end
