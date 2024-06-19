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
subplot(411), plot(wpt.time,wpt.pos.x,'o',t, [x_p; x_s])
subplot(412), plot(wpt.time,wpt.pos.y, 'o' ,t, [y_p; y_s])
subplot(413), plot(wpt.pos.y,wpt.pos.x, 'o' ,y_p,x_p,y_s,x_s)
subplot(414), plot(wpt.time,kpos, 'o' ,t, [k_p; k_s])
xlabel
x = x_s;
y = y_s;