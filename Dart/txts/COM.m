clc; clear; close all;

%% LIP
xlip = importdata('LIP\x.txt');
xlip_m = importdata('LIP\x_m.txt');
ylip = importdata('LIP\y.txt');
ylip_m = importdata('LIP\y_m.txt');

xzlip = importdata('LIP\xz.txt');
xzlip_m = importdata('LIP\xz_m_cop.txt');
yzlip = importdata('LIP\yz.txt');
yzlip_m = importdata('LIP\yz_m_cop.txt');

xdlip = importdata('LIP\xd.txt');
xdlip_m = importdata('LIP\xd_m.txt');
ydlip = importdata('LIP\yd.txt');
ydlip_m = importdata('LIP\yd_m.txt');


%% VIP (partial feedback position only)
% x = importdata('VIP(partial feedback position only ant)\x.txt');
% x_m = importdata('VIP(partial feedback position only ant)\x_m.txt');
% y = importdata('VIP(partial feedback position only ant)\y.txt');
% y_m = importdata('VIP(partial feedback position only ant)\y_m.txt');

% xd = importdata('VIP(partial feedback position only ant)\xd.txt');
% xdcam = importdata('VIP(partial feedback position only ant)\xdcam.txt');
% xd_m = importdata('VIP(partial feedback position only ant)\xd_m.txt');
% yd = importdata('VIP(partial feedback position only ant)\yd.txt');
% ydcam = importdata('VIP(partial feedback position only ant)\ydcam.txt');
% yd_m = importdata('VIP(partial feedback position only ant)\yd_m.txt');


% xz = importdata('VIP(partial feedback position only ant)\xz.txt');
% xz_m = importdata('VIP(partial feedback position only ant)\xz_m_cop.txt');
% yz = importdata('VIP(partial feedback position only ant)\yz.txt');
% yz_m = importdata('VIP(partial feedback position only ant)\yz_m_cop.txt');


% xcam = importdata('VIP(partial feedback position only ant)\xcam.txt');
% ycam = importdata('VIP(partial feedback position only ant)\ycam.txt');
% xzcam = importdata('VIP(partial feedback position only ant)\xzcam.txt');
% yzcam = importdata('VIP(partial feedback position only ant)\yzcam.txt');

% desiredTorsoPosition_x = importdata('VIP(partial feedback position only ant)\desiredTorsoPosition_x.txt');
% desiredTorsoPosition_y = importdata('VIP(partial feedback position only ant)\desiredTorsoPosition_y.txt');
% desiredTorsoPosition_z = importdata('VIP(partial feedback position only ant)\desiredTorsoPosition_z.txt');

% desiredTorsoVelocity_x = importdata('VIP(partial feedback position only ant)\desiredTorsoVelocity_x.txt');
% desiredTorsoVelocity_y = importdata('VIP(partial feedback position only ant)\desiredTorsoVelocity_y.txt');
% desiredTorsoVelocity_z = importdata('VIP(partial feedback position only ant)\desiredTorsoVelocity_z.txt');

%%VIP(partial feedback pos,vel,acc,zmp)

% x = importdata('VIP(partial feedback pos,vel,acc,zmp)\x.txt');
% x_m = importdata('VIP(partial feedback pos,vel,acc,zmp)\x_m.txt');
% y = importdata('VIP(partial feedback pos,vel,acc,zmp)\y.txt');
% y_m = importdata('VIP(partial feedback pos,vel,acc,zmp)\y_m.txt');

% xd = importdata('VIP(partial feedback pos,vel,acc,zmp)\xd.txt');
% xdcam = importdata('VIP(partial feedback pos,vel,acc,zmp)\xdcam.txt');
% xd_m = importdata('VIP(partial feedback pos,vel,acc,zmp)\xd_m.txt');
% yd = importdata('VIP(partial feedback pos,vel,acc,zmp)\yd.txt');
% ydcam = importdata('VIP(partial feedback pos,vel,acc,zmp)\ydcam.txt');
% yd_m = importdata('VIP(partial feedback pos,vel,acc,zmp)\yd_m.txt');


% xz = importdata('VIP(partial feedback pos,vel,acc,zmp)\xz.txt');
% xz_m = importdata('VIP(partial feedback pos,vel,acc,zmp)\xz_m_cop.txt');
% yz = importdata('VIP(partial feedback pos,vel,acc,zmp)\yz.txt');
% yz_m = importdata('VIP(partial feedback pos,vel,acc,zmp)\yz_m_cop.txt');


% xcam = importdata('VIP(partial feedback pos,vel,acc,zmp)\xcam.txt');
% ycam = importdata('VIP(partial feedback pos,vel,acc,zmp)\ycam.txt');
% xzcam = importdata('VIP(partial feedback pos,vel,acc,zmp)\xzcam.txt');
% yzcam = importdata('VIP(partial feedback pos,vel,acc,zmp)\yzcam.txt');


% desiredTorsoPosition_x = importdata('VIP(partial feedback pos,vel,acc,zmp)\desiredTorsoPosition_x.txt');
% desiredTorsoPosition_y = importdata('VIP(partial feedback pos,vel,acc,zmp)\desiredTorsoPosition_y.txt');
% desiredTorsoPosition_z = importdata('VIP(partial feedback pos,vel,acc,zmp)\desiredTorsoPosition_z.txt');

% desiredTorsoVelocity_x = importdata('VIP(partial feedback pos,vel,acc,zmp)\desiredTorsoVelocity_x.txt');
% desiredTorsoVelocity_y = importdata('VIP(partial feedback pos,vel,acc,zmp)\desiredTorsoVelocity_y.txt');
% desiredTorsoVelocity_z = importdata('VIP(partial feedback pos,vel,acc,zmp)\desiredTorsoVelocity_z.txt');

%% VIP(partial feedback position only+angle)

% x = importdata('VIP(partial feedback position only+angle)\x.txt');
% x_m = importdata('VIP(partial feedback position only+angle)\x_m.txt');
% y = importdata('VIP(partial feedback position only+angle)\y.txt');
% y_m = importdata('VIP(partial feedback position only+angle)\y_m.txt');

% xd = importdata('VIP(partial feedback position only+angle)\xd.txt');
% xdcam = importdata('VIP(partial feedback position only+angle)\xdcam.txt');
% xd_m = importdata('VIP(partial feedback position only+angle)\xd_m.txt');
% yd = importdata('VIP(partial feedback position only+angle)\yd.txt');
% ydcam = importdata('VIP(partial feedback position only+angle)\ydcam.txt');
% yd_m = importdata('VIP(partial feedback position only+angle)\yd_m.txt');


% xz = importdata('VIP(partial feedback position only+angle)\xz.txt');
% xz_m = importdata('VIP(partial feedback position only+angle)\xz_m_cop.txt');
% yz = importdata('VIP(partial feedback position only+angle)\yz.txt');
% yz_m = importdata('VIP(partial feedback position only+angle)\yz_m_cop.txt');


% xcam = importdata('VIP(partial feedback position only+angle)\xcam.txt');
% ycam = importdata('VIP(partial feedback position only+angle)\ycam.txt');
% xzcam = importdata('VIP(partial feedback position only+angle)\xzcam.txt');
% yzcam = importdata('VIP(partial feedback position only+angle)\yzcam.txt');


% desiredTorsoPosition_x = importdata('VIP(partial feedback position only+angle)\desiredTorsoPosition_x.txt');
% desiredTorsoPosition_y = importdata('VIP(partial feedback position only+angle)\desiredTorsoPosition_y.txt');
% desiredTorsoPosition_z = importdata('VIP(partial feedback position only+angle)\desiredTorsoPosition_z.txt');

% desiredTorsoVelocity_x = importdata('VIP(partial feedback position only+angle)\desiredTorsoVelocity_x.txt');
% desiredTorsoVelocity_y = importdata('VIP(partial feedback position only+angle)\desiredTorsoVelocity_y.txt');
% desiredTorsoVelocity_z = importdata('VIP(partial feedback position only+angle)\desiredTorsoVelocity_z.txt');

% virt_torqx = importdata('VIPmatlab\virt_torqx.txt');
% virt_torqy = importdata('VIPmatlab\virt_torqy.txt');

%% no title plots

x = importdata('x.txt');
x_m = importdata('x_m.txt');
y = importdata('y.txt');
y_m = importdata('y_m.txt');

xd = importdata('xd.txt');
xdcam = importdata('xdcam.txt');
xd_m = importdata('xd_m.txt');
yd = importdata('yd.txt');
ydcam = importdata('ydcam.txt');
yd_m = importdata('yd_m.txt');


xz = importdata('xz.txt');
xz_m = importdata('xz_m_cop.txt');
yz = importdata('yz.txt');
yz_m = importdata('yz_m_cop.txt');


xcam = importdata('xcam.txt');
ycam = importdata('ycam.txt');
xzcam = importdata('xzcam.txt');
yzcam = importdata('yzcam.txt');


desiredTorsoPosition_x = importdata('desiredTorsoPosition_x.txt');
desiredTorsoPosition_y = importdata('desiredTorsoPosition_y.txt');
desiredTorsoPosition_z = importdata('desiredTorsoPosition_z.txt');

desiredTorsoVelocity_x = importdata('desiredTorsoVelocity_x.txt');
desiredTorsoVelocity_y = importdata('desiredTorsoVelocity_y.txt');
desiredTorsoVelocity_z = importdata('desiredTorsoVelocity_z.txt');


% % Plot the centre of mass trajectories
% figure
% plot(xdlip);
% hold on
% plot(xdlip_m);
% legend('desired','actual','Location','northeast','Interpreter','latex')
% xlabel('Time');
% ylabel('Velocities [m/s]');
% title('LIP model');
% grid on;

% figure;
% plot(ydlip);
% hold on
% plot(ydlip_m);
% legend('desired','actual','Location','northeast','Interpreter','latex')
% xlabel('Time');
% ylabel('Velocities [m/s]');
% title('LIP model');
% grid on;

% figure;
% plot(xzlip,yzlip);
% hold on
% plot(xzlip_m, yzlip_m);
% legend('desired','actual','Location','northeast','Interpreter','latex')
% xlabel('Xz [M]');
% ylabel('Yz [M]');
% title('LIP model');
% grid on;
if(true)
	% disp(size(x));
	% disp(size(y));
	% disp(size(x_m));
	% disp(size(y_m));
xtot_store = importdata('xtot_store.txt');
ytot_store = importdata('ytot_store.txt');
xz_store = importdata('xz_store.txt');
yz_store = importdata('yz_store.txt');
xztot_store = importdata('xztot_store.txt');
yztot_store = importdata('yztot_store.txt');
thetax = importdata('thetax.txt');
thetay = importdata('thetay.txt');
thetaz = importdata('thetaz.txt');

xwihtnoise = importdata('xwithnoise.txt');
ywithnoise = importdata('ywithnoise.txt');
xzwihtnoise = importdata('xzwithnoise.txt');
yzwithnoise = importdata('yzwithnoise.txt');




% figure;
% plot(x,y);
% hold on;
% plot(xz,yz);
% hold on;
% plot(xtot_store,ytot_store);
% hold on;
% plot(xz_store,yz_store);
% legend('COM','ZMP','Matlab COM','Matlab ZMP','Location','northeast','Interpreter','latex')
% xlabel('X [M]');
% ylabel('Y [M]');
% title('VIP model(partial feedback position only)');
% grid on;

% figure;
% plot(xwihtnoise+xcam,ywithnoise+ycam);
% hold on;
% plot(xzwihtnoise+xzcam,yzwithnoise+yzcam);
% hold on;
% plot(xtot_store,ytot_store);
% hold on;
% plot(xztot_store,yztot_store);
% legend('COM','ZMP','Matlab COM','Matlab ZMP','Location','northeast','Interpreter','latex')
% xlabel('X [M]');
% ylabel('Y [M]');
% grid on;

% figure;
% plot(xzcam);
% hold on;
% plot(xztot_store-xz_store);
% legend('Dart','Matlab','Location','northeast','Interpreter','latex')
% xlabel('X [M]');
% ylabel('Y [M]');
% grid on;

% figure;
% plot(yzcam);
% hold on;
% plot(yztot_store-yz_store);
% legend('Dart','Matlab','Location','northeast','Interpreter','latex')
% xlabel('X [M]');
% ylabel('Y [M]');
% grid on;

% figure;
% subplot(1,3,1);
% plot(desiredTorsoPosition_x);
% hold on;
% plot(thetax);
% xlabel('time (samples)');
% ylabel('angle (rad)');
% legend('Dart','Matlab','Location','northeast','Interpreter','latex')
% title('Rotation around x-axis');
% grid on
% subplot(1,3,2);
% plot(desiredTorsoPosition_y);
% hold on;
% plot(thetay);
% xlabel('time (samples)');
% ylabel('angle (rad)');
% legend('Dart','Matlab','Location','northeast','Interpreter','latex')
% title('Rotation around y-axis');
% grid on
% subplot(1,3,3);
% plot(desiredTorsoPosition_z);
% hold on;
% plot(thetaz);
% xlabel('time (samples)');
% ylabel('angle (rad)');
% legend('Dart','Matlab','Location','northeast','Interpreter','latex')
% title('Rotation around z-axis');
% grid on


figure;
plot(x,y);
hold on
plot(xz, yz)
legend('COM','ZMP','Location','northeast','Interpreter','latex')
xlabel('X [M]');
ylabel('Y [M]');
grid on;

% figure;
% plot(xtot_store, ytot_store, 'r', 'Linewidth', 2);
% hold on;
% plot(xz_store, yz_store, 'b', 'Linewidth', 2);
% plot(xztot_store, yztot_store,'k --', 'Linewidth', 1);
% grid on
% xlabel('x (m)');
% ylabel('y (m)');
% legend('CoM','ZMP','CMP','Location','northeast')

else
%%

figure;
plot(x+xcam,y+ycam);
hold on
plot(x_m, y_m);
legend('desired','actual','Location','northeast','Interpreter','latex')
xlabel('X [M]');
ylabel('Y [M]');
title('VIP model(partial feedback position only)');
grid on;

figure;
plot(xz+xzcam,yz+yzcam);
hold on
plot(xz_m, yz_m);
legend('desired','actual','Location','northeast','Interpreter','latex')
xlabel('Xz [M]');
ylabel('Yz [M]');
title('VIP model(partial feedback position only)');
grid on;



figure;
plot(xd+xdcam);
hold on
plot(xd_m);
legend('desired','actual','Location','northeast','Interpreter','latex')
xlabel('Time');
ylabel('Velocities [m/s]');
title('VIP model(partial feedback position only)');
grid on;

figure;
plot(yd+ydcam);
hold on
plot(yd_m);
legend('desired','actual','Location','northeast','Interpreter','latex')
xlabel('Time');
ylabel('Velocities [m/s]');
title('VIP model(partial feedback position only)');
grid on;


figure;
plot(desiredTorsoPosition_x);
xlabel('Time');
ylabel('Torso Angle x [rad]');
title('VIP model(partial feedback position only)');
grid on;

figure;
plot(desiredTorsoPosition_y);
xlabel('Time');
ylabel('Torso Angle y [rad]');
title('VIP model(partial feedback position only)');
grid on;

figure;
plot(desiredTorsoVelocity_x);
xlabel('Time');
ylabel('Torso Angular velocity x [rad/s]');
title('VIP model(partial feedback position only)');
grid on;

figure;
plot(desiredTorsoVelocity_y);
xlabel('Time');
ylabel('Torso Angular velocity y [rad/s]');
title('VIP model(partial feedback position only)');
grid on;



figure;
plot(xzcam);
hold on;
plot(yzcam)
xlabel('Time');
ylabel('CAM ZMPs[M]');
title('VIP model(partial feedback position only)');
grid on;




%% NOISE
% for i = 1:300
% %     if(mod(i,50))
% %         flag = 0; 
% %     end
% %     if(i<50)
% %         virt_torq(1,i) = -0.005*sin(i*0.01*pi);
% %     else
% %         if(i<50+ssSamples)
% %             virt_torq(1,i) = -0.005+0.008*sin((i-50)*(1/115)*2*pi);
% %         end
% %     end
% %     if(flag)
% %         virt_torq(2,i) = -0.001*sin(i*0.01*2*pi-4*pi/5);
% %         i = i+ssSamples;
% %         flag = 0;
% %         continue;
% %         %             else
% %         %             virt_torq(1,i) = 0.003*cos((i-100)*0.01*1.83*2*pi);
% %     end
%     NOISE(1,i+50) = -0.0005*sin((i+50)*0.01*4*pi-pi);

%     NOISE(2,i) = -0.0005*sin(i*0.01*2*pi-4*pi/5);

% end
% NOISE = NOISE(:,1:300);
% virt_torqx = virt_torqx(1:300);
% virt_torqy = virt_torqy(1:300);

% figure
% subplot(1,2,1);
% plot(virt_torqx);
% xlabel('time [samples]');
% ylabel('CoM [m]');
% title('CoM x-axis');
% grid on
% subplot(1,2,2);
% plot(virt_torqy);
% xlabel('time [samples]');
% ylabel('CoM [m]');
% title('CoM y-axis');
% grid on


% figure
% subplot(1,2,1);
% plot(NOISE(1,:));
% xlabel('time [samples]');
% ylabel('CoM [m]');
% title('CoM x-axis');
% grid on
% subplot(1,2,2);
% plot(NOISE(2,:));
% xlabel('time [samples]');
% ylabel('CoM [m]');
% title('CoM y-axis');
% grid on
end