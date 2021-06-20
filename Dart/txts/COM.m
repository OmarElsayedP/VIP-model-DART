clc; clear; close all;

%% LIP
% xlip = importdata('LIP\x.txt');
% xlip_m = importdata('LIP\x_m.txt');
% ylip = importdata('LIP\y.txt');
% ylip_m = importdata('LIP\y_m.txt');

% xzlip = importdata('LIP\xz.txt');
% xzlip_m = importdata('LIP\xz_m_cop.txt');
% yzlip = importdata('LIP\yz.txt');
% yzlip_m = importdata('LIP\yz_m_cop.txt');

% xdlip = importdata('LIP\xd.txt');
% xdlip_m = importdata('LIP\xd_m.txt');
% ydlip = importdata('LIP\yd.txt');
% ydlip_m = importdata('LIP\yd_m.txt');


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


%% no title plots

x = importdata('x.txt');
x_m = importdata('x_m.txt');
y = importdata('y.txt');
y_m = importdata('y_m.txt');

xsupp = importdata('x_supp.txt');
xsupp_m = importdata('x_m_supp.txt');
xsw_m = importdata('x_m_sw.txt');
ysupp = importdata('y_supp.txt');
ysupp_m = importdata('y_m_supp.txt');
xsw = importdata('x_sw.txt');
ysw = importdata('y_sw.txt');
ysw_m = importdata('x_m_sw.txt');


xwithnoise = importdata('xwithnoise.txt');
ywithnoise = importdata('ywithnoise.txt');
xzwithnoise = importdata('xzwithnoise.txt');
yzwithnoise = importdata('yzwithnoise.txt');
virt_torqx = importdata('virt_torqx.txt');
virt_torqy = importdata('virt_torqy.txt');

xd = importdata('xd.txt');
xdcam = importdata('xdcam.txt');
xd_m = importdata('xd_m.txt');
yd = importdata('yd.txt');
ydcam = importdata('ydcam.txt');
yd_m = importdata('yd_m.txt');

xzd = importdata('xzd.txt');
yzd = importdata('yzd.txt');


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

actualOrientation_x = importdata('actualOrientation_x.txt');
actualOrientation_y = importdata('actualOrientation_y.txt');
actualOrientation_z = importdata('actualOrientation_z.txt');
actualAngVel_x = importdata('actualAngVel_x.txt');
actualAngVel_y = importdata('actualAngVel_y.txt');
actualAngVel_z = importdata('actualAngVel_y.txt');


error_x = importdata('error_x.txt');
error_y = importdata('error_y.txt');

errorfoot_x = importdata('errorfoot_x.txt');
errorfoot_y = importdata('errorfoot_y.txt');


desiredTorsoPosition_foot_x = importdata('desiredTorsoPosition_foot_x.txt');
desiredTorsoPosition_foot_y = importdata('desiredTorsoPosition_foot_y.txt');
desiredTorsoPosition_foot_z = importdata('desiredTorsoPosition_foot_z.txt');

actualOrientation_foot_x = importdata('actualOrientation_foot_x.txt');
actualOrientation_foot_y = importdata('actualOrientation_foot_y.txt');
actualOrientation_foot_z = importdata('actualOrientation_foot_z.txt');

torquex_desired = importdata('torquex_desired.txt');
torquey_desired = importdata('torquey_desired.txt');
torquez_desired = importdata('torquez_desired.txt');


%% Import matlab plots

% xtot_store = importdata('xtot_store.txt');
% ytot_store = importdata('ytot_store.txt');
% x_store = importdata('x_store.txt');
% y_store = importdata('y_store.txt');
% xz_store = importdata('xz_store.txt');
% yz_store = importdata('yz_store.txt');
% xztot_store = importdata('xztot_store.txt');
% yztot_store = importdata('yztot_store.txt');
% thetax = importdata('thetax.txt');
% thetay = importdata('thetay.txt');
% thetaz = importdata('thetaz.txt');
% modx_store = importdata('modx_store.txt');
% modxz_store = importdata('modxz_store.txt');
% mody_store = importdata('mody_store.txt');
% modyz_store = importdata('modyz_store.txt');
% virt_torqx_matlab = importdata('virt_torqx_matlab.txt');
% virt_torqy_matlab = importdata('virt_torqy_matlab.txt');

% xzd_store = importdata('xzd_store.txt');
% yzd_store = importdata('yzd_store.txt');

% Plot the natural 'noise'
% figure;
% subplot(1,2,1);
% plot(virt_torqx_matlab)
% % plot(xwithnoise-x);
% hold on;
% plot(x_m-x);
% legend('virtual','actual','Location','northeast');
% xlabel('Time [samples]');
% ylabel('Position [m]');
% title('Noise x-axis');
% grid on;
% subplot(1,2,2);
% % plot(ywithnoise-y);
% plot(virt_torqy_matlab)
% hold on;
% plot(y_m-y);
% legend('virtual','actual','Location','northeast');
% xlabel('Time [samples]');
% ylabel('Position [m]');
% title('Noise y-axis');
% grid on;

% return;

% Plot the centre of mass trajectories
% figure
% % suptitle('Difference between model and real COM trajectories X-axis');
% subplot(1,2,1);
% title('Dart');
% plot(xwithnoise-x);
% grid on;
% subplot(1,2,2);
% title('Matlab');
% plot(x_store-modx_store);
% xlabel('Time');
% ylabel('Velocities [m/s]');
% grid on;

% figure
% % sgtitle('Difference between model and real COM trajectories Y-axis');
% subplot(1,2,1);
% title('Dart');
% plot(ywithnoise-y);
% grid on;
% subplot(1,2,2)
% title('Matlab');
% plot(y_store-mody_store);
% xlabel('Time');
% ylabel('Velocities [m/s]');
% grid on;

% figure
% % sgtitle('ZMP Velocities');
% subplot(1,2,1);
% title('X-axis');
% plot(xzd_store);
% hold on;
% plot(xzd);
% legend('matlab','Dart','Location','northeast');
% xlabel('Time');
% ylabel('Velocities [m/s]');
% grid on;
% subplot(1,2,2)
% title('Y-axis');
% plot(yzd_store);
% hold on;
% plot(yzd);
% xlabel('Time');
% ylabel('Velocities [m/s]');
% legend('matlab','Dart','Location','northeast');
% grid on;
% return;

% figure;
% plot(xzlip,yzlip);
% hold on
% plot(xzlip_m, yzlip_m);
% legend('desired','actual','Location','northeast')
% xlabel('Xz [M]');
% ylabel('Yz [M]');
% title('LIP model');
% grid on;
if(false)
	% disp(size(x));
	% disp(size(y));
	% disp(size(x_m));
	% disp(size(y_m));

% figure;
% plot(x,y);
% hold on;
% plot(xz,yz);
% hold on;
% plot(xzwithnoise+xzcam,yzwithnoise+yzcam);
% legend('COM','ZMP','CMP','Location','northeast')
% xlabel('X [M]');
% ylabel('Y [M]');
% title('VIP model(partial feedback position only)');
% grid on;

% % return;
% figure;
% plot(xwithnoise+xcam,ywithnoise+ycam);
% hold on;
% plot(xzwithnoise+xzcam,yzwithnoise+yzcam);
% hold on;
% plot(xtot_store,ytot_store);
% hold on;
% plot(xztot_store,yztot_store);
% legend('COM','CMP','Matlab COM','Matlab CMP','Location','northeast')
% xlabel('X [M]');
% ylabel('Y [M]');
% title('CoM, ZMP and CMP trajectories');
% grid on;
% figure;
% plot(xwihtnoise,ywithnoise);
% hold on;
% plot(xzwihtnoise,yzwithnoise);
% % plot(x,y);
% % hold on;
% % plot(xz,yz);
% hold on;
% plot(xtot_store,ytot_store);
% hold on;
% plot(xz_store,yz_store);
% legend('COM','ZMP','Matlab COM','Matlab ZMP','Location','northeast')
% xlabel('X [M]');
% ylabel('Y [M]');
% title('CoM & ZMP trajectories');
% grid on;

figure;
subplot(1,2,1);
plot(xzcam);
hold on;
plot(xztot_store-xz_store);
legend('Dart','Matlab','Location','northeast')
xlabel('X [M]');
ylabel('Y [M]');
title('Xzcam');
grid on;
subplot(1,2,2);
plot(yzcam);
hold on;
plot(yztot_store-yz_store);
legend('Dart','Matlab','Location','northeast')
xlabel('X [M]');
ylabel('Y [M]');
title('Yzcam');
grid on;

figure;
subplot(1,3,1);
plot(desiredTorsoPosition_x);
hold on;
plot(thetax);
xlabel('time (samples)');
ylabel('angle (rad)');
legend('Dart','Matlab','Location','northeast')
title('Rotation around x-axis');
grid on
subplot(1,3,2);
plot(desiredTorsoPosition_y);
hold on;
plot(thetay);
xlabel('time (samples)');
ylabel('angle (rad)');
legend('Dart','Matlab','Location','northeast')
title('Rotation around y-axis');
grid on
subplot(1,3,3);
plot(desiredTorsoPosition_z);
hold on;
plot(thetaz);
xlabel('time (samples)');
ylabel('angle (rad)');
legend('Dart','Matlab','Location','northeast')
title('Rotation around z-axis');
grid on

% figure;
% plot(x,y);
% hold on
% plot(xz, yz)
% legend('COM','ZMP','Location','northeast')
% xlabel('X [M]');
% ylabel('Y [M]');
% grid on;

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
if(false) % true = isDoublePendulum, false = LIP
% figure;
% plot(actualOrientation_x);
% xlabel('Time');
% ylabel('Torso Angular velocity y [rad/s]');
% title('Torso ang. velocity X-axis');
% grid on;

% 	figure;
% plot(actualOrientation_y);
% xlabel('Time');
% ylabel('Torso Angular velocity y [rad/s]');
% title('Torso ang. velocity Y-axis');
% grid on;

% return;

figure;
plot(x+xcam,y+ycam);
hold on
plot(x_m, y_m);
legend('desired','actual','Location','northeast')
xlabel('X [M]');
ylabel('Y [M]');
title('Total CoM position');
grid on;

figure;
plot(xz+xzcam,yz+yzcam);
hold on
plot(xz_m, yz_m);
legend('desired','actual','Location','northeast')
xlabel('Xz [M]');
ylabel('Yz [M]');
title('Total ZMP position');
grid on;



figure;
plot(xd+xdcam);
hold on
plot(xd_m);
legend('desired','actual','Location','northeast')
xlabel('Time');
ylabel('Velocities [m/s]');
title('Total X-CoM velocity');
grid on;

figure;
plot(yd+ydcam);
hold on
plot(yd_m);
legend('desired','actual','Location','northeast')
xlabel('Time');
ylabel('Velocities [m/s]');
title('Total Y-CoM velocity');
grid on;


figure;
plot(desiredTorsoPosition_x);
hold on
plot(actualOrientation_x);
legend('desired','actual','Location','northeast')
xlabel('Time');
ylabel('Torso Angle x [rad]');
title('Torso orientation X-axis');
grid on;

figure;
plot(desiredTorsoPosition_y);
hold on
plot(actualOrientation_y);
legend('desired','actual','Location','northeast')
xlabel('Time');
ylabel('Torso Angle y [rad]');
title('Torso orientation Y-axis');
grid on;

figure;
plot(desiredTorsoPosition_z);
hold on
plot(actualOrientation_z);
legend('desired','actual','Location','northeast')
xlabel('Time');
ylabel('Torso Angle z [rad]');
title('Torso orientation Z-axis');
grid on;



figure;
plot(desiredTorsoVelocity_x);
hold on
plot(actualAngVel_x);
xlabel('Time');
ylabel('Torso Angular velocity x [rad/s]');
title('Torso ang. velocity X-axis');
legend('desired','actual','Location','northeast')
grid on;

figure;
plot(desiredTorsoVelocity_y);
hold on
plot(actualAngVel_y);
xlabel('Time');
ylabel('Torso Angular velocity y [rad/s]');
title('Torso ang. velocity Y-axis');
legend('desired','actual','Location','northeast')
grid on;

figure;
plot(desiredTorsoVelocity_z);
hold on
plot(actualAngVel_z);
xlabel('Time');
ylabel('Torso Angular velocity z [rad/s]');
title('Torso ang. velocity Z-axis');
legend('desired','actual','Location','northeast')
grid on;




figure;
plot(xcam);
hold on;
plot(ycam)
xlabel('Time');
ylabel('CAM CoM[M]');
title('CAM CoM');
legend('Xcam','Ycam','Location','northeast')
grid on;

figure;
plot(xzcam);
hold on;
plot(yzcam)
xlabel('Time');
ylabel('CAM ZMP [M]');
title('CAM ZMP (proportional to Torques on the Torso)');
legend('Xzcam','Yzcam','Location','northeast')
grid on;


figure;
subplot(1,2,1)
plot(xcam+x);
xlabel('Time');
ylabel('CoM Position [M]');
title('Total CoM X-axis');
grid on;
subplot(1,2,2)
plot(ycam+y);
xlabel('Time');
ylabel('CoM Position [M]');
title('Total CoM Y-axis');
grid on;


figure;
plot(x,y);
hold on
plot(xz, yz);
hold on
plot(xz+xzcam, yz+yzcam);
legend('CoM','ZMP','CMP','Location','northeast')
xlabel('X [M]');
ylabel('Y [M]');
title('ZMP, CoM and CMP trajectories');
grid on;

figure;
plot(xzcam);
hold on;
plot(yzcam)
xlabel('Time');
ylabel('CAM ZMP [M]');
title('CAM ZMP (proportional to Torques on the Torso)');
legend('Xzcam','Yzcam','Location','northeast')
grid on;



else
	if(false)

% figure;
% subplot(1,2,1)
% plot(xcam);
% xlabel('Time');
% ylabel('CoM Position [M]');
% title('CoM Angular Momentum Pendulum X-axis');
% grid on;
% subplot(1,2,2)
% plot(ycam);
% xlabel('Time');
% ylabel('CoM Position [M]');
% title('CoM Angular Momentum Pendulum Y-axis');
% grid on;


% figure;
% subplot(1,2,1)
% plot(xzcam);
% xlabel('Time');
% ylabel('ZMP Position [M]');
% title('ZMP Angular Momentum Pendulum X-axis');
% grid on;
% subplot(1,2,2)
% plot(yzcam);
% xlabel('Time');
% ylabel('ZMP Position [M]');
% title('ZMP Angular Momentum Pendulum Y-axis');
% grid on;


% figure;
% plot(xzcam, yzcam);
% xlabel('X [M]');
% ylabel('Y [M]');
% title('CoM Angular Momentum Pendulum');
% grid on;



	else
% figure;
% subplot(2,1,1);
% plot(error_x);
% hold on
% plot(errorfoot_x);
% legend('World frame','Foot frame','Location','northeast')
% xlabel('Time [samples]');
% ylabel('X [M]');
% title('CoM error X-axis');
% grid on;
% subplot(2,1,2);
% plot(error_y);
% hold on
% plot(errorfoot_y);
% legend('World frame','Foot frame','Location','northeast')
% xlabel('Time [samples]');
% ylabel('Y [M]');
% title('CoM error Y-axis');
% grid on;
% end

% figure;
% plot(x,y);
% hold on
% plot(x_m, y_m);
% legend('desired','actual','Location','northeast')
% xlabel('X [M]');
% ylabel('Y [M]');
% grid on;

% figure;
% scatter(xsupp,ysupp);
% hold on
% scatter(xsw, ysw);
% % legend('desired','actual','Location','northeast')
% xlabel('X [M]');
% ylabel('Y [M]');
% grid on;


end


% figure;
% plot(xd);
% hold on
% plot(xd_m);
% legend('desired','actual','Location','northeast')
% xlabel('Time');
% ylabel('Velocities [m/s]');
% title('VIP model(partial feedback position only)');
% grid on;


% figure;
% plot(x,y);
% hold on
% plot(x_m, y_m);
% legend('desired','measured','Location','northeast')
% xlabel('X [M]');
% ylabel('Y [M]');
% title('CoM');
% grid on;

figure;
subplot(2,1,1);
plot(error_x);
hold on
plot(errorfoot_x);
legend('World frame','Foot frame','Location','northeast')
xlabel('Time [samples]');
ylabel('X [M]');
title('CoM error X-axis');
grid on;
subplot(2,1,2);
plot(error_y);
hold on
plot(errorfoot_y);
legend('World frame','Foot frame','Location','northeast')
xlabel('Time [samples]');
ylabel('Y [M]');
title('CoM error Y-axis');
grid on;

figure;
plot(desiredTorsoPosition_x);
hold on
plot(actualOrientation_x);
legend('desired','actual','Location','northeast')
xlabel('Time [samples]');
ylabel('Torso Angle x [rad]');
title('Torso orientation X-axis');
grid on;

figure;
plot(desiredTorsoPosition_y);
hold on
plot(actualOrientation_y);
legend('desired','actual','Location','northeast')
xlabel('Time [samples]');
ylabel('Torso Angle y [rad]');
title('Torso orientation Y-axis');
grid on;

figure;
plot(desiredTorsoPosition_foot_x);
hold on
plot(actualOrientation_foot_x);
legend('desired','actual','Location','northeast')
xlabel('Time [samples]');
ylabel('Torso Angle x [rad]');
title('Torso orientation X-axis in footframe');
grid on;

figure;
plot(desiredTorsoPosition_foot_y);
hold on
plot(actualOrientation_foot_y);
legend('desired','actual','Location','northeast')
xlabel('Time [samples]');
ylabel('Torso Angle y [rad]');
title('Torso orientation Y-axis in footframe');
grid on;


% figure;
% plot(desiredTorsoPosition_z);
% hold on
% plot(actualOrientation_z);
% legend('desired','actual','Location','northeast')
% xlabel('Time');
% ylabel('Torso Angle z [rad]');
% title('Torso orientation Z-axis');
% grid on;


figure;
plot(xzcam);
hold on;
plot(yzcam)
xlabel('Time [samples]');
ylabel('CAM ZMP [M]');
title('CAM ZMP (proportional to Torques on the Torso)');
legend('Xzcam','Yzcam','Location','northeast')
grid on;



figure;
plot(torquex_desired);
hold on;
plot(torquey_desired);
hold on;
plot(torquez_desired);
xlabel('Time [samples]');
ylabel('Torques [N m]');
title('Torques');
legend('X','Y','Z','Location','northeast')
grid on;


end
end