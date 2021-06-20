clc; clear; close all;
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

error_torsoAngle_foot_x = importdata('error_torsoAngle_foot_x.txt');
error_torsoAngle_foot_y = importdata('error_torsoAngle_foot_y.txt');
error_torsoAngle_foot_z = importdata('error_torsoAngle_foot_z.txt');
error_torsoPos_foot_x = importdata('error_torsoPos_foot_x.txt');
error_torsoPos_foot_y = importdata('error_torsoPos_foot_y.txt');
error_torsoPos_foot_z = importdata('error_torsoPos_foot_z.txt');
error_footAngle_foot_x = importdata('error_footAngle_foot_x.txt');
error_footAngle_foot_y = importdata('error_footAngle_foot_y.txt');
error_footAngle_foot_z = importdata('error_footAngle_foot_z.txt');
error_footPos_foot_x = importdata('error_footPos_foot_x.txt');
error_footPos_foot_y = importdata('error_footPos_foot_y.txt');
error_footPos_foot_z = importdata('error_footPos_foot_z.txt');

figure;
subplot(2,2,1);
plot(error_torsoAngle_foot_x);
hold on
plot(error_torsoAngle_foot_y);
hold on
plot(error_torsoAngle_foot_z);
legend('X','Y','Z','Location','southeast')
xlabel('Time [samples]');
ylabel('Orientation error [rad]');
title('error torsoAngle foot frame');
grid on;
subplot(2,2,2);
plot(error_torsoPos_foot_x);
hold on
plot(error_torsoPos_foot_y);
hold on
plot(error_torsoPos_foot_z);
legend('X','Y','Z','Location','southeast')
xlabel('Time [samples]');
ylabel('Position error [m]');
title('error torsoPos foot frame');
grid on;


subplot(2,2,3);
plot(error_footAngle_foot_x);
hold on
plot(error_footAngle_foot_y);
hold on
plot(error_footAngle_foot_z);
legend('X','Y','Z','Location','southeast')
xlabel('Time [samples]');
ylabel('Orientation error [rad]');
title('error footAngle foot frame');
grid on;
subplot(2,2,4);
plot(error_footPos_foot_x,'Linewidth', 2);
hold on
plot(error_footPos_foot_y,'--');
hold on
plot(error_footPos_foot_z);
legend('X','Y','Z','Location','southeast')
xlabel('Time [samples]');
ylabel('Position error [m]');
title('error footPos foot frame');
grid on;
