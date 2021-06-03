clear
close all
clc
load 'xzcam%xz (0%10) stepsx2.mat'
xzdLIP = xzd_store + xzdcam_store;
yzdLIP = yzd_store + yzdcam_store;
load 'xzcam%xz (1%10) stepsx2 angle constraint'
xzdVIP = xzd_store + xzdcam_store;
yzdVIP = yzd_store + yzdcam_store;
%% Plot ZMP velocities
figure;
subplot(2,1,1)
plot(xzdLIP,'r');
hold on
plot(xzdVIP,'b-');
hold on
legend('$\dot{x}_{z,lip}$','$\dot{x}_{z,vip}$','Location','northeast','Interpreter','latex')
xlabel('time [samples]');
ylabel('Xzmp Velocity [m/s]');
title('ZMP velocity in x');
grid on
%ZMP velocity in y
subplot(2,1,2)
plot(yzdLIP,'r');
hold on
plot(yzdVIP,'b-');
hold on
legend('$\dot{y}_{z,lip}$','$\dot{y}_{z,vip}$','Location','northeast','Interpreter','latex')
xlabel('time [samples]');
ylabel('Yzmp Velocity [m/s]');
title('ZMP velocity in y');
grid on