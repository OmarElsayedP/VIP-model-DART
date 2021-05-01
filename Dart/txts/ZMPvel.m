clear
close all
% clc
load 'C:\Users\hp\Desktop\masters\thesis\Reports\Report 6\2nd\Cdd = 0\VIPa'
xzdLIP = xzd_store + xzdcam_store;
yzdLIP = yzd_store + yzdcam_store;
xZMPL = xz_store;
yZMPL = yz_store;
load 'C:\Users\hp\Desktop\masters\thesis\Reports\Report 6\2nd\Cdd = 0\([1e-2,1e-1],[1e-1,1]a,0)\VIPa'
xzdVIP = xzd_store + xzdcam_store;
yzdVIP = yzd_store + yzdcam_store;
xZMPV = xz_store;
yZMPV = yz_store;
%% Plot ZMP velocities
figure;
subplot(2,1,1)
plot(xzdLIP,'r');
hold on
plot(xzdVIP,'b-');
hold on
legend('$\dot{x}_{z,lip}$','$\dot{x}_{z,vip}$','Location','northeast','Interpreter','latex')
% legend('Normal','with $\ddot{c}$ term','Location','northeast','Interpreter','latex')
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
% legend('Normal','with $\ddot{c}$ term','Location','northeast','Interpreter','latex')
xlabel('time [samples]');
ylabel('Yzmp Velocity [m/s]');
title('ZMP velocity in y');
grid on

diff = norm(xzdLIP)+norm(yzdLIP)-norm(xzdVIP)-norm(yzdVIP);
if diff<0
    disp("LIP is better")
else
    disp("VIP is better")
end
%% Plot ZMP velocities
figure;
plot(xZMPL,yZMPL,'r');
hold on
plot(xZMPV,yZMPV,'b');
hold on
legend('$LIP$','$VIP$','Location','northeast','Interpreter','latex')
xlabel('time [samples]');
ylabel('ZMP [m]');
title('ZMP position');
grid on