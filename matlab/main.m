clc; clear; close all;
load 'plots\fixed steps\cdd\1%1%1\LIP'
xcLIP = xtot_store;
ycLIP = ytot_store;
load 'plots\fixed steps\cdd\1%1%1\VIP'
xcVIP = xtot_store;
ycVIP = ytot_store;
%% Plot the centre of mass trajectories
figure;
subplot(2,1,1)
plot(xcLIP,'r.');
hold on
plot(xcVIP,'b-');
% hold off
legend('xcLIP','xcVIP','Location','northeast')
xlabel('time [samples]');
ylabel('x_c [m]');
title('Centre of mass trajactory in x');
grid on
%ZMP velocity in y
subplot(2,1,2)
plot(ycLIP,'r.');
hold on
plot(ycVIP,'b-');
% hold off
legend('ycLIP','ycVIP','Location','northeast')
xlabel('time [samples]');
ylabel('y_c [m]');
title('Centre of mass trajactory in y');
grid on