clc; clear; close all;
load 'push\(1, 10, 0, 1e15)'
tx = tx_store;
ty = ty_store;
load 'push\(1, 10, 1e-2, 1e15)'
tx_cdd = tx_store;
ty_cdd = ty_store;
%% Plot the centre of mass trajectories
figure;
subplot(1,2,1)
plot(tx);
hold on
plot(tx_cdd);
legend('Normal','with $\ddot{c}$ term','Location','northeast','Interpreter','latex')
xlabel('time [samples]');
ylabel('Torque [Nm]');
title('Torque in x');
grid on

subplot(1,2,2)
plot(ty);
hold on
plot(ty_cdd);
legend('Normal','with $\ddot{c}$ term','Location','northeast','Interpreter','latex')
xlabel('time [samples]');
ylabel('Torque [Nm]');
title('Torque in y');
grid on