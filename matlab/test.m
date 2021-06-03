clear
close all
load 'C:\Users\hp\Desktop\masters\thesis\Reports\Report 6\2nd\xnoise = -pi\LIP'
clear virt_torq
%Imitating torque from cam
% for i = 1:j
%     virt_torq(1,i+104) = 0.00030333333*i;
%     virt_torq(2,i+55) = -0.0008*sin(i*(1/104)*2*pi)+i*0.000001;
% end
%Imitating Angular momentum of humanoid
for i = 1:j
%     if(mod(i,50))
%         flag = 0; 
%     end
%     if(i<50)
%         virt_torq(1,i) = -0.005*sin(i*0.01*pi);
%     else
%         if(i<50+ssSamples)
%             virt_torq(1,i) = -0.005+0.008*sin((i-50)*(1/115)*2*pi);
%         end
%     end
%     if(flag)
%         virt_torq(2,i) = -0.001*sin(i*0.01*2*pi-4*pi/5);
%         i = i+ssSamples;
%         flag = 0;
%         continue;
%         %             else
%         %             virt_torq(1,i) = 0.003*cos((i-100)*0.01*1.83*2*pi);
%     end
    virt_torq(1,i+50) = 0.0005*sin((i+50)*0.01*4*pi-pi);
    
    virt_torq(2,i) = -0.0005*sin(i*0.01*2*pi);
end
virt_torq = virt_torq(:,1:j);
subplot(1,2,1);
plot(xz_store)
hold on
plot(xz_store+virt_torq(1,:))
legend('model','virtual','Location','NorthEast')
xlabel('time [samples]');
ylabel('ZMP [m]');
title('ZMP x-axis');
grid on
subplot(1,2,2);
plot(yz_store);
hold on
plot(yz_store + virt_torq(2,:))
legend('model','virtual'    )
xlabel('time [samples]');
ylabel('ZMP [m]');
title('ZMP y-axis');
grid on

%%
figure
subplot(1,2,1);
plot(virt_torq(1,:));
xlabel('time [samples]');
ylabel('CoM [m]');
title('CoM x-axis');
grid on
subplot(1,2,2);
plot(virt_torq(2,:));
xlabel('time [samples]');
ylabel('CoM [m]');
title('CoM y-axis');
grid on
%%
% figure
% subplot(1,2,1);
% plot(x_store);
% hold on
% plot(xz_store);
% xlabel('time [samples]');
% ylabel('CoM [m]');
% title(' x-axis');
% grid on
% legend('CoM','Zmp')
% subplot(1,2,2);
% plot(y_store);
% hold on
% plot(yz_store);
% xlabel('time [samples]');
% ylabel('CoM [m]');
% title('y-axis');
% grid on
% legend('CoM','Zmp')

%%
figure
subplot(1,2,1);
plot(virt_torq(1,50:150));
hold on
plot(yz_store(50:150)*0.01)
legend('Noise','ZMP')
xlabel('time [samples]');
ylabel('CoM [m]');
title('CoM x-axis');
grid on
subplot(1,2,2);
plot(virt_torq(2,50:150));
% hold on
% plot(yz_store(100:200))
% legend('Noise','ZMP')
xlabel('time [samples]');
ylabel('CoM [m]');
title('CoM y-axis');
grid on