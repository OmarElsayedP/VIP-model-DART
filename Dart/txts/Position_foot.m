%%
clear
close all
clc
load 'steps\push\(1, 10, 0, 1e15)'
predicted_xfs_1e7 = predicted_xfs;
xfs_store_1e7 = xfs_store;
predicted_yfs_1e7 = predicted_yfs;
yfs_store_1e7 = yfs_store;
load 'steps\push\(1, 10, 1e-2, 1e15)'
predicted_xfs_1e15 = predicted_xfs;
xfs_store_1e15 = xfs_store;
predicted_yfs_1e15 = predicted_yfs;
yfs_store_1e15 = yfs_store;
%%
clf
hold on
rectangle_initial = [initial_wx/2,initial_wx/2,-initial_wx/2,-initial_wx/2,initial_wx/2; ...
    initial_wy/2,-initial_wy/2,-initial_wy/2,initial_wy/2,initial_wy/2];
rectangle = [wx/2,wx/2,-wx/2,-wx/2,wx/2; wy/2,-wy/2,-wy/2,wy/2,wy/2];
% rectangle_big = kron(ones(10,1),rectangle');
% fs_plan_big = kron(fs_plan(3:end,:),ones(5,1));
% Plot desired trajectory
plot(rectangle_initial(1,:), rectangle_initial(2,:),'g','Linewidth',2)
hold on
for i = 2:10
    plot(rectangle(1,:) + fs_plan(i,1), rectangle(2,:) + fs_plan(i,2),...
        'g','Linewidth',2,'HandleVisibility','off');
    hold on
end
%Plot 1st set of actual footsteps
plot(rectangle_initial(1,:), rectangle_initial(2,:),'r','Linewidth',2)
for i = 2:fsCounter
    plot(rectangle(1,:) + xfs_store_1e7(i), rectangle(2,:) + yfs_store_1e7(i),...
        'r','Linewidth',2,'HandleVisibility','off');
    hold on
end
for i = 1:F
    plot(rectangle(1,:) + predicted_xfs_1e7(i), ...
        rectangle(2,:) + predicted_yfs_1e7(i),'r','Linewidth',2,'HandleVisibility','off')
end

%Plot 2nd set of actual footsteps
plot(rectangle_initial(1,:), rectangle_initial(2,:),'b','Linewidth',2)
for i = 2:fsCounter
    plot(rectangle(1,:) + xfs_store_1e15(i), rectangle(2,:) + yfs_store_1e15(i),...
        'b','Linewidth',2,'HandleVisibility','off');
    hold on
end
for i = 1:F
    plot(rectangle(1,:) + predicted_xfs_1e15(i), ...
        rectangle(2,:) + predicted_yfs_1e15(i),'b','Linewidth',2,'HandleVisibility','off')
end
axis equal
axis([-0.2,1.2,-0.15,0.15])
grid on
xlabel('x (m)');
ylabel('y (m)');
legend('desired','Normal','with \ddot(c) term','Location','northeast')
drawnow

%% Error in footstep norm
%1e7
for i = 1:length(xfs_store)
    xferror = xfs_store_1e7(i)-fs_plan(i,1);
    yferror = yfs_store_1e7(i)-fs_plan(i,2);
end
norm(xferror) + norm(yferror)
%1e15
for i = 1:length(xfs_store)
    xferror = xfs_store_1e15(i)-fs_plan(i,1);
    yferror = yfs_store_1e15(i)-fs_plan(i,2);
end
norm(xferror) + norm(yferror)