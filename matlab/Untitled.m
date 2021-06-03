clear;
clc;
close;
hold on
a=1; % horizontal radius
b=4; % vertical radius
x0=0; % x0,y0 ellipse centre coordinates
z0=10;
plot(x0,z0,'k.','Markersize',15);
t=-pi:0.01:pi;
x=x0+a*cos(t);
z=z0+b*sin(t);
plot(x,z,'b');
axis([-10, 10, -5, 20]);
xpend(1) = 0;
zpend(1) = 0;
xpend(2) = x0;
zpend(2) = z0;
plot(xpend,zpend,'black','Markersize',2);
grid on



    A_zmpconstrwithcam = [Pzmp, -mapping(:,2:end), zeros(C, C+F), ...
        Pzmp, zeros(C,C); zeros(C, C+F), Pzmp, -mapping(:,2:end), ...
        zeros(C,C), Pzmp; -Pzmp, mapping(:,2:end), zeros(C, C+F), ...
        -Pzmp, zeros(C, C); zeros(C, C+F), -Pzmp, mapping(:,2:end), ...
        zeros(C, C), -Pzmp];
    
    b_zmpconstrwithcam = [pzmp * (- xz + wx/2 + xz_cam_max-xz_cam) + mapping(:,1) * current_xfs; ...
        pzmp * (- yz + wy/2 + yz_cam_max - yz_cam) + mapping(:,1) * current_yfs; ...
        - pzmp * (- xz - wx/2 - xz_cam_max - xz_cam) - mapping(:,1) * current_xfs; ...
        - pzmp * (- yz - wy/2 - yz_cam_max - yz_cam) - mapping(:,1) * current_yfs];

