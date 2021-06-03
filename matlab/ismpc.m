clear; clc; close;
% footstep plan and footstep timings
NF = 10;

fs_plan = [0.0, 0.0; ... %dummy footstep
    0.0, -0.075; ...
    0.05, 0.075; ...
    0.1, -0.075; ...
    0.15, 0.075; ...
    0.2, -0.075; ...
    0.25, 0.075; ...
    0.3, -0.075; ...
    0.35, 0.075; ...
    0.4, -0.075; ...
    0.45, 0.075; ...
    0.5, -0.075];


fs_timing = [0; 50; 100; 150; 200; 250; 300; 350; 400; 450];

simDuration = 1000;

% general MPC parameters
C = 100; %Control Horizon
P = 200; %Predition Horizon
F = 3; %
mpcTimeStep = 0.01;
z0 = 0.75; %constant height = 0.75
eta = sqrt(9.81/z0);
dsSamples = 20; %Double support samples 20 per 50
% ZMP constraint parameters
wx = 0.08;  %dzx in paper
wy = 0.08; %dzy in paper
initial_wx = 0.08;
initial_wy = 0.08/2 + 0.2;


%ZMP cam %according to paper
tx_cam = 100; %Max and min torque in x
ty_cam = 100; %max and min torque in y
m = 6.67348; %mass of torso
%Moment of inertia of torso
Jxx = 0.344345670122806; Jyy = 0.338324801980916; Jzz = 0.048160214086886;
Jyz = 0.003172978374495; Jxz = 0.045048560212699; Jxy = -5.676994253777424e-04;
J = [Jxx, Jxy, Jxz; Jxy, Jyy, Jyz; Jxz, Jyz, Jzz];
xz_cam_max = ty_cam/(m*9.81);
yz_cam_max = tx_cam/(m*9.81);

% kinematic constraint parameters
ell = 0.2; %l (distance between 2 feet centers)
wkx = 0.5; %dax
wky = 0.08; %day

% state initialization
x = 0.0;
xd = 0.0;
xz = 0.0;
y = 0.0;
yd = 0.0;
yz = 0.0;
current_xfs = fs_plan(1,1);
current_yfs = fs_plan(1,2);

fsCounter = 1;

% state inizialization CAM
x_cam = 0.0;
xd_cam = 0.0;
xz_cam = 0.0;
y_cam = 0.0;
yd_cam = 0.0;
yz_cam = 0.0;

% state inizialazation total
xtot = x + x_cam;
xdtot = xd + xd_cam;
xztot = xz + xz_cam;
ytot = y + y_cam;
ydtot = yd + yd_cam;
yztot = yz + yz_cam;


% state update matrices
ch = cosh(eta*mpcTimeStep);
sh = sinh(eta*mpcTimeStep);
A_upd = [ch, sh/eta, 1-ch; eta* sh, ch, -eta*sh; 0, 0, 1];
B_upd = [mpcTimeStep-sh/eta; 1-ch; mpcTimeStep];

% state update matrices CAM
ch_cam = cosh(eta*mpcTimeStep);
sh_cam = sinh(eta*mpcTimeStep);
A_upd_cam = [ch_cam, sh_cam/eta, 1-ch_cam; eta*sh_cam, ch_cam, -eta*sh_cam; 0, 0, 1];
B_upd_cam = [mpcTimeStep-sh_cam/eta; 1-ch_cam; mpcTimeStep];

% %%% Habal
% A_upd_cam = [1, mpcTimeStep, 0; eta^2*mpcTimeStep, 0, -eta^2*mpcTimeStep; 0, 0, 1];
% B_upd_cam = [0; 0; mpcTimeStep];


% stored trajectories
x_store = x;
xz_store = xz;
y_store = y;
yz_store = yz;
xfs_store = current_xfs;
yfs_store = current_yfs;

% stored trajectories cam
x_store_cam = x_cam;
xz_store_cam = xz_cam;
y_store_cam = y_cam;
yz_store_cam = yz_cam;
tx_store_cam = 0;
ty_store_cam = 0;
tz_store_cam = 0;

% stored trajectories total
xtot_store = xtot;
xztot_store = xztot;
ytot_store = ytot;
yztot_store = yztot;

%Angular velocity and position
thetad = zeros(3,simDuration);
theta = zeros(3,simDuration);

% ZMP constraint constant matrices
Pzmp = tril(ones(C,C)) * mpcTimeStep;
pzmp = ones(C,1);

% compute the centerline for the entire footstep plan
cl_x = fs_plan(1,1) * ones(fs_timing(2)-fs_timing(1), 1);
cl_y = fs_plan(1,2) * ones(fs_timing(2)-fs_timing(1), 1);
cl_fscounter = 1;
for i = 2:NF-1
    cl_x = [cl_x; fs_plan(i,1) * ones(fs_timing(2)-dsSamples-fs_timing(1), 1); ...
        linspace(fs_plan(i,1), fs_plan(i+1,1), dsSamples)'];
    cl_y = [cl_y; fs_plan(i,2) * ones(fs_timing(2)-dsSamples-fs_timing(1), 1); ...
        linspace(fs_plan(i,2), fs_plan(i+1,2), dsSamples)'];
end
% boundedness functions
constant = @(xz,ti,tf) xz * (exp(-eta*ti) - exp(-eta*tf));
linear = @(xzi,xzf,ti,tf) (xzi + (xzf-xzi) / (eta * (tf-ti))) * (exp(-eta*ti) - exp(-eta*tf));
truncated = @(xz,ti) xz * exp(-eta*ti);
for j = 1:simDuration
    % Break (anticipative) when tail is out of bounds
%     if(j+P > length(cl_x))
%         break;
%     end
    % compute anticipative tail
    anticipative_x = exp(-eta*mpcTimeStep*(C+1:P)) * (1-exp(-eta*mpcTimeStep)) *  cl_x(j+C+1:j+P) + ...
        exp(-eta*mpcTimeStep*P) * cl_x(P);
    anticipative_y = exp(-eta*mpcTimeStep*(C+1:P)) * (1-exp(-eta*mpcTimeStep)) *  cl_y(j+C+1:j+P) + ...
        exp(-eta*mpcTimeStep*P) * cl_y(P);
    
    % ZMP constraint
    
    % compute mapping timestep -> predicted footstep number
    mapping = zeros(C,F+1);
    predictedFootstep = 0;
    for i = 1:C
        if j + i >= fs_timing(fsCounter + predictedFootstep + 1)
            predictedFootstep = predictedFootstep + 1;
        end
        samplesRemaining = fs_timing(fsCounter + predictedFootstep+1)-(j+i);
        if samplesRemaining > dsSamples || j + i < fs_timing(2)
            mapping(i,predictedFootstep+1) = 1;
        else
            mapping(i,predictedFootstep+1) = samplesRemaining/dsSamples;
            mapping(i,predictedFootstep+2) = 1 - samplesRemaining/dsSamples;
        end
    end
    %%Mapping takes care of SS AND DS
    A_zmpconstr = [Pzmp, -mapping(:,2:end), zeros(C, C+F); ...
        zeros(C, C+F), Pzmp, -mapping(:,2:end); ...
        -Pzmp, mapping(:,2:end), zeros(C, C+F); ...
        zeros(C, C+F), -Pzmp, mapping(:,2:end)];
    
    b_zmpconstr = [pzmp * (- (xztot-xz_cam) + wx/2) + mapping(:,1) * current_xfs; ...
        pzmp * (- (yztot-yz_cam) + wy/2) + mapping(:,1) * current_yfs; ...
        - pzmp * (- (xztot-xz_cam) - wx/2) - mapping(:,1) * current_xfs; ...
        - pzmp * (- (yztot-yz_cam) - wy/2) - mapping(:,1) * current_yfs];
    
    if fsCounter == 1
        b_zmpconstr = b_zmpconstr + [ mapping(:,1) * (- wx/2 + initial_wx/2); ...
            mapping(:,1) * (- wy/2 + initial_wy/2); ...
            mapping(:,1) * (- wx/2 + initial_wx/2); ...
            mapping(:,1) * (- wy/2 + initial_wy/2)];
    end
    
    %ZMP CAM
    
    %%% This might be wrong!    
    
    A_zmpconstr_cam = [Pzmp, zeros(C, C); ...
        zeros(C, C), Pzmp; ...
        -Pzmp, zeros(C, C); ...
        zeros(C, C), -Pzmp];
    
    b_zmpconstr_cam = [pzmp*(xz_cam_max-(xztot-xz)); ...
        pzmp*(yz_cam_max-(yztot-yz)); ...
        -pzmp*(-xz_cam_max-(xztot-xz)); ...
        -pzmp*(-yz_cam_max-(yztot-yz))];
    
    % kinematic constraint
    
    difference_matrix = eye(F);
    for i = 2:F
        difference_matrix(i,i-1) = -1;
    end
    
    A_kinconstr_component = blkdiag(zeros(C,C), difference_matrix, zeros(C,C), difference_matrix);
    A_kinconstr = [A_kinconstr_component; - A_kinconstr_component];
    b_kinconstr = zeros(4*(C+F), 1);
    for i = 1:F
        b_kinconstr(C+i) = wkx/2;
        b_kinconstr(3*C+2*F+i) = wkx/2;
        
        % differentiate between left and right footstep
        if mod(fsCounter+i, 2) == 1
            b_kinconstr(2*C+F+i) = ell + wky/2;
            b_kinconstr(4*C+3*F+i) = -(ell - wky/2);
        else
            b_kinconstr(2*C+F+i) = -ell + wky/2;
            b_kinconstr(4*C+3*F+i) = -(-ell - wky/2);
        end
    end
    
    
    % add the contribution from the current footstep
    b_kinconstr(C+1) = b_kinconstr(C+1) + current_xfs;
    b_kinconstr(2*C+F+1) = b_kinconstr(2*C+F+1) + current_yfs;
    b_kinconstr(3*C+2*F+1) = b_kinconstr(3*C+2*F+1) - current_xfs;
    b_kinconstr(4*C+3*F+1) = b_kinconstr(4*C+3*F+1) - current_yfs;
    
    % fixme: remove the constraint for the first footstep
    if fsCounter == 1
        b_kinconstr(C+1) = 1000000;
        b_kinconstr(2*C+F+1) = 1000000;
        b_kinconstr(3*C+2*F+1) = 1000000;
        b_kinconstr(4*C+3*F+1) = 1000000;
    end
    
    % stability constraint (numerical anticipative)
    
    lambda = exp(- eta * mpcTimeStep);
    b_exp = exp(- eta * mpcTimeStep * (0:C-1))';
    
    % ANTICIPATIVE TAIL
    %     A_stabconstr = [(1/eta)*(1-lambda) * b_exp' - mpcTimeStep * ones(1,C)*exp(- eta * mpcTimeStep * C), ...
    %         zeros(1,F+C+F); ...
    %         zeros(1,C+F), ...
    %         (1/eta)*(1-lambda) * b_exp' - mpcTimeStep * ones(1,C)*exp(- eta * mpcTimeStep * C), ...
    %         zeros(1,F)];
    
    %     b_stabconstr = [x + xd/eta - xz*(1-exp(- eta * mpcTimeStep * C)) - anticipative_x; ...
    %         y + yd/eta - yz*(1-exp(- eta * mpcTimeStep * C)) - anticipative_y];
    
    % Periodic tale
    A_stabconstr = [(1/eta)*((1-lambda)/(1-lambda^C)) * b_exp', ...
        zeros(1,F+C+F); ...
        zeros(1,C+F), ...
        (1/eta)*(1-lambda)/(1-lambda^C) * b_exp', ...
        zeros(1,F)];
    b_stabconstr = [(xtot-x_cam) + (xdtot-xd_cam)/eta - (xztot-xz_cam) ; ...
        (ytot-y_cam) + (ydtot-yd_cam)/eta - (yztot-yz_cam) ];
    %
    % Truncated tail
    %         A_stabconstr = [(1/eta)*(1-lambda)* b_exp' - lambda^C*mpcTimeStep * pzmp', ...
    %             zeros(1,F+C+F); ...
    %             zeros(1,C+F), ...
    %             (1/eta)*(1-lambda)* b_exp' - lambda^C*mpcTimeStep * pzmp', ...
    %             zeros(1,F)];
    %         b_stabconstr = [x + xd/eta - xz ; ...
    %             y + yd/eta - yz ];
    
    %stability constraint (numerical anticipative) cam %%%paper uses periodic tail
    %Truncated tail
    %     A_stabconstr_cam = [(1/eta)*(1-lambda)* b_exp' - lambda^C*mpcTimeStep * pzmp', ...
    %         zeros(1,C); ...
    %         zeros(1,C), ...
    %         (1/eta)*(1-lambda)* b_exp' - lambda^C*mpcTimeStep * pzmp'];
    %
    %     b_stabconstr_cam = [x_cam + xd_cam/eta - xz_cam ; ...
    %         y_cam + yd_cam/eta - yz_cam ];
    
    %Periodic tail
    A_stabconstr_cam = [(1/eta)*((1-lambda)/(1-lambda^C)) * b_exp', ...
        zeros(1,C); ...
        zeros(1,C), ...
        (1/eta)*((1-lambda)/(1-lambda^C)) * b_exp'];
    b_stabconstr_cam = [(xtot-x) + (xdtot-xd)/eta - (xztot-xz) ; ...
        (ytot-y) + (ydtot-yd)/eta - (yztot-yz)];
    
%     A_stabconstrwithcam = [(1/eta)*((1-lambda)/(1-lambda^C)) * b_exp', ...
%         zeros(1,F+C+F), (1/eta)*((1-lambda)/(1-lambda^C)) * b_exp', ...
%         zeros(1,C); ...
%         zeros(1,C+F), ...
%         (1/eta)*(1-lambda)/(1-lambda^C) * b_exp', ...
%         zeros(1,F), zeros(1,C), ...
%         (1/eta)*((1-lambda)/(1-lambda^C)) * b_exp'];
%     
%     b_stabconstrwithcam = [x + x_cam + (xd_cam + xd)/eta - xz - xz_cam ; ...
%         y + y_cam + (yd+yd_cam)/eta - yz - yz_cam ];

    %Terminal constraint cam (%%%don't know if it is right or not(see
    %paper))
    delta_remaining = zeros(1,C);
    n_remaining = j - fs_timing(fsCounter) + 1; % According to paper n must be >= 3
    if(n_remaining > fs_timing(2)-2)
        n_remaining = fs_timing(2)-2;
    end
%     delta_remaining(1,n_remaining:fs_timing(2)) = mpcTimeStep;
    A_termconstr_cam = [delta_remaining, zeros(1,C); ...
        zeros(1,C), delta_remaining];
    b_termconstr_cam = zeros(2,1);
%     b_termconstr_cam = [-xz_cam; -yz_cam];
    
    
    % cost function
    
    Qzdot = 10;
    Qfootsteps = 1000000*Qzdot;
    Qzdot_cam = 0;
    H_half = blkdiag(Qzdot * eye(C,C), Qfootsteps * eye(F,F));
    H_half_cam = blkdiag(Qzdot_cam * eye(C,C));
    H = blkdiag(H_half, H_half, H_half_cam, H_half_cam);
    f = [zeros(C,1); - Qfootsteps * fs_plan(fsCounter+1:fsCounter+F,1); ...
        zeros(C,1); - Qfootsteps * fs_plan(fsCounter+1:fsCounter+F,2); ...
        zeros(C,1); zeros(C,1)]; 
    % solve QP
    
    %concatenating A and b matricies
    A_zmpconstrwithcam = blkdiag(A_zmpconstr, A_zmpconstr_cam);
    b_zmpconstrwithcam = [b_zmpconstr; b_zmpconstr_cam];
    A_kinconstrwithcam = [A_kinconstr, zeros(size(A_kinconstr,1), 2*C)];
    A_termconstrwithcam = [zeros(size(A_termconstr_cam,1),2*C+2*F), A_termconstr_cam];
    A_stabconstrwithcam = blkdiag(A_stabconstr, A_stabconstr_cam);
    b_stabconstrwithcam = [b_stabconstr; b_stabconstr_cam];

    options = optimset('Display','off');
    decisionVariables = quadprog(H, f, [A_zmpconstrwithcam; A_kinconstrwithcam], [b_zmpconstrwithcam; b_kinconstr], ...
        A_stabconstrwithcam, b_stabconstrwithcam,...
        [],[],[],options);
    
    predicted_xzd = decisionVariables(1:C);
    predicted_xfs = decisionVariables(C+1:C+F);
    predicted_yzd = decisionVariables(C+F+1:C+F+C);
    predicted_yfs = decisionVariables(C+F+C+1:C+F+C+F);
    
    predicted_xzd_cam = decisionVariables(C+F+C+F+1:C+F+C+F+C);
    predicted_yzd_cam = decisionVariables(C+F+C+F+C+1:C+F+C+F+C+C);
        
    % update state and store
    
    predstate_x = [x; xd; xz];
    predstate_y = [y; yd; yz];
    
    predstate_x_cam = [x_cam; xd_cam; xz_cam];
    predstate_y_cam = [y_cam; yd_cam; yz_cam];
    
    for i = 1:C
        predstate_x = A_upd * predstate_x + B_upd * predicted_xzd(i);
        predicted_x(i) = predstate_x(1);
        predicted_xd(i) = predstate_x(2);
        predicted_xz(i) = predstate_x(3);
        
        predstate_y = A_upd * predstate_y + B_upd * predicted_yzd(i);
        predicted_y(i) = predstate_y(1);
        predicted_yd(i) = predstate_y(2);
        predicted_yz(i) = predstate_y(3);
        
        predstate_x_cam = A_upd_cam * predstate_x_cam + B_upd_cam * predicted_xzd_cam(i);
        predicted_x_cam(i) = predstate_x_cam(1);
        predicted_xd_cam(i) = predstate_x_cam(2);
        predicted_xz_cam(i) = predstate_x_cam(3);
        
        predstate_y_cam = A_upd_cam * predstate_y_cam + B_upd_cam * predicted_yzd_cam(i);
        predicted_y_cam(i) = predstate_y_cam(1);
        predicted_yd_cam(i) = predstate_y_cam(2);
        predicted_yz_cam(i) = predstate_y_cam(3);
        
    end
    
    %Setting new states
    x = predicted_x(1);
    xd = predicted_xd(1);
    xz = predicted_xz(1);
    
    y = predicted_y(1);
    yd = predicted_yd(1);
    yz = predicted_yz(1);
    
    x_cam = predicted_x_cam(1);
    xd_cam = predicted_xd_cam(1);
    xz_cam = predicted_xz_cam(1);
    
    y_cam = predicted_y_cam(1);
    yd_cam = predicted_yd_cam(1);
    yz_cam = predicted_yz_cam(1);
    
    xtot = x + x_cam;
    xdtot = xd + xd_cam;
    xztot = xz + xz_cam;
    
    ytot = y + y_cam;
    ydtot = yd + yd_cam;
    yztot = yz + yz_cam;
    
    
    %Torques Desired calculation
    
    tx_des_cam = -yz_cam*(m*9.81);
    ty_des_cam = xz_cam*(m*9.81);
    tz_des_cam = ((x-xz)*tx_des_cam + (y-yz)*ty_des_cam)/z0;
    t_des_cam = [tx_des_cam, ty_des_cam, tz_des_cam]';
    thetadd = J\t_des_cam;
    thetad(:,j+1) = thetad(:,j) + thetadd*mpcTimeStep;
    theta(:,j+1) = theta(:,j) + thetad(:,j)*mpcTimeStep + thetadd*mpcTimeStep^2/2;
    
    % store trajectories of both motions
    x_store(end+1) = x;
    xz_store(end+1) = xz;
    y_store(end+1) = y;
    yz_store(end+1) = yz;
    x_store_cam(end+1) = x_cam;
    y_store_cam(end+1) = y_cam;
    xz_store_cam(end+1) = xz_cam;
    yz_store_cam(end+1) = yz_cam;
    tx_store_cam(end+1) = tx_des_cam;
    ty_store_cam(end+1) = ty_des_cam;
    tz_store_cam(end+1) = tz_des_cam;
    
    xtot_store(end+1) = xtot;
    ytot_store(end+1) = ytot;
    xztot_store(end+1) = xztot;
    yztot_store(end+1) = yztot;
    
    
    % plot
    clf
    hold on
    rectangle_initial = [initial_wx/2,initial_wx/2,-initial_wx/2,-initial_wx/2,initial_wx/2; ...
        initial_wy/2,-initial_wy/2,-initial_wy/2,initial_wy/2,initial_wy/2];
    rectangle = [wx/2,wx/2,-wx/2,-wx/2,wx/2; wy/2,-wy/2,-wy/2,wy/2,wy/2];
    
    % plot footsteps (first double support manually added)
    plot(rectangle_initial(1,:), rectangle_initial(2,:),'m','Linewidth',2,'HandleVisibility','off')
    for i = 2:fsCounter
        plot(rectangle(1,:) + xfs_store(i), rectangle(2,:) + yfs_store(i),'m','Linewidth',2,'HandleVisibility','off');
    end
%     % Plot double support
%     for i = 2:fsCounter
%         diag_mx = [-rectangle(1,1)+ xfs_store(i-1), -rectangle(1,1) + xfs_store(i)];
%         diag_px = [rectangle(1,1)+ xfs_store(i-1), rectangle(1,1) + xfs_store(i)];
%         diag_mxupd = [-rectangle(1,1)+ xfs_store(fsCounter), -rectangle(1,1) + predicted_xfs(1)];
%         diag_pxupd = [rectangle(1,1)+ xfs_store(fsCounter), rectangle(1,1) + predicted_xfs(1)];
%         
%         if(mod(i,2))
%             diag_y = [rectangle(2,1) + yfs_store(i-1), rectangle(2,2) + yfs_store(i)];
%             diag_yupd = [rectangle(2,2) + yfs_store(fsCounter), rectangle(2,1) + predicted_yfs(1)];
%         else
%             diag_y = [rectangle(2,2) + yfs_store(i-1), rectangle(2,1) + yfs_store(i)];
%             diag_yupd = [rectangle(2,1) + yfs_store(fsCounter), rectangle(2,2) + predicted_yfs(1)];
%         end
%         plot(diag_px,diag_y,'k:','Linewidth',1,'HandleVisibility','off');
%         plot(diag_mx,diag_y,'k:','Linewidth',1,'HandleVisibility','off');
%         plot(diag_mxupd,diag_yupd,'k:','Linewidth',1,'HandleVisibility','off');
%         plot(diag_pxupd,diag_yupd,'k:','Linewidth',1,'HandleVisibility','off');        
%     end
    for i = 1:F
        plot(predicted_xfs(i),predicted_yfs(i),'*k','HandleVisibility','off')
        plot(rectangle(1,:) + predicted_xfs(i), rectangle(2,:) + predicted_yfs(i),'m','Linewidth',2,'HandleVisibility','off')
    end
    plot(xtot_store, ytot_store, 'r', 'Linewidth', 2);
    plot(xztot_store, yztot_store, 'b', 'Linewidth', 2);
    plot(predicted_x + predicted_x_cam, predicted_y + predicted_y_cam, 'g', 'Linewidth', 2);
    
    axis equal
    axis([-0.2,0.7,-0.15,0.15])
    grid on
    xlabel('x (m)');
    ylabel('y (m)');
    legend('CoM','ZMP','Location','northeast')
    drawnow
    
    % update some things
    
    if j+1 >= fs_timing(fsCounter + 1)
        disp('updating footstep counter')
        
        fsCounter = fsCounter + 1;
        current_xfs = predicted_xfs(1);
        current_yfs = predicted_yfs(1);
        xfs_store(end+1) = predicted_xfs(1);
        yfs_store(end+1) = predicted_yfs(1);
    end
    
end
%%
% Desired torque
% close all;clc;
fig = figure;
Fig = struct('cdata', cell(1,j), 'colormap', cell(1,j));
theta = theta(:,2:end);
theta_x = theta(1,1:j); theta_y = theta(2,1:j); theta_z = theta(3,1:j);
for iter = 1:j
    while theta_x(iter) >= 2*pi
        theta_x(iter) = theta_x(iter) - 2*pi;
    end
    while theta_y(iter) >= 2*pi
        theta_y(iter) = theta_y(iter) - 2*pi;
    end
    while theta_z(iter) >= 2*pi
        theta_z(iter) = theta_z(iter) - 2*pi;
    end
    while theta_x(iter) <= -2*pi
        theta_x(iter) = theta_x(iter) + 2*pi;
    end
    while theta_y(iter) <= -2*pi
        theta_y(iter) = theta_y(iter) + 2*pi;
    end
    while theta_z(iter) <= -2*pi
        theta_z(iter) = theta_z(iter) + 2*pi;
    end
end
for k = 1:j
    clf
    hold on
    
    a=0.1; % horizontal radius
    b=0.3; % vertical radius
    xplot0=x_store(k); % x0,y0 ellipse centre coordinates
    zplot0=z0;
    plot(xplot0,zplot0,'k.','Markersize',15);
    t = linspace(0,2*pi,100);
    xplot= xplot0 + a*cos(t)*cos(theta_y(k))...
        - b*sin(t)*sin(theta_y(k));
    zplot= zplot0 + b*sin(t)*cos(theta_y(k))...
        + a*cos(t)*sin(theta_y(k));
    plot(xplot,zplot,'b');
    xlabel("x (m)");
    ylabel("z (m)");
    xpend(1) = xz_store(k);
    zpend(1) = 0;
    xpend(2) = xplot0;
    zpend(2) = zplot0;
    plot(xpend,zpend,'black','Markersize',2);
    grid on
    axis equal
    axis([-0.2, 0.7, 0, 1.5])
    drawnow
    Fig(k) = getframe(fig);
end
% Plot rotation angles of vme
figure;
subplot(1,2,1);
plot(theta_x);
xlabel('time (samples)');
ylabel('pitch angle (rad)');
title('Rotation around x-axis');
grid on
subplot(1,2,2);
plot(theta_y);
xlabel('time (samples)');
ylabel('roll angle (rad)');
title('Rotation around y-axis');
grid on

% % Write video to file
% writerObj = VideoWriter('vme.avi');
% % writerObj.FrameRate = 100;
% % set the seconds per image
% % open the video writer
% open(writerObj);
% % write the frames to the video
% for i=1:length(Fig)
%     % convert the image to a frame
%     frame = Fig(i) ;
%     writeVideo(writerObj, frame);
% end
% % close the writer object
% close(writerObj);