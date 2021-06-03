clear; clc; close all;
VIP = 1;
% footstep plan and footstep timings
% Choose 'Normal' for standard footsteps, 'Weird' for varying lengths of
% the footsteps and 'FwdRev' for forward then reverse footsteps
NF = 10;
fs_plan = footstep('Normal');
fs_timing = [0; 50; 100; 150; 200; 250; 300; 350; 400; 450];
simDuration = 1000;
% general MPC parameters
C = 100; %Control Horizon
P = 200; %Predition Horizon
F = 3; %
mpcTimeStep = 0.01;
Z0 = 0.75; %constant height = 0.75
eta = sqrt(9.81/Z0);
dsSamples = 20; %Double support samples 20 per 50
% ZMP constraint parameters
wx = 0.08;  %dzx in paper
wy = 0.08; %dzy in paper
initial_wx = 0.08;
initial_wy = 0.08/2 + 0.2;

%ZMP cam %according to paper
MAX_TORQUEX = 100; %Max torque in x
MAX_TORQUEY = 100; %max torque in y
m = 6.67348; %mass of torso
%Moment of inertia of torso
Jxx = 0.344345670122806; Jyy = 0.338324801980916; Jzz = 0.048160214086886;
Jyz = 0.003172978374495; Jxz = 0.045048560212699; Jxy = -5.676994253777424e-04;
J = [Jxx, Jxy, Jxz; Jxy, Jyy, Jyz; Jxz, Jyz, Jzz];
xzcam_max = MAX_TORQUEY/(m*9.81);
yzcam_max = MAX_TORQUEX/(m*9.81);

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
xcam = 0.0;
xdcam = 0.0;
xzcam = 0.0;
ycam = 0.0;
ydcam = 0.0;
yzcam = 0.0;

% state inizialazation total
xtot = x + xcam;
xdtot = xd + xdcam;
xztot = xz + xzcam;
ytot = y + ycam;
ydtot = yd + ydcam;
yztot = yz + yzcam;

% state update matrices
ch = cosh(eta*mpcTimeStep);
sh = sinh(eta*mpcTimeStep);
A_upd = [ch, sh/eta, 1-ch; eta* sh, ch, -eta*sh; 0, 0, 1];
B_upd = [mpcTimeStep-sh/eta; 1-ch; mpcTimeStep];

% stored trajectories
x_store = x;
xz_store = xz;
y_store = y;
yz_store = yz;
xfs_store = current_xfs;
yfs_store = current_yfs;

% stored trajectories cam
xcam_store = xcam;
xzcam_store = xzcam;
ycam_store = ycam;
yzcam_store = yzcam;
tx_store = 0;
ty_store = 0;
tz_store = 0;
xzd_store = 0;
yzd_store = 0;
xzdcam_store = 0;
yzdcam_store = 0;
xzdtot_store = 0;
yzdtot_store = 0;

% stored trajectories total
xtot_store = xtot;
xztot_store = xztot;
ytot_store = ytot;
yztot_store = yztot;

%Angular velocity and position
thetad = zeros(3,simDuration);
theta = zeros(3,simDuration);
THETA_MAX = [pi/6, pi/6, 1e6]';

% ZMP constraint constant matrices
Pzmp = tril(ones(C,C)) * mpcTimeStep;
pzmp = ones(C,1);

% Angle constraint constant matrices
Pmg = tril(ones(C),-1)*m*9.81*mpcTimeStep;
pmg = pzmp*m*9.81;
Pthdd = (triu(ones(C),1)*mpcTimeStep)*Pzmp' + (triu(ones(C))*mpcTimeStep^2/2);

%Constructing xcdd in terms of the decision variables
%Constructing Matrix of A's to represent COM acceleration in terms of ZMP
%velocities
Trans_mat = zeros(3*C,3);
Trans_mat(1:3,:) = A_upd;
for i = 4:3:3*C
    Trans_mat(i:i+2,:) = Trans_mat(i-3:i-1,:)*A_upd;
end
%Constructing 3CxC Contr matrix
Contr_mat = kron(eye(C,C),B_upd);
for i = 1:3:3*C
    j = (i-1)/3;
    while j > 0
        Contr_mat(i:i+2,j) = A_upd*Contr_mat(i:i+2,j+1);
        j = j-1;
    end
end
%Constructing matrix to extract xc^k
Kcdd = kron(eye(C,C),[1 0 0]);
Pcdd = eta^2 *(Kcdd*Contr_mat - Pzmp);
for j = 1:simDuration
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
    
    b_zmpconstr = [pzmp * (- (xz) + wx/2) + mapping(:,1) * current_xfs; ...
        pzmp * (- (yz) + wy/2) + mapping(:,1) * current_yfs; ...
        - pzmp * (- (xz) - wx/2) - mapping(:,1) * current_xfs; ...
        - pzmp * (- (yz) - wy/2) - mapping(:,1) * current_yfs];
    
    if fsCounter == 1
        b_zmpconstr = b_zmpconstr + [ mapping(:,1) * (- wx/2 + initial_wx/2); ...
            mapping(:,1) * (- wy/2 + initial_wy/2); ...
            mapping(:,1) * (- wx/2 + initial_wx/2); ...
            mapping(:,1) * (- wy/2 + initial_wy/2)];
    end
    %ZMP CAM
    A_zmpconstr_cam = [Pzmp, zeros(C, C); ...
        zeros(C, C), Pzmp; ...
        -Pzmp, zeros(C, C); ...
        zeros(C, C), -Pzmp];
    
    b_zmpconstr_cam = [pzmp*(xzcam_max-xzcam); ...
        pzmp*(yzcam_max-yzcam); ...
        -pzmp*(-xzcam_max-xzcam); ...
        -pzmp*(-yzcam_max-yzcam)];
    
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
    %Periodic tail
    A_stabconstr = [(1/eta)*((1-lambda)/(1-lambda^C)) * b_exp', ...
        zeros(1,F+C+F); ...
        zeros(1,C+F), ...   
        (1/eta)*(1-lambda)/(1-lambda^C) * b_exp', ...
        zeros(1,F)];
    b_stabconstr = [(x) + (xd)/eta - (xz) ; ...
        (y) + (yd)/eta - (yz)];
    %Periodic tail CAM
    A_stabconstr_cam = [(1/eta)*((1-lambda)/(1-lambda^C)) * b_exp', ...
        zeros(1,C); ...
        zeros(1,C), ...
        (1/eta)*((1-lambda)/(1-lambda^C)) * b_exp'];
    b_stabconstr_cam = [xcam + xdcam/eta - xzcam ; ...
        ycam + ydcam/eta - yzcam];
    %Terminal constraint cam (%%%don't know if it is right or not(see
    %paper))
    delta_remaining = zeros(1,C);
    n_remaining = j - fs_timing(fsCounter) + 1; % According to paper n must be >= 3
    if(n_remaining > fs_timing(2)-2)
        n_remaining = fs_timing(2)-2;
    end
    delta_remaining(1,n_remaining:fs_timing(2)) = mpcTimeStep;
    A_termconstr_cam = [delta_remaining, zeros(1,C); ...
        zeros(1,C), delta_remaining];
    b_termconstr_cam = [-xzcam; -yzcam];
    
    %Angle constraint
    Txdes = -yzcam*pmg;
    Tydes = xzcam*pmg;
    Tzdes = ((xtot-xz)*Txdes+(ytot-yz)*Tydes)/Z0;
    Tdes = [Txdes'; Tydes'; Tzdes'];
    rhs = THETA_MAX*pzmp' - theta(:,j)*pzmp' - thetad(:,j)*pzmp'*Pzmp';
    rhs = J*rhs;
    rhs = rhs - Tdes*Pthdd;
    rhs = rhs(1:2,:);
    rhs = rhs';
    
    lhs = -THETA_MAX*pzmp' - theta(:,j)*pzmp' - thetad(:,j)*pzmp'*Pzmp';
    lhs = J*lhs;
    lhs = lhs - Tdes*Pthdd;
    lhs = lhs(1:2,:);
    lhs = lhs';
    
    A_angleconstr = [zeros(C,C), -Pthdd'*Pmg; ...
        Pthdd'*Pmg, zeros(C,C); ...
        zeros(C), Pthdd'*Pmg; ...
        -Pthdd'*Pmg, zeros(C)];
    b_angleconstr = [rhs(:); -lhs(:)];
    
    % cost function
    if(VIP)
        A_zmpconstrfull = [A_zmpconstr, zeros(size(A_zmpconstr_cam)); ...
            zeros(size(A_zmpconstr_cam,1),2*C+2*F), A_zmpconstr_cam];
        b_zmpconstrfull = [b_zmpconstr ; b_zmpconstr_cam];
        A_stabconstrfull = blkdiag(A_stabconstr, A_stabconstr_cam);
        b_stabconstrfull = [b_stabconstr; b_stabconstr_cam];
    else
        A_zmpconstrfull = [A_zmpconstr, zeros(size(A_zmpconstr_cam))];
        b_zmpconstrfull = b_zmpconstr;
        A_stabconstrfull = blkdiag(A_stabconstr, zeros(size(A_stabconstr_cam)));
        b_stabconstrfull = [b_stabconstr; zeros(size(b_stabconstr_cam))];
    end
    A_kinconstrfull = [A_kinconstr, zeros(size(A_kinconstr,1), 2*C)];
    A_termconstrfull = [zeros(size(A_termconstr_cam,1),2*C+2*F), A_termconstr_cam];
    A_angleconstrfull = [zeros(size(A_angleconstr,1),2*C+2*F), A_angleconstr];
    
    Qzdot = 10;
    Qzdot_cam = 1;
    Qcddot = 0;
    Qfootsteps = 1e15;
    Qzmp = 1;
    
    H_pcdd = (Qcddot*eye(C))*(Pcdd.'*Pcdd);
    f_xcdd1 = 2*eta^2 *((Kcdd*Trans_mat)*[xtot; xdtot; xztot] - ...
        pzmp*xztot);
    f_ycdd1 = 2*eta^2 *((Kcdd*Trans_mat)*[ytot; ydtot; yztot] - ...
        pzmp*yztot);
    
    f_xcdd = (Qcddot*(f_xcdd1).'*Pcdd).';
    f_ycdd = (Qcddot*(f_ycdd1).'*Pcdd).';
    
    H_half = blkdiag(Qzdot * eye(C,C), Qfootsteps * eye(F,F));
    H = blkdiag(H_half, H_half, Qzdot_cam*eye(C,C), Qzdot_cam*eye(C,C));
    f = [zeros(C,1); - Qfootsteps * fs_plan(fsCounter+1:fsCounter+F,1); ...
        zeros(C,1); - Qfootsteps * fs_plan(fsCounter+1:fsCounter+F,2); ...
        zeros(C,1); zeros(C,1)];
    
    %Applying the cdd effect on the H-matrix
    H(1:C, 1:C) = H(1:C,1:C) + H_pcdd;
    H(C+F+1:2*C+F, C+F+1:2*C+F) = H(C+F+1:2*C+F, C+F+1:2*C+F) + H_pcdd;
    
    if(VIP)
        %delta xzd^2
        H(2*C+2*F+1:3*C+2*F, 2*C+2*F+1:3*C+2*F) = ...
            H(2*C+2*F+1:3*C+2*F, 2*C+2*F+1:3*C+2*F) + H_pcdd;
        H(3*C+2*F+1:4*C+2*F, 3*C+2*F+1:4*C+2*F) = ...
            H(3*C+2*F+1:4*C+2*F, 3*C+2*F+1:4*C+2*F) + H_pcdd;
        %2(delta xzd)' H xzd
        H(1:C, C+C+F+F+1:C+C+F+F+C) = H(1:C, C+C+F+F+1:C+C+F+F+C) + H_pcdd;
        H(C+F+1:C+F+C, C+C+F+F+C+1:C+C+F+F+C+C) = ...
            H(C+F+1:C+F+C, C+C+F+F+C+1:C+C+F+F+C+C) + H_pcdd;
        
        H(C+C+F+F+1:C+C+F+F+C, 1:C) = H(C+C+F+F+1:C+C+F+F+C, 1:C) + H_pcdd;
        H(C+C+F+F+C+1:C+C+F+F+C+C, C+F+1:C+F+C) = ...
            H(C+C+F+F+C+1:C+C+F+F+C+C, C+F+1:C+F+C) + H_pcdd;
    end
    
    %Applying the cdd effect on the f-vector
    f(1:C) = f(1:C) + f_xcdd;
    f(C+F+1:2*C+F) = f(C+F+1:2*C+F) + f_ycdd;
    
    if(VIP)
        f(2*C+2*F+1:3*C+2*F) = f(2*C+2*F+1:3*C+2*F) + f_xcdd;
        f(3*C+2*F+1:4*C+2*F) = f(3*C+2*F+1:4*C+2*F) + f_ycdd;
    end
    % solve QP
    options = optimset('Display','off');
    %     if(VIP)
    %         decisionVariables = quadprog(H, f, ...
    %             [A_zmpconstrfull; A_kinconstrfull; A_angleconstrfull],...
    %             [b_zmpconstrfull; b_kinconstr; b_angleconstr], ...
    %             A_stabconstrfull, b_stabconstrfull,...
    %             [],[],[],options);
    %     else
    [decisionVariables,fval,exitflag,output,lambda] = ...
        quadprog(H, f, ...
        [A_zmpconstrfull; A_kinconstrfull],...
        [b_zmpconstrfull; b_kinconstr], ...
        A_stabconstrfull, b_stabconstrfull,...
        [],[],[],options);
    %     end
    predicted_xzd = decisionVariables(1:C);
    predicted_xfs = decisionVariables(C+1:C+F);
    predicted_yzd = decisionVariables(C+F+1:C+F+C);
    predicted_yfs = decisionVariables(C+F+C+1:C+F+C+F);
    predicted_xzdcam = decisionVariables(C+F+C+F+1:C+F+C+F+C);
    predicted_yzdcam = decisionVariables(C+F+C+F+C+1:C+F+C+F+C+C);
    
    % update state and store
    
    predstate_x = [x; xd; xz];
    predstate_y = [y; yd; yz];
    predstate_xcam = [xcam; xdcam; xzcam];
    predstate_ycam = [ycam; ydcam; yzcam];
    
    predicted_x = zeros(1,C); predicted_xcam = zeros(1,C);
    predicted_xd = zeros(1,C); predicted_xdcam = zeros(1,C);
    predicted_xz = zeros(1,C); predicted_xzcam = zeros(1,C);
    predicted_y = zeros(1,C); predicted_ycam = zeros(1,C);
    predicted_yd = zeros(1,C); predicted_ydcam = zeros(1,C);
    predicted_yz = zeros(1,C); predicted_yzcam = zeros(1,C);
    
    xzd_store(end+1) = predicted_xzd(1);
    yzd_store(end+1) = predicted_yzd(1);
    xzdcam_store(end+1) = predicted_xzdcam(1);
    yzdcam_store(end+1) = predicted_yzdcam(1);
    xzdtot_store(end+1) = xzd_store(end) + xzdcam_store(end);
    yzdtot_store(end+1) = yzd_store(end) + yzdcam_store(end);
    
    for i = 1:C
        predstate_x = A_upd * predstate_x + B_upd * predicted_xzd(i);
        predicted_x(i) = predstate_x(1);
        predicted_xd(i) = predstate_x(2);
        predicted_xz(i) = predstate_x(3);
        
        predstate_y = A_upd * predstate_y + B_upd * predicted_yzd(i);
        predicted_y(i) = predstate_y(1);
        predicted_yd(i) = predstate_y(2);
        predicted_yz(i) = predstate_y(3);
        
        predstate_xcam = A_upd * predstate_xcam + B_upd * predicted_xzdcam(i);
        predicted_xcam(i) = predstate_xcam(1);
        predicted_xdcam(i) = predstate_xcam(2);
        predicted_xzcam(i) = predstate_xcam(3);
        
        predstate_ycam = A_upd * predstate_ycam + B_upd * predicted_yzdcam(i);
        predicted_ycam(i) = predstate_ycam(1);
        predicted_ydcam(i) = predstate_ycam(2);
        predicted_yzcam(i) = predstate_ycam(3);
        
    end
    
    %Setting new states
    x = predicted_x(1);
    xd = predicted_xd(1);
    xz = predicted_xz(1);
    
    y = predicted_y(1);
    yd = predicted_yd(1);
    yz = predicted_yz(1);
    
    xcam = predicted_xcam(1);
    xdcam = predicted_xdcam(1);
    xzcam = predicted_xzcam(1);
    
    ycam = predicted_ycam(1);
    ydcam = predicted_ydcam(1);
    yzcam = predicted_yzcam(1);
    
    %Torques Desired calculation
    
    txdes = -yzcam*(m*9.81);
    tydes = xzcam*(m*9.81);
    tzdes = ((xtot-xz)*txdes + (ytot-yz)*tydes)/Z0;
    tdes = [txdes, tydes, tzdes]';
    thetadd = J\tdes;
    thetad(:,j+1) = thetad(:,j) + thetadd*mpcTimeStep;
    theta(:,j+1) = theta(:,j) + thetad(:,j)*mpcTimeStep + thetadd*mpcTimeStep^2/2;
    
    %total of both states
    xtot = x + xcam;
    xdtot = xd + xdcam;
    xztot = xz + xzcam;
    
    ytot = y + ycam;
    ydtot = yd + ydcam;
    yztot = yz + yzcam;
    
    % store trajectories
    x_store(end+1) = x;
    xz_store(end+1) = xz;
    y_store(end+1) = y;
    yz_store(end+1) = yz;
    xcam_store(end+1) = xcam;
    ycam_store(end+1) = ycam;
    xzcam_store(end+1) = xzcam;
    yzcam_store(end+1) = yzcam;
    tx_store(end+1) = txdes;
    ty_store(end+1) = tydes;
    tz_store(end+1) = tzdes;
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
    
    %     plot footsteps (first double support manually added)
    plot(rectangle_initial(1,:), rectangle_initial(2,:),'m','Linewidth',2,'HandleVisibility','off')
    for i = 2:fsCounter
        plot(rectangle(1,:) + xfs_store(i), rectangle(2,:) + yfs_store(i),...
            'm','Linewidth',2,'HandleVisibility','off');
    end
    for i = 1:F
        plot(predicted_xfs(i),predicted_yfs(i),'*k','HandleVisibility','off')
        plot(rectangle(1,:) + predicted_xfs(i), ...
            rectangle(2,:) + predicted_yfs(i),'m','Linewidth',2,'HandleVisibility','off')
    end
    plot(xtot_store, ytot_store, 'r', 'Linewidth', 2);
    plot(xztot_store, yztot_store, 'b', 'Linewidth', 2);
    plot(predicted_x+predicted_xcam, predicted_y+predicted_ycam, 'g', 'Linewidth', 2);
    axis equal
    axis([-0.2,1.2,-0.15,0.15])
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
    %     if j == 267
    %         xd = xd*2;
    %         yd = yd*2;
    %     end
end
%% Animation vme
clear rectangle;
thetax = theta(1,1:j); thetay = theta(2,1:j); thetaz = theta(3,1:j);
fig = figure;
Fig = struct('cdata', cell(1,j), 'colormap', cell(1,j));
DoubleSwitch = 1;
for k = 1:j
    if(~mod((k-1),50))
        timeperfoot = 1;
    end
    if(~mod((k-1),25))
        DoubleSwitch = 1;
    end
    clf
    hold on
    a=0.1; % horizontal radius
    b=0.3; % vertical radius
    xplot0 = xtot_store(k);
    yplot0 = ytot_store(k);
    zplot0=Z0;
    t = linspace(0,2*pi,100);
    xplot= xplot0 + a*cos(t)*cos(thetay(k))...
        - b*sin(t)*sin(thetay(k));
    yplot= yplot0 + a*cos(t)*cos(thetax(k))...
        -b*sin(t)*sin(thetax(k));
    zplot= zplot0 + b*sin(t)*cos(thetay(k))...
        + a*cos(t)*sin(thetay(k));
    zplot2= zplot0 + b*sin(t)*cos(thetax(k))...
        + a*cos(t)*sin(thetax(k));
    xpend(1) = fs_plan(floor(k/50)+1,1);
    ypend(1) = fs_plan(floor(k/50)+1,2);
    zpend(1) = 0;
    xpend(2) = xplot0;
    ypend(2) = yplot0;
    zpend(2) = zplot0;
    
    ascend = (0.5:0.01:1);
    descend = (0.5:-0.01:0);
    
    %     xstep = fs_plan(floor(k/50)+1,1):(fs_plan(floor(k/50)+2,1)-fs_plan(floor(k/50)+1,1))/25:fs_plan(floor(k/50)+2,1);
    %     if(DoubleSwitch < 26 && timeperfoot ~=1 || floor(k/50) < 1)
    %         xpendstep(1) = fs_plan(floor(k/50)+1,1);
    %     else
    %         %xpendstep(1) = fs_plan(floor(k/50)+2,1)*ascend(timeperfoot);
    %         xpendstep(1) = xstep(DoubleSwitch);
    %     end
    
    width = 0.08; % whatever
    height = 0.01; % whatever ...
    
    ypendstep(1) = fs_plan(floor(k/50)+2,2)*ascend(timeperfoot);
    zpendstep(1) = descend(timeperfoot);
    xpendstep(2) = xplot0;
    ypendstep(2) = yplot0;
    zpendstep(2) = zplot0;
    subplot(1,3,[1 2])
    plot(xplot0,zplot0,'k.','Markersize',15); hold on
    plot(xplot,zplot,'b'); hold on
    plot(xpend,zpend,'black','Markersize',2); hold on
%     xLeft = fs_plan(floor(k/50)+1,1) - width / 2;
%     zBottom = 0 - height / 2;
%     rectangle( 'Position' , [xLeft, zBottom, width, height], 'EdgeColor' , 'b' , 'FaceColor' , 'r' , 'LineWidth' , 2);
%     plot(xpendstep,zpendstep,'black','Markersize',2); hold on
    title('Longitudinal axis');
    xlabel("x (m)");
    ylabel("z (m)");
    grid on
    axis equal
    axis([-0.2, 1.2, 0, 1.5])
    subplot(1,3,3)
    plot(yplot0,zplot0,'k.','Markersize',15); hold on
    plot(yplot,zplot2,'b'); hold on
    plot(ypend,zpend,'black','Markersize',2); hold on
%     yLeft = fs_plan(floor(k/50)+1,2) - width / 2;
%     zBottom = 0 - height / 2;
%     rectangle( 'Position' , [yLeft, zBottom, width, height], 'EdgeColor' , 'b' , 'FaceColor' , 'r' , 'LineWidth' , 2);
    %     plot(ypendstep,zpendstep,'black','Markersize',2); hold on
    title('Lateral axis');
    xlabel("y (m)");
    ylabel("z (m)");
    grid on
    axis equal
    axis([-0.3, 0.3, 0, 1.5])
    drawnow
    Fig(k) = getframe(fig);
    timeperfoot = timeperfoot+1;
    DoubleSwitch = DoubleSwitch+1;
end
disp("MEOW")
% Write video to file
writerObj = VideoWriter('vme.avi');
% writerObj.FrameRate = 100;
% set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(Fig)
    % convert the image to a frame
    frame = Fig(i) ;
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);
%% Plot rotation angles of vme
thetax = wrapToPi(theta(1,1:j));
thetay = wrapToPi(theta(2,1:j));
thetaz = wrapToPi(theta(3,1:j));
figure;
subplot(1,3,1);
plot(thetax);
xlabel('time (samples)');
ylabel('angle (rad)');
title('Rotation around x-axis');
grid on
subplot(1,3,2);
plot(thetay);
xlabel('time (samples)');
ylabel('angle (rad)');
title('Rotation around y-axis');
grid on
subplot(1,3,3);
plot(thetaz);
xlabel('time (samples)');
ylabel('angle (rad)');
title('Rotation around z-axis');
grid on
%% Plot ZMP velocities
figure;
subplot(2,1,1)
plot(xzdcam_store,'r-');
% hold on
% plot(xzdcam_store,'b-');
% legend('xzd','xzdcam','Location','northeast')
xlabel('time [samples]');
ylabel('Xzmp Velocity [m/s]');
title('ZMP velocity in x');
grid on
subplot(2,1,2)
plot(yzdcam_store,'r-');
% hold on
% plot(yzdcam_store,'b-');
% legend('yzd','yzdcam','Location','northeast')
xlabel('time [samples]');
ylabel('Yzmp Velocity [m/s]');
title('ZMP velocity in y');
grid
%%
disp(strcat('xcam from:',num2str(min(xcam_store)),' to:',num2str(max(xcam_store))));
disp(strcat('ycam from:',num2str(min(ycam_store)),' to:',num2str(max(ycam_store))));
disp(strcat('xzcam from:',num2str(min(xzcam_store)),' to:',num2str(max(xzcam_store))));
disp(strcat('yzcam from:',num2str(min(yzcam_store)),' to:',num2str(max(yzcam_store))));