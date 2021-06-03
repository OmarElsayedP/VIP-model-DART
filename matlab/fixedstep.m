clear; clc; close all;
VIP = 1;
Angle_const = 1;
Virt_noise = 1;

Qzdotx = 1;
Qzdoty = 1;
Qzdot_camx = 1e-3;
Qzdot_camy = 1e-3;

% Qzdotx = 1e-2;
% Qzdoty = 1e-1;
% Qzdot_camx = 1e-1;
% Qzdot_camy = 1;

Qcddot = 0;
theta_max = [pi/18, pi/18, 0]';

NF = 20;
steptime = 50;  
fs_plan = varstep(NF);
fs_timing = 0:steptime:steptime*(NF-1);
simDuration = 1000;
% general MPC parameters
C = 100; %Control Horizon
P = 200; %Prediction Horizon
F = 3; %
mpcTimeStep = 0.01;
Z0 = 0.75; %constant height = 0.75
eta = sqrt(9.81/Z0);
dsSamples = 25; %Double support samples 20 per 50
% ZMP constraint parameters
wx = 0.08;  %dzx in paper
wy = 0.08; %dzy in paper
initial_wx = 0.08;
initial_wy = 0.08/2 + 0.2;

%ZMP cam %according to paper
MAX_TORQUEX = 100; %Max torque in x
MAX_TORQUEY = 100; %max torque in y
m = 6.67348; %mass of torso
% m = 50;
%Moment of inertia of torso
Jxx = 0.344345670122806; Jyy = 0.338324801980916; Jzz = 0.048160214086886;
% Jyz = 0.003172978374495; Jxz = 0.045048560212699; Jxy = -5.676994253777424e-04;
Jyz = 0; Jxz = 0; Jxy = 0;
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

fsCounter = 1;
current_xfs = fs_plan(fsCounter,1);
current_yfs = fs_plan(fsCounter,2);

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

A_updcam = [ch, sh/eta, 1-ch; eta* sh, ch, -eta*sh; 0, 0, 1];
B_updcam = [mpcTimeStep-sh/eta; 1-ch; mpcTimeStep];

% A_updcam = [1, mpcTimeStep, 0; eta*eta*mpcTimeStep, 1, -eta*eta*mpcTimeStep; 0, 0, 1];
% B_updcam = [0; 0; mpcTimeStep];


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

%Predefine store terms

%Angular velocity and position
thetad = zeros(3,simDuration);
theta = zeros(3,simDuration);

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
%Constructing m atrix to extract xc^k
Kcdd = kron(eye(C,C),[1 0 0]);
Pcdd = eta^2 *(Kcdd*Contr_mat - Pzmp);
for j = 1:simDuration
    %Generating virtual Ang. momentum
    virt_torq(1,j) = 0;
    if(j>50)
        virt_torq(1,j) = -0.0005*sin(j*0.01*4*pi-pi/2);
%         virt_torq(1,j) = -0.0005*sin(j*0.01*4*pi-pi);
    end
    virt_torq(2,j) = -0.0005*sin(j*0.01*2*pi-4*pi/5);
    % ZMP constraint
    % compute mapping timestep -> predicted footstep number
    %%Mapping takes care of SS AND DS
    mapping = zeros(C,F+1);
    predictedFootstep = 0;
    footx = zeros(C,1);
    footy = zeros(C,1);
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
    for i = 1:size(mapping,2)
        footx = footx + mapping(:,i) * fs_plan(fsCounter-1+i,1);
        footy = footy + mapping(:,i) * fs_plan(fsCounter-1+i,2);
    end
    A_zmpconstr = [Pzmp, zeros(C, C); ...
        zeros(C, C), Pzmp; ...
        -Pzmp, zeros(C, C); ...
        zeros(C, C), -Pzmp];
    
    b_zmpconstr = [pzmp * (- (xz) + wx/2) + footx; ...
        pzmp * (- (yz) + wy/2) + footy; ...
        - pzmp * (- (xz) - wx/2) - footx; ...
        - pzmp * (- (yz) - wy/2) - footy];
    
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
    
    % stability constraint (numerical anticipative)
    
    lambda = exp(- eta * mpcTimeStep);
    b_exp = exp(- eta * mpcTimeStep * (0:C-1))';
    %Periodic tail
    A_stabconstr = [(1/eta)*((1-lambda)/(1-lambda^C)) * b_exp', ...
        zeros(1,C); ...
        zeros(1,C), ...
        (1/eta)*(1-lambda)/(1-lambda^C) * b_exp'];
    b_stabconstr = [(x) + (xd)/eta - (xz) ; ...
        (y) + (yd)/eta - (yz)];
    %Periodic tail CAM
    A_stabconstr_cam = [(1/eta)*((1-lambda)/(1-lambda^C)) * b_exp', ...
        zeros(1,C); ...
        zeros(1,C), ...
        (1/eta)*((1-lambda)/(1-lambda^C)) * b_exp'];
    b_stabconstr_cam = [xcam + xdcam/eta - xzcam ; ...
        ycam + ydcam/eta - yzcam];
    %Angle constraint
    Txdes = -yzcam*pmg;
    Tydes = xzcam*pmg;
    Tzdes = ((xtot-xz)*Txdes+(ytot-yz)*Tydes)/Z0;
    Tdes = [Txdes'; Tydes'; Tzdes'];
    
    % THETA_MAX is the maximum angle VME can rotate to
    rhs = theta_max*pzmp' - theta(:,j)*pzmp' - thetad(:,j)*pzmp'*Pzmp';
    rhs = J*rhs;
    rhs = rhs - Tdes*Pthdd;
    rhs = rhs(1:2,:);
    rhs = rhs';
    
    lhs = -theta_max*pzmp' - theta(:,j)*pzmp' - thetad(:,j)*pzmp'*Pzmp';
    lhs = J*lhs;
    lhs = lhs - Tdes*Pthdd;
    lhs = lhs(1:2,:);
    lhs = lhs';
    
    A_angleconstr = [zeros(C,C), -Pthdd'*Pmg; ...
        Pthdd'*Pmg, zeros(C,C); ...
        zeros(C,C), Pthdd'*Pmg; ...
        -Pthdd'*Pmg, zeros(C,C)];
    b_angleconstr = [rhs(:); -lhs(:)];
    
    % cost function
    if(VIP)
        A_zmpconstrfull = blkdiag(A_zmpconstr, A_zmpconstr_cam);
        b_zmpconstrfull = [b_zmpconstr; b_zmpconstr_cam];
        A_stabconstrfull = blkdiag(A_stabconstr, A_stabconstr_cam);
        b_stabconstrfull = [b_stabconstr; b_stabconstr_cam];
        A_angleconstrfull = [zeros(size(A_angleconstr,1),2*C), A_angleconstr];
    else
        A_zmpconstrfull = [A_zmpconstr, zeros(size(A_zmpconstr_cam))];
        b_zmpconstrfull = b_zmpconstr;
        A_stabconstrfull = [A_stabconstr, zeros(size(A_stabconstr_cam))];
        b_stabconstrfull = b_stabconstr;
        b_angleconstr = 0;
    end
    
    H_pcdd = (Qcddot*eye(C))*(Pcdd.'*Pcdd);
    f_xcdd1 = 2*eta^2 *((Kcdd*Trans_mat)*[xtot; xdtot; xztot] - ...
        pzmp*xztot);
    f_ycdd1 = 2*eta^2 *((Kcdd*Trans_mat)*[ytot; ydtot; yztot] - ...
        pzmp*yztot);
    
    f_xcdd = (Qcddot*(f_xcdd1).'*Pcdd).';
    f_ycdd = (Qcddot*(f_ycdd1).'*Pcdd).';
    
    H = blkdiag(Qzdotx * eye(C,C), Qzdoty * eye(C,C), Qzdot_camx*eye(C,C), Qzdot_camy*eye(C,C));
    f = zeros(4*C,1);
    
    %Applying the cdd effect on the H-matrix
    H(1:C, 1:C) = H(1:C,1:C) + H_pcdd;
    H(C+1:2*C, C+1:2*C) = H(C+1:2*C, C+1:2*C) + H_pcdd;
    if(VIP)
        %delta xzd^2
        H(2*C+1:3*C, 2*C+1:3*C) = ...
            H(2*C+1:3*C, 2*C+1:3*C) + H_pcdd;
        H(3*C+1:4*C, 3*C+1:4*C) = ...
            H(3*C+1:4*C, 3*C+1:4*C) + H_pcdd;
        
        %2(delta xzd)' H xzd
        H(1:C, 2*C+1:3*C) = H(1:C, 2*C+1:3*C) + H_pcdd;
        H(C+1:2*C, 3*C+1:4*C) = H(C+1:2*C, 3*C+1:4*C) + H_pcdd;
        
        H(2*C+1:3*C, 1:C) = H(2*C+1:3*C, 1:C) + H_pcdd;
        H(3*C+1:4*C, C+1:2*C) = H(3*C+1:4*C, C+1:2*C) + H_pcdd;
    end
    
    %Applying the cdd effect on the f-vector
    f(1:C) = f(1:C) + f_xcdd;
    f(C+1:2*C) = f(C+1:2*C) + f_ycdd;
    if(VIP)
        f(2*C+1:3*C) = f(2*C+1:3*C) + f_xcdd;
        f(3*C+1:4*C) = f(3*C+1:4*C) + f_ycdd;
    end
    
    % solve QP
    options = optimset('Display','off');
    if(Angle_const)
        [decisionVariables,fval,exitflag,output,~] = ...
            quadprog(H, f, [A_zmpconstrfull; A_angleconstrfull], ...
            [b_zmpconstrfull; b_angleconstr], A_stabconstrfull, b_stabconstrfull, ...
            [],[],[],options);
    else
        [decisionVariables,fval,exitflag,output,~] = ...
            quadprog(H, f, A_zmpconstrfull, ...
            b_zmpconstrfull, A_stabconstrfull, b_stabconstrfull, ...
            [],[],[],options);
    end
    
    predicted_xzd = decisionVariables(1:C);
    predicted_yzd = decisionVariables(C+1:C+C);
    predicted_xzdcam = decisionVariables(C+C+1:C+C+C);
    predicted_yzdcam = decisionVariables(C+C+C+1:C+C+C+C);
    
    xzd_store(end+1) = predicted_xzd(1);
    yzd_store(end+1) = predicted_yzd(1);
    xzdcam_store(end+1) = predicted_xzdcam(1);
    yzdcam_store(end+1) = predicted_yzdcam(1);
    xzdtot_store(end+1) = xzd_store(end) + xzdcam_store(end);
    yzdtot_store(end+1) = yzd_store(end) + yzdcam_store(end);
    
    %% Model Test
    %Initialize new state
    predicted_x = zeros(1,C); predicted_xcam = zeros(1,C);
    predicted_xd = zeros(1,C); predicted_xdcam = zeros(1,C);
    predicted_xz = zeros(1,C); predicted_xzcam = zeros(1,C);
    predicted_y = zeros(1,C); predicted_ycam = zeros(1,C);
    predicted_yd = zeros(1,C); predicted_ydcam = zeros(1,C);
    predicted_yz = zeros(1,C); predicted_yzcam = zeros(1,C);
    
    % update state and store
    predstate_modx = [x; xd; xz];
    predstate_mody = [y; yd; yz];
    predstate_modxcam = [xcam; xdcam; xzcam];
    predstate_modycam = [ycam; ydcam; yzcam];
    
    %Model robot
    for i = 1:C
        predstate_modx = A_upd * predstate_modx + B_upd * predicted_xzd(i);
        predicted_x(i) = predstate_modx(1);
        predicted_xd(i) = predstate_modx(2);
        predicted_xz(i) = predstate_modx(3);
        
        predstate_mody = A_upd * predstate_mody + B_upd * predicted_yzd(i);
        predicted_y(i) = predstate_mody(1);
        predicted_yd(i) = predstate_mody(2);
        predicted_yz(i) = predstate_mody(3);
        
        predstate_modxcam = A_upd * predstate_modxcam + B_upd * predicted_xzdcam(i);
        predicted_xcam(i) = predstate_modxcam(1);
        predicted_xdcam(i) = predstate_modxcam(2);
        predicted_xzcam(i) = predstate_modxcam(3);
        
        predstate_modycam = A_upd * predstate_modycam + B_upd * predicted_yzdcam(i);
        predicted_ycam(i) = predstate_modycam(1);
        predicted_ydcam(i) = predstate_modycam(2);
        predicted_yzcam(i) = predstate_modycam(3);
    end
    
    %Setting new states
    modx = predicted_x(1);
    modxd = predicted_xd(1);
    modxz = predicted_xz(1);
    
    mody = predicted_y(1);
    modyd = predicted_yd(1);
    modyz = predicted_yz(1);
    
    modxcam = predicted_xcam(1);
    modxdcam = predicted_xdcam(1);
    modxzcam = predicted_xzcam(1);
    
    modycam = predicted_ycam(1);
    modydcam = predicted_ydcam(1);
    modyzcam = predicted_yzcam(1);
    %% Real robot experiment
    %initialize states
    predicted_x = zeros(1,C); predicted_xcam = zeros(1,C);
    predicted_xd = zeros(1,C); predicted_xdcam = zeros(1,C);
    predicted_xz = zeros(1,C); predicted_xzcam = zeros(1,C);
    predicted_y = zeros(1,C); predicted_ycam = zeros(1,C);
    predicted_yd = zeros(1,C); predicted_ydcam = zeros(1,C);
    predicted_yz = zeros(1,C); predicted_yzcam = zeros(1,C);
    
    % update state and store
    predstate_x = [x; xd; xz];
    predstate_y = [y; yd; yz];
    predstate_xcam = [xcam; xdcam; xzcam];
    predstate_ycam = [ycam; ydcam; yzcam];
    
    %Add noise
    if(Virt_noise)
        predstate_x = predstate_x + [0; 0; virt_torq(1,j)];
        predstate_y = predstate_y + [0; 0; virt_torq(2,j)];
        predstate_xcam = predstate_xcam + [0; 0; virt_torq(1,j)];
        predstate_ycam = predstate_ycam + [0; 0; virt_torq(2,j)];        
    end
    %Real robot
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
    %% Update total states
    %total of both states
    xtot = x + xcam;
    xdtot = xd + xdcam;
    xztot = xz + xzcam;
    
    ytot = y + ycam;
    ydtot = yd + ydcam;
    yztot = yz + yzcam;
    
    %Torques Desired calculation
    txdes = -yzcam*(m*9.81);
    tydes = xzcam*(m*9.81);
    tzdes = ((xtot-xz)*txdes + (ytot-yz)*tydes)/Z0;
    tdes = [txdes, tydes, tzdes]';
    thetadd = J\tdes;
    thetad(:,j+1) = thetad(:,j) + thetadd*mpcTimeStep;
    theta(:,j+1) = theta(:,j) + thetad(:,j)*mpcTimeStep + thetadd*mpcTimeStep^2/2;
    
    % store trajectories
    x_store(end+1) = x;
    xz_store(end+1) = xz;
    y_store(end+1) = y;
    yz_store(end+1) = yz;
    xcam_store(end+1) = xcam;
    ycam_store(end+1) = ycam;
    xzcam_store(end+1) = xzcam;
    yzcam_store(end+1) = yzcam;
    xtot_store(end+1) = xtot;
    ytot_store(end+1) = ytot;
    xztot_store(end+1) = xztot;
    yztot_store(end+1) = yztot;
    
    %store torques
    tx_store(end+1) = txdes;
    ty_store(end+1) = tydes;
    tz_store(end+1) = tzdes;
    
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
        plot(fs_plan(i+fsCounter,1),fs_plan(i+fsCounter,2),'*k','HandleVisibility','off')
        plot(rectangle(1,:) + fs_plan(i+fsCounter,1), ...
            rectangle(2,:) + fs_plan(i+fsCounter,2),'m','Linewidth',2,'HandleVisibility','off')
    end
    plot(xtot_store, ytot_store, 'r', 'Linewidth', 2);
    plot(xz_store, yz_store, 'b', 'Linewidth', 2);
    plot(xztot_store, yztot_store,'k --', 'Linewidth', 1);
    plot(predicted_x+predicted_xcam, predicted_y+predicted_ycam, 'g', 'Linewidth', 2);
    axis equal
    axis([-0.2,1.2,-0.15,0.15])
    grid on
    xlabel('x (m)');
    ylabel('y (m)');
    legend('CoM','ZMP','CMP','Location','northeast')
    drawnow
    
    % update some things
    if j+1 >= fs_timing(fsCounter + 1)
        disp('updating footstep counter')
        fsCounter = fsCounter + 1;
        current_xfs = fs_plan(fsCounter,1);
        current_yfs = fs_plan(fsCounter,2);
        xfs_store(end+1) = current_xfs;
        yfs_store(end+1) = current_yfs;
    end
    %% Change the delta states to the change of model and experiment
    if(VIP)
        xcam = (x+xcam) - modx;
        xdcam = (xd+xdcam) - modxd;
        xzcam = (xz+xzcam) - modxz;
        
        ycam = (y+ycam) - mody;
        ydcam = (yd+ydcam) - modyd;
        yzcam = (yz+yzcam) - modyz;
     
    end
end
%% Plot zmp velocities
figure;
subplot(2,1,1)
plot(xzd_store,'r-');
hold on
plot(xzdcam_store,'b-');
legend('xzd','xzdcam','Location','northeast')
xlabel('time [samples]');
ylabel('Xzmp Velocity [m/s]');
title('ZMP velocity in x');
grid on
subplot(2,1,2)
plot(yzd_store,'r-');
hold on
plot(yzdcam_store,'b-');
legend('yzd','yzdcam','Location','northeast')
xlabel('time [samples]');
ylabel('Yzmp Velocity [m/s]');
title('ZMP velocity in y');
grid
%% Plot rotation angles of vme
thetax = wrapToPi(theta(1,1:j));
thetay = wrapToPi(theta(2,1:j));
thetaz = wrapToPi(theta(3,1:j));
figure;
subplot(1,2,1);
plot(thetax);
xlabel('time (samples)');
ylabel('angle (rad)');
title('Rotation around x-axis');
grid on
subplot(1,2,2);
plot(thetay);
xlabel('time (samples)');
ylabel('angle (rad)');
title('Rotation around y-axis');
grid on
% subplot(1,3,3);
% plot(thetaz);
% xlabel('time (samples)');
% ylabel('angle (rad)');
% title('Rotation around z-axis');
% grid on
%% Plot torques of vme
figure;
subplot(1,3,1);
plot(tx_store);
xlabel('time [samples]');
ylabel('torque [Nm]');
title('torque around x-axis');
grid on
subplot(1,3,2);
plot(ty_store);
xlabel('time [samples]');
ylabel('torque [Nm]');
title('torque around y-axis');
grid on
subplot(1,3,3);
plot(tz_store);
xlabel('time [samples]');
ylabel('torque [Nm]');
title('torque around z-axis');
grid on

%% Plot CoM
virt_torq(:,j:end) = [];
figure;
subplot(1,2,1);
plot(xzcam_store);
hold on
plot(virt_torq(1,:))
legend('true','virtual')
xlabel('time [samples]');
ylabel('CoM [m]');
title('CoM x-axis');
grid on
subplot(1,2,2);
plot(yzcam_store);
hold on
plot(virt_torq(2,:))
legend('true','virtual')
xlabel('time [samples]');
ylabel('CoM [m]');
title('CoM y-axis');
grid on
%%
disp(strcat('xcam from:',num2str(min(xcam_store)),' to:',num2str(max(xcam_store))));
disp(strcat('ycam from:',num2str(min(ycam_store)),' to:',num2str(max(ycam_store))));
disp(strcat('xzcam from:',num2str(min(xzcam_store)),' to:',num2str(max(xzcam_store))));
disp(strcat('yzcam from:',num2str(min(yzcam_store)),' to:',num2str(max(yzcam_store))));