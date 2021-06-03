clear; clc; close all;
VIP = 0;
Angle_const = 0;
Virt_noise = 0;

Qzdotx = 1;
Qzdoty = 1;
Qzdot_camx = 1;
Qzdot_camy = 1;
Qcddot = 0;
theta_max = [pi/18, pi/18, 0]';

NF = 20;
steptime = 50;
fs_plan = varstep(NF);
fs_timing = 0:steptime:steptime*(NF-1);
simDuration = 300;
% general MPC parameters
C = 100; %Control Horizon
P = 200; %Predition Horizon
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
m = 6.93472; %mass of torso
% m = 50.055; %mass of body
%Moment of inertia of torso
Jxx = 0.344345670122806; Jyy = 0.338324801980916; Jzz = 0.048160214086886;
Jyz = 0.003172978374495; Jxz = 0.045048560212699; Jxy = -5.676994253777424e-04;
% Jyz = 0; Jxz = 0; Jxy = 0;
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

% stored trajectories
x_store = x;
xz_store = xz;
y_store = y;
yz_store = yz;
xfs_store = current_xfs;
yfs_store = current_yfs;

modx_store = 0;
modxd_store = 0;
modxz_store = 0;
mody_store = 0;
modyd_store = 0;
modyz_store = 0;


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

Pthddnew = Pzmp*(tril(ones(C),-1)*mpcTimeStep) + (tril(ones(C))*mpcTimeStep^2/2);

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

predicted_xzdcam = zeros(C,1);
predicted_yzdcam = zeros(C,1);
for j = 1:simDuration
    %Generating virtual Ang. momentum
    if(j>50)
        % virt_torq(1,j) = -0.0005*sin(j*0.01*4*pi-pi/2);
        virt_torq(1,j) = -0.0005*sin(j*0.01*4*pi-pi);
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

    %% LIP model
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

    H_pcdd = (Qcddot*eye(C))*(Pcdd.'*Pcdd);
    f_xcdd1 = 2*eta^2 *((Kcdd*Trans_mat)*[x; xd; xz] - pzmp*xz);
    f_ycdd1 = 2*eta^2 *((Kcdd*Trans_mat)*[y; yd; yz] - pzmp*yz);
    
    f_xcdd1b = eta^2 *((Kcdd*Trans_mat)*[xcam; xdcam; xzcam] - pzmp*xzcam);
    f_ycdd1b = eta^2 *((Kcdd*Trans_mat)*[ycam; ydcam; yzcam] - pzmp*yzcam);
    
    f_xcdd = (Qcddot*(f_xcdd1).'*Pcdd).';
    f_ycdd = (Qcddot*(f_ycdd1).'*Pcdd).';
    
    f_xcddb = (2*Qcddot*(f_xcdd1b).'*Pcdd).';
    f_ycddb = (2*Qcddot*(f_ycdd1b).'*Pcdd).';
    
    H = blkdiag(Qzdotx * eye(C,C), Qzdoty * eye(C,C));
    f = zeros(2*C,1);
    
    %Applying the cdd effect on the H-matrix
    H(1:C, 1:C) = H(1:C,1:C) + H_pcdd;
    H(C+1:2*C, C+1:2*C) = H(C+1:2*C, C+1:2*C) + H_pcdd;
    
    %Applying the cdd effect on the f-vector
    f(1:C) = f(1:C) + f_xcdd;
    f(C+1:2*C) = f(C+1:2*C) + f_ycdd;
    
    %adding term with previous zmp velocity (added to the next MPC so 0 here)
    if(VIP)
        f(1:C) = f(1:C) + Qcddot*0*(predicted_xzdcam'* H(1:C, 1:C))';
        f(C+1:2*C) = f(C+1:2*C) + Qcddot*0*(predicted_yzdcam'* H(C+1:2*C, C+1:2*C))';
    end 
    
    %adding term with previous f multiplied by current velocity
    if(VIP)
        f(1:C) = f(1:C) + f_xcddb;
        f(C+1:2*C) = f(C+1:2*C) + f_ycddb; 
    end 

    % solve QP
    
    options = optimset('Display','off');
    [decisionVariables,fval,exitflag,output,~] = ...
        quadprog(H, f, A_zmpconstr, ...
        b_zmpconstr, A_stabconstr, b_stabconstr, ...
        [],[],[],options);
    
    predicted_xzd = decisionVariables(1:C);
    predicted_yzd = decisionVariables(C+1:C+C);
    
    % Store ZMP velocities
    xzd_store(end+1) = predicted_xzd(1);
    yzd_store(end+1) = predicted_yzd(1);
    %% Model Test
    %Initialize new state
    predicted_x = zeros(1,C);
    predicted_xd = zeros(1,C);
    predicted_xz = zeros(1,C);
    predicted_y = zeros(1,C);
    predicted_yd = zeros(1,C);
    predicted_yz = zeros(1,C);
    
    % store state for update
    predstate_modx = [x; xd; xz];
    predstate_mody = [y; yd; yz];
    predstate_modxcam = [xcam; xdcam; xzcam];
    predstate_modycam = [ycam; ydcam; yzcam];
    
    for i = 1:C
        predstate_modx = A_upd * predstate_modx + B_upd * predicted_xzd(i);
        predicted_x(i) = predstate_modx(1);
        predicted_xd(i) = predstate_modx(2);
        predicted_xz(i) = predstate_modx(3);
        
        predstate_mody = A_upd * predstate_mody + B_upd * predicted_yzd(i);
        predicted_y(i) = predstate_mody(1);
        predicted_yd(i) = predstate_mody(2);
        predicted_yz(i) = predstate_mody(3);
    end
    
    %Setting new states
    modx = predicted_x(1);
    modxd = predicted_xd(1);
    modxz = predicted_xz(1);
    
    mody = predicted_y(1);
    modyd = predicted_yd(1);
    modyz = predicted_yz(1);
    
    %% Real robot experiment
    % store state for update
    predstate_x = [x; xd; xz];
    predstate_y = [y; yd; yz];
    
    %Initialize new state
    predicted_x = zeros(1,C); predicted_xcam = zeros(1,C);
    predicted_xd = zeros(1,C); predicted_xdcam = zeros(1,C);
    predicted_xz = zeros(1,C); predicted_xzcam = zeros(1,C);
    predicted_y = zeros(1,C); predicted_ycam = zeros(1,C);
    predicted_yd = zeros(1,C); predicted_ydcam = zeros(1,C);
    predicted_yz = zeros(1,C); predicted_yzcam = zeros(1,C);
      
    %Add noise
    if(Virt_noise)
        predstate_x = predstate_x + [0; 0; virt_torq(1,j)];
        predstate_y = predstate_y + [0; 0; virt_torq(2,j)];
    end
    
    for i = 1:C
        predstate_x = A_upd * predstate_x + B_upd * predicted_xzd(i);
        predicted_x(i) = predstate_x(1);
        predicted_xd(i) = predstate_x(2);
        predicted_xz(i) = predstate_x(3);
        
        predstate_y = A_upd * predstate_y + B_upd * predicted_yzd(i);
        predicted_y(i) = predstate_y(1);
        predicted_yd(i) = predstate_y(2);
        predicted_yz(i) = predstate_y(3);        
    end
    
    %Setting new states
    x = predicted_x(1);
    xd = predicted_xd(1);
    xz = predicted_xz(1);
    
    y = predicted_y(1);
    yd = predicted_yd(1);
    yz = predicted_yz(1);
    %% ZMP CAM
    if(VIP)
    %Update CAM
    xcam = x + xcam - modx;
    xdcam = xd + xdcam - modxd;
    xzcam = xz + xzcam - modxz;

    ycam = y + ycam - mody;
    ydcam = yd + ydcam - modyd;
    yzcam = yz + yzcam - modyz;
    
    A_zmpconstr_cam = [Pzmp, zeros(C, C); ...
        zeros(C, C), Pzmp; ...
        -Pzmp, zeros(C, C); ...
        zeros(C, C), -Pzmp];
    
    b_zmpconstr_cam = [pzmp*(xzcam_max-xzcam); ...
        pzmp*(yzcam_max-yzcam); ...
        -pzmp*(-xzcam_max-xzcam); ...
        -pzmp*(-yzcam_max-yzcam)];
    
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
    Tzdes = ((xtot-xz)*Txdes+(ytot-yz)*Tydes)/-Z0;
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
        
    H_pcdd = (Qcddot*eye(C))*(Pcdd.'*Pcdd);
    f_xcdd1 = 2*eta^2 *((Kcdd*Trans_mat)*[xcam; xdcam; xzcam] - pzmp*xzcam);
    f_ycdd1 = 2*eta^2 *((Kcdd*Trans_mat)*[ycam; ydcam; yzcam] - pzmp*yzcam);
    
    f_xcdd1a = eta^2 *((Kcdd*Trans_mat)*[x; xd; xz] - pzmp*xz);
    f_ycdd1a = eta^2 *((Kcdd*Trans_mat)*[y; yd; yz] - pzmp*yz);
    
    f_xcdd = (Qcddot*(f_xcdd1).'*Pcdd).';
    f_ycdd = (Qcddot*(f_ycdd1).'*Pcdd).';
    
    f_xcdda = (2*Qcddot*(f_xcdd1a).'*Pcdd).';
    f_ycdda = (2*Qcddot*(f_ycdd1a).'*Pcdd).';

    H = blkdiag(Qzdot_camx*eye(C,C), Qzdot_camy*eye(C,C));
    f = zeros(2*C,1);
    
    %Applying the cdd effect on the H-matrix
    H(1:C, 1:C) = H(1:C,1:C) + H_pcdd;
    H(C+1:2*C, C+1:2*C) = H(C+1:2*C, C+1:2*C) + H_pcdd;
    
    %Applying the cdd effect on the f-vector
    f(1:C) = f(1:C) + f_xcdd;
    f(C+1:2*C) = f(C+1:2*C) + f_ycdd;
    
    %adding term with previous zmp velocity
    if(VIP)
        f(1:C) = f(1:C) + Qcddot*2*(predicted_xzd'* H(1:C, 1:C))';
        f(C+1:2*C) = f(C+1:2*C) + Qcddot*2*(predicted_yzd'* H(C+1:2*C, C+1:2*C))';
    end 
    
    %adding term with previous f multiplied by current velocity
    if(VIP)
        f(1:C) = f(1:C) + f_xcdda;
        f(C+1:2*C) = f(C+1:2*C) + f_ycdda; 
    end 
    
    % solve QP
    options = optimset('Display','off');
        if(Angle_const)
    [decisionVariables,fval,exitflag,output,~] = ...
        quadprog(H, f, [A_zmpconstr_cam; A_angleconstr], ...
        [b_zmpconstr_cam; b_angleconstr], A_stabconstr_cam, b_stabconstr_cam, ...
        [],[],[],options);
    else 
    [decisionVariables,fval,exitflag,output,~] = ...
        quadprog(H, f, A_zmpconstr_cam, ...
        b_zmpconstr_cam, A_stabconstr_cam, b_stabconstr_cam, ...
        [],[],[],options);
        end
        
    predicted_xzdcam = decisionVariables(1:C);
    predicted_yzdcam = decisionVariables(C+1:C+C);
    
    % Store ZMP cam and total ZMP velocities
    xzdcam_store(end+1) = predicted_xzdcam(1);
    yzdcam_store(end+1) = predicted_yzdcam(1);
    xzdtot_store(end+1) = xzd_store(end) + xzdcam_store(end);
    yzdtot_store(end+1) = yzd_store(end) + yzdcam_store(end);

    %Store state for update
    predstate_xcam = [xcam; xdcam; xzcam];
    predstate_ycam = [ycam; ydcam; yzcam];
    for i = 1:C
        predstate_xcam = A_upd * predstate_xcam + B_upd * predicted_xzdcam(i);
        predicted_xcam(i) = predstate_xcam(1);
        predicted_xdcam(i) = predstate_xcam(2);
        predicted_xzcam(i) = predstate_xcam(3);
        
        predstate_ycam = A_upd * predstate_ycam + B_upd * predicted_yzdcam(i);
        predicted_ycam(i) = predstate_ycam(1);
        predicted_ydcam(i) = predstate_ycam(2);
        predicted_yzcam(i) = predstate_ycam(3);   
    end
    
    xcam = predicted_xcam(1);
    xdcam = predicted_xdcam(1);
    xzcam = predicted_xzcam(1);
    
    ycam = predicted_ycam(1);
    ydcam = predicted_ydcam(1);
    yzcam = predicted_yzcam(1);
    end
    %total of both states (after both MPCs)
    xtot = x + xcam;
    xdtot = xd + xdcam;
    xztot = xz + xzcam;

    ytot = y + ycam;
    ydtot = yd + ydcam;
    yztot = yz + yzcam;
        
    %Torques Desired calculation
    txdes = -yzcam*(m*9.81);
    tydes = xzcam*(m*9.81);
    tzdes = ((xtot-xz)*txdes + (ytot-yz)*tydes)/-Z0;
    tdes = [txdes, tydes, tzdes]';
    thetadd = J\tdes;
    thetad(:,j+1) = thetad(:,j) + thetadd*mpcTimeStep;
    theta(:,j+1) = theta(:,j) + thetad(:,j)*mpcTimeStep + thetadd*(mpcTimeStep^2/2);
    
    % store trajectories
    x_store(end+1) = x;
    xz_store(end+1) = xz;
    y_store(end+1) = y;
    yz_store(end+1) = yz;

    modx_store(end+1) = modx;
    modxd_store(end+1) = modxd;
    modxz_store(end+1) = modxz;
    
    mody_store(end+1) = mody;
    modyd_store(end+1) = modyd;
    modyz_store(end+1) = modyz;
    

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
end
figure;
plot(xtot_store, ytot_store, 'r', 'Linewidth', 2);
hold on;
plot(xz_store, yz_store, 'b', 'Linewidth', 2);
plot(xztot_store, yztot_store,'k --', 'Linewidth', 1);
grid on
xlabel('x (m)');
ylabel('y (m)');
legend('CoM','ZMP','CMP','Location','northeast')
save('..\Dart\txts\xtot_store.txt', 'xtot_store','-ascii');
save('..\Dart\txts\ytot_store.txt', 'ytot_store','-ascii');
save('..\Dart\txts\x_store.txt', 'x_store','-ascii');
save('..\Dart\txts\y_store.txt', 'y_store','-ascii');
save('..\Dart\txts\xz_store.txt', 'xz_store','-ascii');
save('..\Dart\txts\yz_store.txt', 'yz_store','-ascii');
save('..\Dart\txts\xztot_store.txt', 'xztot_store','-ascii');
save('..\Dart\txts\yztot_store.txt', 'yztot_store','-ascii');


save('..\Dart\txts\modx_store.txt', 'modx_store','-ascii');
save('..\Dart\txts\modxz_store.txt', 'modxz_store','-ascii');
save('..\Dart\txts\mody_store.txt', 'mody_store','-ascii');
save('..\Dart\txts\modyz_store.txt', 'modyz_store','-ascii');

virt_torqx = virt_torq(1,:);
virt_torqy = virt_torq(2,:);

save('..\Dart\txts\virt_torqx_matlab.txt', 'virt_torqx','-ascii');
save('..\Dart\txts\virt_torqy_matlab.txt', 'virt_torqy','-ascii');

save('..\Dart\txts\xzd_store.txt', 'xzd_store','-ascii');
save('..\Dart\txts\yzd_store.txt', 'yzd_store','-ascii');


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
grid on
%% Plot rotation angles of vme
thetax = (theta(1,1:j));
thetay = (theta(2,1:j));
thetaz = (theta(3,1:j));
save('..\Dart\txts\thetax.txt', 'thetax','-ascii');
save('..\Dart\txts\thetay.txt', 'thetay','-ascii');
save('..\Dart\txts\thetaz.txt', 'thetaz','-ascii');

thetax = wrapToPi(thetax);
thetay = wrapToPi(thetay);
thetaz = wrapToPi(thetaz);
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
% virt_torq(:,j:end) = [];
% figure;
% subplot(1,2,1);
% plot(xzcam_store);
% hold on
% plot(virt_torq(1,:))
% legend('true','virtual')
% xlabel('time [samples]');
% ylabel('CoM [m]');
% title('CoM x-axis');
% grid on
% subplot(1,2,2);
% plot(yzcam_store);
% hold on
% plot(virt_torq(2,:))
% legend('true','virtual')
% xlabel('time [samples]');
% ylabel('CoM [m]');
% title('CoM y-axis');
% grid on
%%
% disp(strcat('xcam from:',num2str(min(xcam_store)),' to:',num2str(max(xcam_store))));
% disp(strcat('ycam from:',num2str(min(ycam_store)),' to:',num2str(max(ycam_store))));
% disp(strcat('xzcam from:',num2str(min(xzcam_store)),' to:',num2str(max(xzcam_store))));
% disp(strcat('yzcam from:',num2str(min(yzcam_store)),' to:',num2str(max(yzcam_store))));     