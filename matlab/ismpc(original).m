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
C = 100;
P = 200;
F = 3;
mpcTimeStep = 0.01;
eta = sqrt(9.8/0.75);
dsSamples = 20;

% ZMP constraint parameters
wx = 0.08;
wy = 0.08;
initial_wx = 0.08;
initial_wy = 0.08/2 + 0.2;

% kinematic constraint parameters
ell = 0.2;
wkx = 0.5;
wky = 0.08;

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

% state update matrices
ch = cosh(eta*mpcTimeStep);
sh = sinh(eta*mpcTimeStep);
A_upd = [ch, sh/eta, 1-ch; eta*sh, ch, -eta*sh; 0, 0, 1];
B_upd = [mpcTimeStep-sh/eta; 1-ch; mpcTimeStep];

% stored strajectories
x_store = x;
xz_store = xz;
y_store = y;
yz_store = yz;
xfs_store = current_xfs;
yfs_store = current_yfs;

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
    
    A_zmpconstr = [Pzmp, -mapping(:,2:end), zeros(C, C+F); ...
        zeros(C, C+F), Pzmp, -mapping(:,2:end); ...
        -Pzmp, mapping(:,2:end), zeros(C, C+F); ...
        zeros(C, C+F), -Pzmp, mapping(:,2:end)];
    
    b_zmpconstr = [pzmp * (- xz + wx/2) + mapping(:,1) * current_xfs; ...
        pzmp * (- yz + wy/2) + mapping(:,1) * current_yfs; ...
        - pzmp * (- xz - wx/2) - mapping(:,1) * current_xfs; ...
        - pzmp * (- yz - wy/2) - mapping(:,1) * current_yfs];
    
    if fsCounter == 1
        b_zmpconstr = b_zmpconstr + [ mapping(:,1) * (- wx/2 + initial_wx/2); ...
            mapping(:,1) * (- wy/2 + initial_wy/2); ...
            mapping(:,1) * (- wx/2 + initial_wx/2); ...
            mapping(:,1) * (- wy/2 + initial_wy/2)];
    end
    
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
    
    A_stabconstr = [(1/eta)*(1-lambda)/(1-lambda^C) * b_exp' - mpcTimeStep * ones(1,C)*exp(- eta * mpcTimeStep * C), ...
        zeros(1,F+C+F); ...
        zeros(1,C+F), ...
        (1/eta)*(1-lambda)/(1-lambda^C) * b_exp' - mpcTimeStep * ones(1,C)*exp(- eta * mpcTimeStep * C), ...
        zeros(1,F)];
    
    b_stabconstr = [x + xd/eta - xz - anticipative_x; ...
        y + yd/eta - yz - anticipative_y];
    
    % cost function
    
    Qzdot = 1;
    Qfootsteps = 100000;
    H_half = blkdiag(Qzdot * eye(C,C), Qfootsteps * eye(F,F));
    H = blkdiag(H_half, H_half);
    
    f = [zeros(C,1); - Qfootsteps * fs_plan(fsCounter+1:fsCounter+F,1); ...
        zeros(C,1); - Qfootsteps * fs_plan(fsCounter+1:fsCounter+F,2)];
    
    % solve QP
    
    options = optimset('Display','off');
    decisionVariables = quadprog(H, f, [A_zmpconstr;A_kinconstr], [b_zmpconstr;b_kinconstr], A_stabconstr, b_stabconstr,...
        [],[],[],options);
    predicted_xzd = decisionVariables(1:C);
    predicted_xfs = decisionVariables(C+1:C+F);
    predicted_yzd = decisionVariables(C+F+1:C+F+C);
    predicted_yfs = decisionVariables(C+F+C+1:C+F+C+F);
    
    % update state and store
    
    predstate_x = [x; xd; xz];
    predstate_y = [y; yd; yz];
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
    
    x = predicted_x(1);
    xd = predicted_xd(1);
    xz = predicted_xz(1);
    
    y = predicted_y(1);
    yd = predicted_yd(1);
    yz = predicted_yz(1);
    
    % store trajectories
    x_store(end+1) = x;
    xz_store(end+1) = xz;
    
    y_store(end+1) = y;
    yz_store(end+1) = yz;
    
    % plot
    
    clf

    hold on
    rectangle_initial = [initial_wx/2,initial_wx/2,-initial_wx/2,-initial_wx/2,initial_wx/2; ...
        initial_wy/2,-initial_wy/2,-initial_wy/2,initial_wy/2,initial_wy/2];
    rectangle = [wx/2,wx/2,-wx/2,-wx/2,wx/2; wy/2,-wy/2,-wy/2,wy/2,wy/2];
    
    % plot footsteps (first double support manually added)
    plot(rectangle_initial(1,:), rectangle_initial(2,:),'m','Linewidth',2)
    for i = 2:fsCounter
        plot(rectangle(1,:) + xfs_store(i), rectangle(2,:) + yfs_store(i),'m','Linewidth',2)
    end
    for i = 1:F
        plot(predicted_xfs(i),predicted_yfs(i),'*k')
        plot(rectangle(1,:) + predicted_xfs(i), rectangle(2,:) + predicted_yfs(i),'m','Linewidth',2)
    end
    
    plot(x_store,y_store,'r','Linewidth',2)
    plot(xz_store,yz_store,'b','Linewidth',2)
    plot(predicted_x,predicted_y,'g','Linewidth',2)
    
    axis equal
    axis([-0.2,0.7,-0.15,0.15])
    
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

