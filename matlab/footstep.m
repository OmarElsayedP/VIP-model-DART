% footstep plan and footstep timings
% Choose 'Normal' for standard footsteps, 'Weird' for varying lengths of
% the footsteps and 'FwdRev' for forward then reverse footsteps
function fs_plan = footstep(varagin)
switch varagin
    case 'Weird'
        fs_plan = [0.0, 0.0; ... %WS
            0.0, -0.075; ...
            0.05, 0.075; ...
            0.1, -0.075; ...
            0.15*2, 0.075; ...
            0.2*2, -0.075; ...
            0.25*3, 0.075; ...
            0.3*3, -0.075; ...
            0.35*3, 0.075; ...
            0.4*3, -0.075; ...
            0.45*3, 0.075; ...
            0.5*3, -0.075];
    case 'Normal'
        fs_plan = [0.0, 0.0; ...
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
    case 'FwdRev'
        fs_plan = [0.0, 0.0; ...
            0.0, -0.15; ...
            0.1, 0.15; ...
            0.2, -0.15; ...
            0.25, 0.15; ...
            0.25, -0.15; ...
            0.2, 0.15; ...
            0.15, -0.15; ...
            0.0, 0.15; ...
            -0.15, -0.15; ...
            -0.2, 0.15; ...
            0.25, -0.15];
end
end
%returns normal footsteps automatically generated