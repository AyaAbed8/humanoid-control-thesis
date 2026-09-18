%% Inverse Dynamics on New RBT (with B/F reversed)
% btw inverse dynamics is same as gravity torque when the configuration
% is constant, aka q_dot = q_ddot = 0

%% settings
builderFunction = @build_manual_humanoid_tree_BFreversed;
gravityVector = [0 0 -9.80665];
standingJointAnglesDeg = {
    'right_shoulder_frontal',    0
    'right_shoulder_sagittal',   0
    'left_shoulder_frontal',     0
    'left_shoulder_sagittal',    0

    'right_hip_transverse',      0
    'right_hip_frontal',       -30
    'right_hip_sagittal',        0
    'right_knee',               30
    'right_ankle_roll',          0
    'right_ankle_pitch',       -10

    'left_hip_transverse',       0
    'left_hip_frontal',        -30
    'left_hip_sagittal',         0
    'left_knee',                30
    'left_ankle_roll',           0
    'left_ankle_pitch',        -10
};

%% Build robot
robotFull = builderFunction();
robotFull.Gravity = gravityVector;

%% Inverse dynamics 
% tauFull = inverseDynamics(robotFull, ...
%     [zeros(1,10),0,-30,0,30,0,-10,0,-30,0,30,0,-10], zeros(1,22), zeros(1,22))
tau_G = gravityTorque(robotFull, ...
    [zeros(1,10),0,-20,0,20,0,-10,0,-20,0,20,0,-10]);