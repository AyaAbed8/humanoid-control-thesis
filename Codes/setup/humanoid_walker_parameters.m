%% Humanoid Walker With Genetic Algorithm Initialization
% Copyright 2019 The MathWorks, Inc.

%% Joint rotation limits
% These limits are similar to human joint rotation limits. 

% Legs= joints
params.jointLimits.hipFrontalUpperLimit  =  30;  % (Deg)
params.jointLimits.hipFrontalLowerLimit  = -90;  % (Deg)
params.jointLimits.hipSagittalUpperLimit =  30;  % (Deg)
params.jointLimits.hipSagittalLowerLimit = -30;  % (Deg)
params.jointLimits.kneeUpperLimit        =  90;  % (Deg)
params.jointLimits.kneeLowerLimit        =  5;   % (Deg)
params.jointLimits.ankleUpperLimit       =  20;  % (Deg)
params.jointLimits.ankleLowerLimit       = -20;  % (Deg)
% Arm joints
params.jointLimits.shoulderFrontalUpperLimit        =  110; % (Deg)
params.jointLimits.shoulderFrontalLowerLimit        = -30;  % (Deg)
params.jointLimits.shoulderSagittalUpperLimit       =  90;  % (Deg)
params.jointLimits.shoulderSagittalLowerLimit       = -30;  % (Deg)

%% Material properties

params.materialProperties.lowerBodyDensity = 990;  % (kg/m^3)
params.materialProperties.upperBodyDensity = 1900; % (kg/m^3)

%% World damping
% We add damping to the vertical (z) and rotational (Rx, Ry, Rz) components
% of the bushing joint. This dissipates energy from the system and facilitates 
% quicker learning. Increasing this improves stability and learning, but is
% less realistic. 

params.simulation.worldDamping = 3; % (Ns/m) % GA
% params.simulation.worldDamping = 6; % (Ns/m) % RL

%% Simulation parameters

params.simulation.initialHeight=1.54; % (m) Initial height of robot
params.simulation.Ts = 0.025;         % (s) Control and sensing discretization time
params.simulation.Tf = 30;            % (s) Max simulation time

%% Spatial contact force block parameters. 
% The value of these parameters affects simulation and learning speed. 
% Higher damping generally improves learning but slows down simulation. 
% High static and dynamic friction values tend to improve learning. 

params.contact.stiffness        = 1e5;  % (N/m)
params.contact.damping          = 1e5;  % (Ns/m) 
params.contact.transitionWidth  = 1e-3; % (m)
params.contact.staticFriction   = 1;    % () 
params.contact.dynamicFriction  = 0.9;  % ()
params.contact.criticalVelocity = 1e-3; % (m/s) 
params.contact.contactRadius    = 0.01;    % (m)

%% Controller parameters

params.controller.hipFrontalStiffness  = 80; % (N*m/rad)
params.controller.hipFrontalDamping    = 1;  % (N*m*s/rad)
params.controller.hipSagittalStiffness = 80; % (N*m/rad)
params.controller.hipSagittalDamping   = 1;  % (N*m*s/rad)
params.controller.kneeStiffness        = 80; % (N*m*/rad)
params.controller.kneeDamping          = 1;  % (N*m*s/rad)
params.controller.ankleStiffness       = 80; % (N*m/rad)
params.controller.ankleDamping         = 1;  % (N*m*s/rad)

params.torqueLimits.hipFrontal  = 100; % (N*m)
params.torqueLimits.hipSagittal = 100; % (N*m)
params.torqueLimits.knee        = 100; % (N*m)
params.torqueLimits.ankle       = 100; % (N*m)
params.torqueLimits.legActuator = 100; % (N*m), final applied limit

%% Stopping criteria
% Define the conditions under which the simulation is terminated early. 
% One or more of three conditions need to be satisfied for termination. 
% If the humanoid torso drops vertically, travels laterally or rotates in
% any axis more than a set of predefined values. Or if the humanoid stops
% moving.

params.stoppingCriteria.heightChange     = 0.5; % (m)
params.stoppingCriteria.lateral          = 1;   % (m)
params.stoppingCriteria.angle            = 30;  % (deg)
params.stoppingCriteria.timeoutTime      = 2;   % (s)
params.stoppingCriteria.timeoutDistance  = 1;   % (m)

% Reward scaling values

params.reward.forwardRewardWeight   = 1;    % Forward velocity scale, w_1
params.reward.timestepRewardWeight  = 1;    % Not falling scale, w_2
params.reward.powerPenaltyWeight    = 5e-4; % Power scale, w_3
params.reward.verticalPenaltyWeight = 25;   % Vertical displacement scale, w_4
params.reward.lateralPenaltyWeight  = 2.5;  % Lateral displacement scale, w_5

%% Display parameters 

params.display.tileColour    = [0.956, 0.941, 0.941];
params.display.floorColour   = [0.756, 0.741, 0.741];
params.display.tileThickness = 0.005; 
params.display.planeWidth        = 2;   % (m)
params.display.planeLength       = 2;  % (m)
params.display.planeHeight       = 1;   % (m)

%% RT3: Balance Control
r = 1;
% for rotation about x (Rx = -10deg)
 % kp_ankle_qx = -0.25/r ; kd_ankle_wx = -0.05/r ; % NEGATIVE FB
 kp_ankle_qx = -0.7 ;  kd_ankle_wx = -0.1;
 % kp_hip_qx = 0.1/r ; kd_hip_wx = 0.05/r ; % POSITIVE FB
 kp_hip_qx = 1; kd_hip_wx = 0.1;
Rx_ref = deg2rad(-5); %-10;

% for torso forward position (y = 0.25m)
kp_ankle_y = 6/r ; kd_ankle_vy = 1/r ; % POSITIVE FB
 % kp_hip_y = -6/r; kd_hip_vy = -1/r; % NEGATIVE FB
kp_hip_y = -4; kd_hip_vy = -1.7;
y_ref = 0.25;

% for torso height (dz = z-Pz; Pz=-1.54m)
kp_hip_dz = -4/r ; kd_hip_vz = -0.1/r ; % NEGATIVE FB

% for rotation about y 
kp_hip_sag_qy = 10; kd_hip_sag_wy = 1;

% for torso lateral position x
kp_hip_sag_x = -10; kd_hip_sag_vx = -1;
x_ref = 0;

% knee standing pos
K = (params.jointLimits.kneeUpperLimit-params.jointLimits.kneeLowerLimit)/2;
b = params.jointLimits.kneeLowerLimit + (params.jointLimits.kneeUpperLimit-params.jointLimits.kneeLowerLimit)/2;
u_knee = (20-b)./K;
% hip frontal standing pos
K = (params.jointLimits.hipFrontalUpperLimit-params.jointLimits.hipFrontalLowerLimit)/2;
b = params.jointLimits.hipFrontalLowerLimit + (params.jointLimits.hipFrontalUpperLimit-params.jointLimits.hipFrontalLowerLimit)/2;
u_hip_frontal = ((-20)-b)./K;
% hip sagittal standing pos (0 deg under symmetric -30/+30 mapping)
u_hip_sagittal = 0;
% ankle pitch standing pos
K = (params.jointLimits.ankleUpperLimit-params.jointLimits.ankleLowerLimit)/2;
b = params.jointLimits.ankleLowerLimit + (params.jointLimits.ankleUpperLimit-params.jointLimits.ankleLowerLimit)/2;
u_ankle_pitch = ((-10)-b)./K;

% shoulders frontal standing pos
K = (params.jointLimits.shoulderFrontalUpperLimit - params.jointLimits.shoulderFrontalLowerLimit)/2;
b = params.jointLimits.shoulderFrontalLowerLimit + K;
u_shoulder_frontal = (0 + b)./K;

%% Robustness force-pulse interface
% World-frame force applied to the torso. The zero default preserves the
% nominal plant. Simscape x is lateral and y is sagittal in this project.
robustnessForceStart_s = 2;            % (s)
robustnessForceDuration_s = 0.1;       % (s)
robustnessForceVector_N = zeros(3,1);   % [Fx; Fy; Fz] (N)

%% MT2: Realistic actuation and sensing
% 
% % Actuator lag: 1st order TF
% p = 0.5;
% 
% % DC Motor modelling
% Kv = 1; % voltage/torque gain
% GearRatio = 5000;

