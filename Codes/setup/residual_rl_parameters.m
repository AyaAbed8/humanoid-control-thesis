%% Residual-RL interface parameters
% These variables configure only HumanoidModel_ResidualRL. The frozen
% Controller A definition and the baseline model are not modified.

residualRLEnabled = 0;
residualRLUseAgent = 0;
residualRLManualAction = zeros(2, 1);
residualRLTorqueLimit_Nm = 15;
residualRLSampleTime_s = params.simulation.Ts;

% Leg-torque order:
%   1 right hip frontal, 2 right knee, 3 right ankle pitch,
%   4 right hip sagittal, 5 right hip transverse, 6 right ankle roll,
%   7 left hip frontal, 8 left knee, 9 left ankle pitch,
%  10 left hip sagittal, 11 left hip transverse, 12 left ankle roll.
residualRLChannelMap = zeros(12, 2);
residualRLChannelMap(4, 1) = 1;
residualRLChannelMap(10, 2) = 1;

% Observation vector:
% [x; vx; qy; wy; right hip angle/rate; left hip angle/rate;
%  right/left normalized foot-load errors; previous right/left actions].
residualRLNominalFootLoad_N = 42.495896 * 9.80665 / 2;
residualRLRightFootLoadMap = [ones(1, 6), zeros(1, 6)];
residualRLLeftFootLoadMap = [zeros(1, 6), ones(1, 6)];
residualRLObservationInverseScales = 1 ./ [ ...
    0.10; 0.50; deg2rad(10); 1.0; ...
    deg2rad(30); 2.0; deg2rad(30); 2.0; ...
    1.0; 1.0; 1.0; 1.0];

% Initial reward calibration. These remain named parameters so trajectory
% ranges can be used to revise them without editing the block diagram.
residualRLRewardStateWeights = [ ...
    2.0, 0.2, 2.0, 0.2, ...
    0.10, 0.05, 0.10, 0.05, ...
    0.25, 0.25, 0.02, 0.02];
residualRLActionPenaltyWeight = 0.02;
residualRLActionRatePenaltyWeight = 0.05;
residualRLAliveReward = 1;
residualRLTerminalPenalty = 100;

