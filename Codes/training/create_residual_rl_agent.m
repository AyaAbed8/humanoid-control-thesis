function [agentObj, observationInfo, actionInfo] = ...
    create_residual_rl_agent()
%CREATE_RESIDUAL_RL_AGENT Create the initial two-action SAC agent.
%   The default networks are intentionally retained for the first
%   environment-validation milestone. Network/hyperparameter tuning occurs
%   only after signal dimensions, zero-action equivalence, reward and reset
%   behavior have been validated.

observationInfo = rlNumericSpec([12 1]);
observationInfo.Name = "residual_rl_observations";
observationInfo.Description = ...
    "Normalized lateral torso, hip, contact and previous-action state";

actionInfo = rlNumericSpec([2 1], ...
    "LowerLimit", -ones(2, 1), ...
    "UpperLimit", ones(2, 1));
actionInfo.Name = "residual_hip_actions";
actionInfo.Description = ...
    "Normalized right and left hip-sagittal residual actions";

agentObj = rlSACAgent(observationInfo, actionInfo);
agentObj.SampleTime = 0.025;
end

