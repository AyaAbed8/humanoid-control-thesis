function [environment, agentObj, observationInfo, actionInfo] = ...
    create_residual_rl_environment(resetProtocol)
%CREATE_RESIDUAL_RL_ENVIRONMENT Create the validated Simulink RL interface.
%   The reset function applies frozen Controller A, enables residual torque,
%   and selects the RL Agent action source. With no input, resets use zero
%   external force. Pass residual_rl_training_protocol() to randomize pushes.

arguments
    resetProtocol struct = residual_rl_training_protocol("validation")
end

thisDirectory = string(project_codes_root());
cd(thisDirectory);
model = "HumanoidModel_ResidualRL";
agentBlock = model + "/Residual RL Agent";

evalin("base", ...
    "humanoid_walker_parameters; inverseDynamics; residual_rl_parameters;");
if isfield(resetProtocol,"residualTorqueLimit_Nm")
    assignin("base","residualRLTorqueLimit_Nm", ...
        resetProtocol.residualTorqueLimit_Nm);
end
[agentObj, observationInfo, actionInfo] = create_residual_rl_agent();
assignin("base", "agentObj", agentObj);
assignin("base", "observationInfo", observationInfo);
assignin("base", "actionInfo", actionInfo);

environment = rlSimulinkEnv(model, agentBlock, ...
    observationInfo, actionInfo, "UseFastRestart", true);
environment.ResetFcn = @(simulationInput) ...
    reset_residual_rl_environment(simulationInput, resetProtocol);
end
