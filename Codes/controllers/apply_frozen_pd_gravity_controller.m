function [simIn, controller] = apply_frozen_pd_gravity_controller( ...
    simIn, controllerID, model)
%APPLY_FROZEN_PD_GRAVITY_CONTROLLER Apply frozen controller A or D.
%   [SIMIN, CONTROLLER] = APPLY_FROZEN_PD_GRAVITY_CONTROLLER( ...
%   SIMIN, ID, MODEL) applies the exact frozen controller variables and
%   top-level gain-block settings to a Simulink.SimulationInput.
%
%   ID must be "A" or "D". This helper does not set model initial
%   conditions, disturbances, uncertainties, stop time, or logging.

arguments
    simIn (1,1) Simulink.SimulationInput
    controllerID (1,1) string
    model (1,1) string = "HumanoidModel_BaselineBalanceControl"
end

frozen = frozen_pd_gravity_controllers();
controllerID = upper(controllerID);
assert(any(controllerID == ["A", "D"]), ...
    "FrozenControllers:UnknownController", ...
    "controllerID must be ""A"" or ""D"".");
controller = frozen.(controllerID);

simIn = simIn.setVariable("alpha", controller.alpha);
simIn = simIn.setVariable( ...
    "u_hip_frontal", controller.nominalCommands.hip);
simIn = simIn.setVariable("u_knee", controller.nominalCommands.knee);
simIn = simIn.setVariable( ...
    "u_ankle_pitch", controller.nominalCommands.ankle);
simIn = simIn.setVariable( ...
    "u_hip_sagittal", controller.nominalCommands.hipSagittal);

for index = 1:numel(frozen.shared.outerPGainNames)
    simIn = simIn.setVariable( ...
        frozen.shared.outerPGainNames(index), ...
        frozen.shared.outerPGainValues(index));
end
for index = 1:numel(frozen.shared.outerDGainNames)
    simIn = simIn.setVariable( ...
        frozen.shared.outerDGainNames(index), ...
        frozen.shared.outerDGainValues(index));
end

simIn = simIn.setVariable( ...
    "Rx_ref", frozen.shared.torsoRollReference_rad);
simIn = simIn.setVariable( ...
    "y_ref", frozen.shared.torsoForwardReference_m);
simIn = simIn.setVariable( ...
    "x_ref", frozen.shared.torsoLateralReference_m);
simIn = simIn.setBlockParameter(model + "/Gain", "Gain", "1");
simIn = simIn.setBlockParameter(model + "/Gain2", "Gain", "alpha");
end
