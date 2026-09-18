function result = run_specific_uncertainty_case( ...
    controllerID, trunkChange_percent, axisName, signedForce_N, ...
    stopTime_s)
%RUN_SPECIFIC_UNCERTAINTY_CASE Run and view one selected uncertainty case.
%   RESULT = RUN_SPECIFIC_UNCERTAINTY_CASE( ...
%       CONTROLLER, TRUNK_CHANGE_PERCENT, AXIS, SIGNED_FORCE_N)
%   runs:
%     1. a matched zero-force reference; then
%     2. the requested disturbed case.
%
%   The disturbed run is executed last, so the open Simscape Results
%   Explorer / Mechanics Explorer shows the requested trajectory.
%
%   Examples:
%     % Controller D, +20% trunk mass/inertia, +50 N lateral push
%     result = run_specific_uncertainty_case("D", 20, "lateral", 50);
%
%     % Controller A, +20% trunk mass/inertia, -75 N sagittal push
%     result = run_specific_uncertainty_case("A", 20, "sagittal", -75);
%
%     % Uncertainty-only visual check (no force)
%     result = run_specific_uncertainty_case("D", 20, "lateral", 0);
%
%   Project directions:
%     lateral  -> world x
%     sagittal -> world y
%
%   The percentage scales only the Simscape Trunk density. Because its
%   geometry is fixed and inertia is calculated from geometry, Trunk mass
%   and inertia scale together. The nominal rigidBodyTree used by gravity
%   compensation is not changed.
%
%   The model is modified only in memory. The original density expression
%   and dirty state are restored after simulation; the model is not saved.

arguments
    controllerID (1,1) string
    trunkChange_percent (1,1) double {mustBeFinite} = 20
    axisName (1,1) string = "lateral"
    signedForce_N (1,1) double {mustBeFinite} = 50
    stopTime_s (1,1) double {mustBeFinite, mustBePositive} = 11
end

controllerID = upper(controllerID);
axisName = lower(axisName);
assert(any(controllerID == ["A", "D"]), ...
    "SpecificUncertainty:UnknownController", ...
    "controllerID must be ""A"" or ""D"".");
assert(any(axisName == ["lateral", "sagittal"]), ...
    "SpecificUncertainty:UnknownAxis", ...
    "axisName must be ""lateral"" or ""sagittal"".");
assert(trunkChange_percent > -100, ...
    "SpecificUncertainty:NonpositiveDensity", ...
    "trunkChange_percent must be greater than -100.");

pulseStart_s = 2;
pulseDuration_s = 0.1;
assert(pulseStart_s + pulseDuration_s < stopTime_s, ...
    "SpecificUncertainty:PulseOutsideRun", ...
    "Stop time must exceed 2.1 s.");

model = "HumanoidModel_BaselineBalanceControl";
thisDirectory = string(project_codes_root());
originalDirectory = string(pwd);
directoryCleanup = onCleanup(@() cd(originalDirectory));
cd(thisDirectory);

variablesBeforeSetup = who;
humanoid_walker_parameters;
setupVariables = setdiff(who, variablesBeforeSetup);
robotFull = build_manual_humanoid_tree_BFreversed();
robotFull.Gravity = [0 0 -9.80665];
setupVariables = [setupVariables; {"robotFull"}];

load_system(model);
trunkBlock = model + "/Humanoid Robot/Torso and Head/Trunk";
assert(getSimulinkBlockHandle(trunkBlock) > 0, ...
    "SpecificUncertainty:TrunkBlockMissing", ...
    "Expected Trunk File Solid block not found.");
originalDensity = string(get_param(trunkBlock, "Density"));
assert(originalDensity == "trunkDensity", ...
    "SpecificUncertainty:UnexpectedDensityExpression", ...
    "Expected Trunk Density ""trunkDensity"", got ""%s"".", ...
    originalDensity);
originalDirty = string(get_param(model, "Dirty"));
restoreCleanup = onCleanup(@() restoreModel( ...
    model, trunkBlock, originalDensity, originalDirty));

scale = 1 + trunkChange_percent / 100;
densityExpression = sprintf( ...
    "(%.17g)*(%s)", scale, originalDensity);
set_param(trunkBlock, "Density", densityExpression);

forceVector = zeros(3, 1);
if axisName == "lateral"
    forceVector(1) = signedForce_N;
else
    forceVector(2) = signedForce_N;
end

fprintf("\nSpecific uncertainty case\n");
fprintf("Controller: %s\n", controllerID);
fprintf("Trunk mass/inertia change: %+.6g%% (scale %.6g)\n", ...
    trunkChange_percent, scale);
fprintf("Axis: %s\n", axisName);
fprintf("Signed force: %+.6g N\n", signedForce_N);
fprintf("Pulse: %.6g to %.6g s; stop time: %.6g s\n", ...
    pulseStart_s, pulseStart_s + pulseDuration_s, stopTime_s);
fprintf("Controller rigidBodyTree remains nominal: YES\n\n");

fprintf("[1/2] Matched zero-force reference\n");
nominalInput = makeSimulationInput( ...
    model, controllerID, setupVariables, ...
    zeros(3, 1), pulseStart_s, pulseDuration_s, ...
    stopTime_s);
nominalOutput = sim(nominalInput);
nominal = extractTrajectory(nominalOutput, "matched nominal");

fprintf("[2/2] Requested disturbed case\n");
disturbedInput = makeSimulationInput( ...
    model, controllerID, setupVariables, ...
    forceVector, pulseStart_s, pulseDuration_s, ...
    stopTime_s);
disturbedOutput = sim(disturbedInput);
disturbed = extractTrajectory(disturbedOutput, "disturbed");

metrics = classifyCase(disturbed, nominal, controllerID, ...
    scale, trunkChange_percent, axisName, signedForce_N, ...
    pulseStart_s, pulseDuration_s, stopTime_s);

result = struct( ...
    "model", model, ...
    "controller", controllerID, ...
    "trunkMassInertiaScale", scale, ...
    "trunkMassInertiaChange_percent", trunkChange_percent, ...
    "axis", axisName, ...
    "signedForce_N", signedForce_N, ...
    "forceVector_N", forceVector, ...
    "metrics", metrics, ...
    "nominalTrajectory", nominal, ...
    "disturbedTrajectory", disturbed, ...
    "nominalSimulationOutput", nominalOutput, ...
    "disturbedSimulationOutput", disturbedOutput);

fprintf("\nSpecific-case metrics:\n");
disp(metrics);
fprintf("The requested disturbed run was simulated last.\n");
fprintf("The model remains open; the .slx file was not saved.\n");
end

function simIn = makeSimulationInput( ...
        model, controllerID, setupVariables, forceVector, ...
        pulseStart, pulseDuration, stopTime)
simIn = Simulink.SimulationInput(model);
for index = 1:numel(setupVariables)
    name = setupVariables{index};
    simIn = simIn.setVariable(name, evalin("caller", name));
end
[simIn, ~] = apply_frozen_pd_gravity_controller( ...
    simIn, controllerID, model);
simIn = simIn.setVariable("robustnessForceStart_s", pulseStart);
simIn = simIn.setVariable( ...
    "robustnessForceDuration_s", pulseDuration);
simIn = simIn.setVariable("robustnessForceVector_N", forceVector);
simIn = simIn.setModelParameter( ...
    "StopTime", num2str(stopTime, 17), ...
    "SignalLogging", "on", ...
    "SignalLoggingName", "logsout", ...
    "ReturnWorkspaceOutputs", "on", ...
    "UnconnectedInputMsg", "none", ...
    "UnconnectedOutputMsg", "none", ...
    "UnconnectedLineMsg", "none");
end

function trajectory = extractTrajectory(output, label)
assert(any(strcmp(output.who, "logsout")), ...
    "SpecificUncertainty:MissingLogsout", ...
    "%s run returned no logsout dataset.", label);
logs = output.logsout;
[qTime, q] = loggedMatrix(logs, "diag_q_rbt", "");
[pureTime, pureTorque] = loggedMatrix( ...
    logs, "Pure PD Torques", "");
[torqueTime, totalTorque] = loggedMatrix( ...
    logs, "diag_tau_leg_command", ...
    "HumanoidModel_BaselineBalanceControl/Final Actuator Torque Saturation");
q = requireWidth(q, 22, "diag_q_rbt");
pureTorque = requireWidth( ...
    pureTorque, 12, "Pure PD Torques");
totalTorque = requireWidth( ...
    totalTorque, 12, "diag_tau_leg_command");
trajectory = struct( ...
    "qTime", qTime, ...
    "q", q, ...
    "pureTime", pureTime, ...
    "pureTorque", pureTorque, ...
    "torqueTime", torqueTime, ...
    "totalTorque", totalTorque);
end

function row = classifyCase( ...
        disturbed, nominal, controller, scale, changePercent, ...
        axisName, signedForce, pulseStart, pulseDuration, stopTime)
t = disturbed.qTime(:);
q = disturbed.q;
qNominal = interpolateMatrix(nominal.qTime, nominal.q, t);
residual = q - qNominal;
if axisName == "lateral"
    axisIndex = 1;
    tiltIndex = 5;
else
    axisIndex = 2;
    tiltIndex = 4;
end
axisResidual = residual(:, axisIndex);
tiltResidual = residual(:, tiltIndex);
legResidual = residual(:, 11:22);
velocityResidual = numericalDerivative(t, axisResidual);

pulseEnd = pulseStart + pulseDuration;
early = t >= pulseEnd & t <= pulseEnd + 2;
late = t >= stopTime - 2;
lateDisplacement = rmsAll(axisResidual(late));
lateTilt = rad2deg(rmsAll(tiltResidual(late)));
lateVelocity = rmsAll(velocityResidual(late));
displacementRatio = safeRatio( ...
    lateDisplacement, rmsAll(axisResidual(early)));
tiltRatio = safeRatio( ...
    lateTilt, rad2deg(rmsAll(tiltResidual(early))));

qInitial = q(1, :);
verticalDrop = max(qInitial(3) - q(:, 3));
horizontalTravel = max(vecnorm( ...
    q(:, 1:2) - qInitial(1:2), 2, 2));
rotationDeviation = max(abs(rad2deg( ...
    q(:, 4:6) - qInitial(4:6))), [], "all");
fell = verticalDrop > 0.5 || ...
    horizontalTravel > 1 || rotationDeviation > 30;

recovered = ~fell && ...
    lateDisplacement <= 0.025 && ...
    lateTilt <= 2 && ...
    lateVelocity <= 0.02 && ...
    (displacementRatio <= 0.75 || lateDisplacement <= 0.005) && ...
    (tiltRatio <= 0.75 || lateTilt <= 0.5);
if fell
    outcome = "fell";
elseif recovered
    outcome = "recovered";
else
    outcome = "upright_not_recovered";
end

saturated = any(abs(disturbed.totalTorque) >= 100 - 1e-6, 2);
row = table(controller, scale, changePercent, axisName, ...
    signedForce, abs(signedForce) * pulseDuration, ...
    outcome, fell, recovered, verticalDrop, ...
    horizontalTravel, rotationDeviation, ...
    max(abs(axisResidual)), ...
    rad2deg(max(abs(tiltResidual))), ...
    lateDisplacement, lateTilt, lateVelocity, ...
    displacementRatio, tiltRatio, ...
    rmsAll(rad2deg(legResidual)), ...
    max(abs(rad2deg(legResidual)), [], "all"), ...
    rmsAll(disturbed.pureTorque), ...
    max(abs(disturbed.pureTorque), [], "all"), ...
    rmsAll(disturbed.totalTorque), ...
    max(abs(disturbed.totalTorque), [], "all"), ...
    any(saturated), ...
    'VariableNames', {'Controller', ...
    'TrunkMassInertiaScale', ...
    'TrunkMassInertiaChange_percent', ...
    'Axis', 'SignedForce_N', 'Impulse_Ns', ...
    'Outcome', 'Fell', 'Recovered', ...
    'PeakVerticalDrop_m', 'PeakHorizontalTravel_m', ...
    'PeakAbsoluteRotationDeviation_deg', ...
    'PeakForceInducedAxisDisplacement_m', ...
    'PeakForceInducedAxisTilt_deg', ...
    'LateForceInducedAxisDisplacementRMS_m', ...
    'LateForceInducedAxisTiltRMS_deg', ...
    'LateForceInducedAxisVelocityRMS_m_s', ...
    'AxisDisplacementLateEarlyRatio', ...
    'AxisTiltLateEarlyRatio', ...
    'ForceInducedPostureRMS_deg', ...
    'MaxForceInducedJointDeviation_deg', ...
    'PurePDTorqueRMS_Nm', 'PeakPurePDTorque_Nm', ...
    'AppliedTorqueRMS_Nm', 'PeakAppliedTorque_Nm', ...
    'ActuatorSaturation'});
end

function [time, values] = loggedMatrix(logs, signalName, expectedBlock)
matching = [];
for index = 1:logs.numElements
    candidate = logs.getElement(index);
    if strcmp(string(candidate.Name), signalName)
        if strlength(expectedBlock) == 0 || ...
                strcmp(elementBlockPath(candidate), expectedBlock)
            matching(end + 1) = index; %#ok<AGROW>
        end
    end
end
assert(~isempty(matching), ...
    "SpecificUncertainty:MissingSignal", ...
    "Logged signal ""%s"" from block ""%s"" was not found.", ...
    signalName, expectedBlock);
assert(isscalar(matching) || strlength(expectedBlock) == 0, ...
    "SpecificUncertainty:AmbiguousSignal", ...
    "Logged signal ""%s"" from block ""%s"" appears more than once.", ...
    signalName, expectedBlock);
element = logs.getElement(matching(1));
payload = element;
if isa(element, "Simulink.SimulationData.Signal")
    payload = element.Values;
end
[time, values] = flattenPayload(payload);
assert(~isempty(time) && ~isempty(values), ...
    "SpecificUncertainty:EmptySignal", ...
    "Logged signal ""%s"" is empty.", signalName);
time = double(time(:));
values = double(values);
if isvector(values)
    if isscalar(time)
        values = reshape(values, 1, []);
    else
        values = values(:);
    end
elseif size(values, 1) ~= numel(time) && ...
        size(values, 2) == numel(time)
    values = values.';
end
assert(size(values, 1) == numel(time), ...
    "SpecificUncertainty:SignalTimeMismatch", ...
    "Signal ""%s"" dimensions do not match its time vector.", ...
    signalName);
end

function path = elementBlockPath(element)
path = "";
try
    blockPath = element.BlockPath;
    path = string(blockPath.getBlock(blockPath.getLength));
catch
end
end

function [time, values] = flattenPayload(payload)
if isa(payload, "timeseries")
    time = payload.Time;
    values = squeeze(payload.Data);
    return
end
if istimetable(payload)
    time = seconds(payload.Properties.RowTimes - ...
        payload.Properties.RowTimes(1));
    values = payload.Variables;
    return
end
if isa(payload, "Simulink.SimulationData.Signal")
    [time, values] = flattenPayload(payload.Values);
    return
end
if isa(payload, "Simulink.SimulationData.Dataset")
    time = [];
    values = [];
    for index = 1:payload.numElements
        item = payload.getElement(index);
        [itemTime, itemValues] = flattenPayload(item);
        if isempty(time)
            time = itemTime(:);
            values = itemValues;
        else
            itemValues = alignToTime( ...
                itemTime, itemValues, time);
            values = [values, itemValues]; %#ok<AGROW>
        end
    end
    return
end
if isstruct(payload)
    if isfield(payload, "time") && isfield(payload, "signals")
        time = payload.time;
        values = squeeze(payload.signals.values);
        return
    end
    fields = fieldnames(payload);
    time = [];
    values = [];
    for index = 1:numel(fields)
        [itemTime, itemValues] = ...
            flattenPayload(payload.(fields{index}));
        if isempty(time)
            time = itemTime(:);
            values = itemValues;
        else
            itemValues = alignToTime( ...
                itemTime, itemValues, time);
            values = [values, itemValues]; %#ok<AGROW>
        end
    end
    return
end
error("SpecificUncertainty:UnsupportedSignalType", ...
    "Unsupported logged payload type: %s", class(payload));
end

function aligned = alignToTime(sourceTime, sourceValues, targetTime)
sourceTime = sourceTime(:);
if isscalar(sourceTime)
    aligned = repmat(sourceValues(1, :), numel(targetTime), 1);
else
    aligned = interp1(sourceTime, sourceValues, ...
        targetTime, "linear", "extrap");
end
end

function values = requireWidth(values, width, signalName)
values = squeeze(values);
if isvector(values) && width == 1
    values = values(:);
end
assert(size(values, 2) == width, ...
    "SpecificUncertainty:UnexpectedSignalWidth", ...
    "Signal ""%s"" has width %d; expected %d.", ...
    signalName, size(values, 2), width);
end

function output = interpolateMatrix(sourceTime, values, targetTime)
if isequal(sourceTime, targetTime)
    output = values;
else
    output = interp1(sourceTime, values, targetTime, ...
        "linear", "extrap");
end
end

function derivative = numericalDerivative(time, values)
if numel(time) < 2
    derivative = zeros(size(values));
else
    derivative = gradient(values, time);
end
end

function ratio = safeRatio(numerator, denominator)
if denominator <= 1e-12
    ratio = double(numerator > 1e-12) * Inf;
    if numerator <= 1e-12
        ratio = 0;
    end
else
    ratio = numerator / denominator;
end
end

function value = rmsAll(values)
value = sqrt(mean(values.^2, "all"));
end

function restoreModel(model, trunkBlock, densityExpression, dirtyState)
if bdIsLoaded(model)
    set_param(trunkBlock, "Density", densityExpression);
    if dirtyState == "off"
        set_param(model, "Dirty", "off");
    end
end
end
