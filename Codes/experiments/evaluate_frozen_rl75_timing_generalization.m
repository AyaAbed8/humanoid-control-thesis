function results = evaluate_frozen_rl75_timing_generalization( ...
    pulseStarts_s, forceMagnitude_N, outputRoot, trunkScale, axisName)
%EVALUATE_FROZEN_RL75_TIMING_GENERALIZATION Compare A and RL75 timings.
%   Uses matched zero-force references, +/- lateral force pulses, the
%   frozen deterministic RL75 policy, and the established 11 s recovery
%   definition. The models are loaded for simulation but never saved.

arguments
    pulseStarts_s (1,:) double ...
        {mustBeFinite,mustBeNonnegative} = [1.75 2.25]
    forceMagnitude_N (1,:) double ...
        {mustBeFinite,mustBePositive} = 150
    outputRoot (1,1) string = fullfile( ...
        project_codes_root(), "Results", ...
        "ResidualRLTimingGeneralization")
    trunkScale (1,1) double {mustBeFinite,mustBePositive} = 1
    axisName (1,1) string ...
        {mustBeMember(axisName,["lateral","sagittal"])} = "lateral"
end

assert(numel(unique(pulseStarts_s)) == numel(pulseStarts_s), ...
    "RLTiming:DuplicateOnset", "Pulse onsets must be unique.");
assert(numel(unique(forceMagnitude_N)) == numel(forceMagnitude_N), ...
    "RLTiming:DuplicateForce", "Force magnitudes must be unique.");

codeDirectory = string(project_codes_root());
cd(codeDirectory);
[agentObj, frozenRL] = load_frozen_residual_rl_agent();
humanoid_walker_parameters;
residual_rl_parameters;
inverseDynamics;
residualVariableNames = string(who("residualRL*"));
residualVariables = struct();
for variableIndex = 1:numel(residualVariableNames)
    variableName = residualVariableNames(variableIndex);
    residualVariables.(variableName) = eval(variableName);
end

p = protocol();
assert(all(pulseStarts_s + p.pulseDuration_s < p.stopTime_s), ...
    "RLTiming:InvalidOnset", ...
    "Every pulse must end before the simulation stop time.");
stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
resultDirectory = fullfile(outputRoot, stamp);
mkdir(resultDirectory);

definitions = makeDefinitions(pulseStarts_s, forceMagnitude_N);
trajectories = cell(height(definitions), 1);
metricRows = cell(height(definitions), 1);
nominalByController = struct();

fprintf("\nFrozen A versus RL75 generalisation test\n");
fprintf("Axis: %s; force magnitudes: +/- %s N\n", ...
    axisName, mat2str(forceMagnitude_N));
fprintf("Pulse duration: %g s; stop time: %g s\n", ...
    p.pulseDuration_s, p.stopTime_s);
fprintf("Pulse onsets: %s s\n", mat2str(pulseStarts_s));
fprintf("Simscape trunk mass/inertia scale: %.6g\n", trunkScale);
fprintf("Controller rigidBodyTree remains nominal: YES\n");
fprintf("Cases: %d; output: %s\n\n", ...
    height(definitions), resultDirectory);

warningState = warning;
warning("off", "all");
warningCleanup = onCleanup(@() warning(warningState));

for caseIndex = 1:height(definitions)
    d = definitions(caseIndex, :);
    fprintf("[%d/%d] %s, onset %.3g s, force %+.6g N\n", ...
        caseIndex, height(definitions), d.Controller, ...
        d.PulseStart_s, d.SignedForce_N);
    trajectory = runCase(d, p, agentObj, params, robotFull, ...
        u_shoulder_frontal, residualVariables, trunkScale, axisName);
    trajectories{caseIndex} = trajectory;
    key = matlab.lang.makeValidName(char(d.Controller));
    if d.SignedForce_N == 0
        nominalByController.(key) = trajectory;
        metricRows{caseIndex} = nominalRow(trajectory, d, p);
    else
        metricRows{caseIndex} = disturbedRow(trajectory, ...
            nominalByController.(key), d, p, axisName);
    end
    partialMetrics = vertcat(metricRows{1:caseIndex});
    writetable(partialMetrics, fullfile(resultDirectory, ...
        "timing_metrics_partial.csv"));
    save(fullfile(resultDirectory, "timing_checkpoint.mat"), ...
        "partialMetrics", "definitions", "p", "frozenRL", ...
        "trunkScale", "axisName");
end

metrics = vertcat(metricRows{:});
summary = makeSummary(metrics);
writetable(metrics, fullfile(resultDirectory, "timing_metrics.csv"));
writetable(summary, fullfile(resultDirectory, "timing_summary.csv"));
save(fullfile(resultDirectory, "processed_results.mat"), ...
    "metrics", "summary", "definitions", "p", "frozenRL", ...
    "trunkScale", "axisName", "trajectories", "-v7.3");

results = struct("resultDirectory", string(resultDirectory), ...
    "metrics", metrics, "summary", summary, ...
    "definitions", definitions, "protocol", p, ...
    "trunkMassInertiaScale", trunkScale, ...
    "axis", axisName, ...
    "trajectories", {trajectories});

fprintf("\nTiming-generalisation metrics:\n");
disp(metrics(:, ["Controller","PulseStart_s","SignedForce_N", ...
    "Outcome","LateAxisDisplacementRMS_m","LateAxisTiltRMS_deg", ...
    "LateAxisVelocityRMS_m_s","PeakResidualTorque_Nm", ...
    "PeakNormalizedAction","ActuatorSaturation"]));
fprintf("\nTiming-generalisation summary:\n");
disp(summary);
fprintf("Results written to %s\n", resultDirectory);
clear warningCleanup
open_system("HumanoidModel_ResidualRL");
end

function p = protocol()
p = struct( ...
    "pulseDuration_s", 0.1, "stopTime_s", 11, "finalDwell_s", 2, ...
    "fallVerticalDrop_m", 0.5, "fallHorizontalTravel_m", 1, ...
    "fallRotation_deg", 30, "saturationTolerance_Nm", 1e-6, ...
    "recoveryDisplacementRMS_m", 0.025, ...
    "recoveryTiltRMS_deg", 2, ...
    "recoveryVelocityRMS_m_s", 0.02, ...
    "settlingRatioLimit", 0.75, ...
    "smallDisplacementRMS_m", 0.005, ...
    "smallTiltRMS_deg", 0.5);
end

function definitions = makeDefinitions(onsets, magnitudes)
caseCount = 2 * (1 + 2 * numel(onsets) * numel(magnitudes));
controller = strings(caseCount, 1);
onset = zeros(caseCount, 1);
force = zeros(caseCount, 1);
row = 0;
for name = ["A","RL75"]
    row = row + 1;
    controller(row) = name;
    onset(row) = NaN;
    force(row) = 0;
    for start = onsets
        for magnitude = magnitudes
            indices = row + (1:2);
            controller(indices) = name;
            onset(indices) = start;
            force(indices) = [magnitude; -magnitude];
            row = row + 2;
        end
    end
end
definitions = table((1:numel(force))', controller, onset, force, ...
    'VariableNames', {'CaseNumber','Controller', ...
    'PulseStart_s','SignedForce_N'});
end

function trajectory = runCase(d, p, agentObj, params, robotFull, ...
        uShoulder, residualVariables, trunkScale, axisName)
if d.Controller == "A"
    model = "HumanoidModel_BaselineBalanceControl";
else
    model = "HumanoidModel_ResidualRL";
end
load_system(model);
modelCleanup = onCleanup(@() closeIfLoaded(model));
trunkBlock = model + "/Humanoid Robot/Torso and Head/Trunk";
assert(getSimulinkBlockHandle(trunkBlock) > 0, ...
    "RLTiming:TrunkBlockMissing", ...
    "Expected Trunk File Solid block was not found: %s", trunkBlock);
nominalDensity = string(get_param(trunkBlock, "Density"));
assert(nominalDensity == "trunkDensity", ...
    "RLTiming:UnexpectedTrunkDensity", ...
    "Expected Trunk Density expression trunkDensity, got %s.", ...
    nominalDensity);
set_param(trunkBlock, "Density", sprintf( ...
    "(%.17g)*(trunkDensity)", trunkScale));
simIn = Simulink.SimulationInput(model);
[simIn, ~] = apply_frozen_pd_gravity_controller(simIn, "A", model);
simIn = simIn.setVariable("params", params);
simIn = simIn.setVariable("robotFull", robotFull);
simIn = simIn.setVariable("u_shoulder_frontal", uShoulder);
start = d.PulseStart_s;
if isnan(start)
    start = 2;
end
forceVector = zeros(3, 1);
if axisName == "lateral"
    forceVector(1) = d.SignedForce_N;
else
    forceVector(2) = d.SignedForce_N;
end
simIn = simIn.setVariable("robustnessForceVector_N", forceVector);
simIn = simIn.setVariable("robustnessForceStart_s", start);
simIn = simIn.setVariable( ...
    "robustnessForceDuration_s", p.pulseDuration_s);
if d.Controller == "RL75"
    names = string(fieldnames(residualVariables));
    for index = 1:numel(names)
        simIn = simIn.setVariable(names(index), ...
            residualVariables.(names(index)));
    end
    simIn = simIn.setVariable("agentObj", agentObj);
    simIn = simIn.setVariable("residualRLEnabled", 1);
    simIn = simIn.setVariable("residualRLUseAgent", 1);
end
simIn = simIn.setModelParameter("StopTime", string(p.stopTime_s), ...
    "SignalLogging", "on", "SignalLoggingName", "logsout", ...
    "ReturnWorkspaceOutputs", "on", ...
    "UnconnectedInputMsg", "none", ...
    "UnconnectedOutputMsg", "none", ...
    "UnconnectedLineMsg", "none");
output = sim(simIn);
trajectory = extractTrajectory(output, d.Controller);
close_system(model, 0);
clear modelCleanup
end

function closeIfLoaded(model)
if getSimulinkBlockHandle(model) >= 0
    close_system(model, 0);
end
end

function trajectory = extractTrajectory(output, controller)
logs = output.logsout;
[qTime, q] = loggedMatrix(logs, "diag_q_rbt", "");
[torqueTime, torque] = loggedMatrix(logs, ...
    "diag_tau_leg_command", "Final Actuator Torque Saturation");
q = requireWidth(q, 22, "diag_q_rbt");
torque = requireWidth(torque, 12, "diag_tau_leg_command");
actionTime = qTime;
action = zeros(numel(qTime), 2);
residualTime = qTime;
residualTorque = zeros(numel(qTime), 2);
if controller == "RL75"
    [actionTime, action] = loggedMatrix( ...
        logs, "rl_action_normalized", "");
    [residualTime, residualTorque] = loggedMatrix( ...
        logs, "rl_residual_hip_torque_Nm", "");
    action = requireWidth(action, 2, "rl_action_normalized");
    residualTorque = requireWidth(residualTorque, 2, ...
        "rl_residual_hip_torque_Nm");
end
trajectory = struct("qTime", qTime, "q", q, ...
    "torqueTime", torqueTime, "totalTorque", torque, ...
    "actionTime", actionTime, "action", action, ...
    "residualTime", residualTime, "residualTorque", residualTorque);
end

function row = nominalRow(x, d, p)
q = x.q;
qInitial = q(1, :);
verticalDrop = max(qInitial(3) - q(:, 3));
horizontalTravel = max(vecnorm(q(:, 1:2) - qInitial(1:2), 2, 2));
rotation = max(abs(rad2deg(q(:, 4:6) - qInitial(4:6))), [], "all");
fell = verticalDrop > p.fallVerticalDrop_m || ...
    horizontalTravel > p.fallHorizontalTravel_m || ...
    rotation > p.fallRotation_deg;
outcome = "nominal_upright";
if fell
    outcome = "fell";
end
saturated = any(abs(x.totalTorque) >= ...
    100 - p.saturationTolerance_Nm, "all");
row = table(d.Controller, d.PulseStart_s, d.SignedForce_N, ...
    outcome, fell, false, 0, 0, 0, ...
    max(abs(x.residualTorque), [], "all"), ...
    max(abs(x.action), [], "all"), 0, saturated, ...
    'VariableNames', metricNames());
end

function row = disturbedRow(x, nominal, d, p, axisName)
t = x.qTime(:);
q = x.q;
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
velocity = gradient(axisResidual, t);
pulseEnd = d.PulseStart_s + p.pulseDuration_s;
early = t >= pulseEnd & t <= pulseEnd + 2;
late = t >= p.stopTime_s - p.finalDwell_s;
lateDisplacement = rmsAll(axisResidual(late));
lateTilt = rad2deg(rmsAll(tiltResidual(late)));
lateVelocity = rmsAll(velocity(late));
displacementRatio = safeRatio(lateDisplacement, ...
    rmsAll(axisResidual(early)));
tiltRatio = safeRatio(lateTilt, ...
    rad2deg(rmsAll(tiltResidual(early))));
qInitial = q(1, :);
verticalDrop = max(qInitial(3) - q(:, 3));
horizontalTravel = max(vecnorm(q(:, 1:2) - qInitial(1:2), 2, 2));
rotation = max(abs(rad2deg(q(:, 4:6) - qInitial(4:6))), [], "all");
fell = verticalDrop > p.fallVerticalDrop_m || ...
    horizontalTravel > p.fallHorizontalTravel_m || ...
    rotation > p.fallRotation_deg;
within = lateDisplacement <= p.recoveryDisplacementRMS_m && ...
    lateTilt <= p.recoveryTiltRMS_deg && ...
    lateVelocity <= p.recoveryVelocityRMS_m_s;
settled = (displacementRatio <= p.settlingRatioLimit || ...
    lateDisplacement <= p.smallDisplacementRMS_m) && ...
    (tiltRatio <= p.settlingRatioLimit || ...
    lateTilt <= p.smallTiltRMS_deg);
recovered = ~fell && within && settled;
outcome = "upright_not_recovered";
if fell
    outcome = "fell";
elseif recovered
    outcome = "recovered";
end
saturated = any(abs(x.totalTorque) >= ...
    100 - p.saturationTolerance_Nm, "all");
row = table(d.Controller, d.PulseStart_s, d.SignedForce_N, ...
    outcome, fell, recovered, lateDisplacement, lateTilt, ...
    lateVelocity, max(abs(x.residualTorque), [], "all"), ...
    max(abs(x.action), [], "all"), ...
    max(abs(axisResidual)), saturated, ...
    'VariableNames', metricNames());
end

function names = metricNames()
names = {'Controller','PulseStart_s','SignedForce_N','Outcome', ...
    'Fell','Recovered','LateAxisDisplacementRMS_m', ...
    'LateAxisTiltRMS_deg','LateAxisVelocityRMS_m_s', ...
    'PeakResidualTorque_Nm','PeakNormalizedAction', ...
    'PeakAxisDisplacement_m','ActuatorSaturation'};
end

function summary = makeSummary(metrics)
rows = metrics.SignedForce_N ~= 0;
data = metrics(rows, :);
forceMagnitude = abs(data.SignedForce_N);
groups = findgroups(data.Controller, data.PulseStart_s, forceMagnitude);
controller = splitapply(@(x) x(1), data.Controller, groups);
onset = splitapply(@(x) x(1), data.PulseStart_s, groups);
magnitude = splitapply(@(x) x(1), forceMagnitude, groups);
positive = splitapply(@(f,o) o(f > 0), ...
    data.SignedForce_N, data.Outcome, groups);
negative = splitapply(@(f,o) o(f < 0), ...
    data.SignedForce_N, data.Outcome, groups);
bidirectional = splitapply(@(x) all(x), data.Recovered, groups);
summary = table(controller, onset, magnitude, positive, negative, ...
    bidirectional, ...
    'VariableNames', {'Controller','PulseStart_s','ForceMagnitude_N', ...
    'PositiveOutcome','NegativeOutcome','BidirectionallyRecovered'});
end

function output = interpolateMatrix(sourceTime, values, targetTime)
output = interp1(sourceTime, values, targetTime, "linear", "extrap");
end

function ratio = safeRatio(numerator, denominator)
if denominator <= 1e-12
    ratio = double(numerator > 1e-12) * Inf;
else
    ratio = numerator / denominator;
end
end

function value = rmsAll(values)
value = sqrt(mean(values.^2, "all"));
end

function values = requireWidth(values, width, signalName)
if size(values, 1) == width && size(values, 2) ~= width
    values = values';
end
assert(size(values, 2) == width, "RLTiming:UnexpectedWidth", ...
    "Signal %s has width %d; expected %d.", ...
    signalName, size(values, 2), width);
end

function [time, values] = loggedMatrix(logs, signalName, blockHint)
matches = [];
for index = 1:logs.numElements
    candidate = logs.getElement(index);
    if string(candidate.Name) == signalName && ...
            (strlength(blockHint) == 0 || ...
            contains(elementBlockPath(candidate), blockHint))
        matches(end + 1) = index; %#ok<AGROW>
    end
end
assert(~isempty(matches), "RLTiming:SignalMissing", ...
    "Logged signal %s was not found.", signalName);
element = logs.getElement(matches(1));
payload = element;
if isa(element, "Simulink.SimulationData.Signal")
    payload = element.Values;
end
[time, values] = flattenPayload(payload);
time = double(time(:));
values = double(squeeze(values));
if isvector(values)
    if isscalar(time)
        values = reshape(values, 1, []);
    else
        values = values(:);
    end
elseif size(values, 1) ~= numel(time) && ...
        size(values, 2) == numel(time)
    values = values';
end
end

function path = elementBlockPath(element)
path = "";
try
    bp = element.BlockPath;
    path = string(bp.getBlock(bp.getLength));
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
            itemValues = alignToTime(itemTime, itemValues, time);
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
        [itemTime, itemValues] = flattenPayload(payload.(fields{index}));
        if isempty(time)
            time = itemTime(:);
            values = itemValues;
        else
            itemValues = alignToTime(itemTime, itemValues, time);
            values = [values, itemValues]; %#ok<AGROW>
        end
    end
    return
end
error("RLTiming:UnsupportedPayload", ...
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
