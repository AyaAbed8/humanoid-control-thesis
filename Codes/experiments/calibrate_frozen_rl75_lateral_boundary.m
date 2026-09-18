function results = calibrate_frozen_rl75_lateral_boundary( ...
    forceLevels_N, outputRoot, policyID)
%CALIBRATE_FROZEN_RL75_LATERAL_BOUNDARY Extend RL75's tested envelope.
%   Tests +/-175 N first and proceeds to +/-200 N only if both directions
%   at 175 N recover. The frozen deterministic RL75 policy, Controller A,
%   model, 100 ms pulse at 2 s, 11 s horizon, and established fall/recovery
%   rules are unchanged. The model is never saved.

arguments
    forceLevels_N (1,:) double ...
        {mustBeFinite,mustBePositive} = [175 200]
    outputRoot (1,1) string = fullfile( ...
        project_codes_root(), "Results", ...
        "FrozenRL75RecoveryBoundary")
    policyID (1,1) string ...
        {mustBeMember(policyID,["RL75","RL25_seed1"])} = "RL75"
end

assert(issorted(forceLevels_N) && ...
    numel(unique(forceLevels_N)) == numel(forceLevels_N), ...
    "FrozenRL75Boundary:InvalidForceGrid", ...
    "forceLevels_N must be unique and sorted in ascending order.");

thisDirectory = string(project_codes_root());
cd(thisDirectory);
protocol = boundaryProtocol();
if policyID == "RL75"
    [agentObj, frozenPolicy] = load_frozen_residual_rl_agent();
else
    [agentObj, frozenPolicy] = ...
        load_frozen_residual_rl_seed1_agent();
end

humanoid_walker_parameters;
residual_rl_parameters;
inverseDynamics;
residualVariableNames = string(who("residualRL*"));
residualVariables = struct();
for variableIndex = 1:numel(residualVariableNames)
    variableName = residualVariableNames(variableIndex);
    residualVariables.(variableName) = eval(variableName);
end

runStamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
resultDirectory = fullfile(outputRoot, runStamp);
mkdir(resultDirectory);

fprintf("\nFrozen %s lateral recovery-boundary extension\n", policyID);
fprintf("Force levels: %s N; signed pairs tested sequentially\n", ...
    mat2str(forceLevels_N));
fprintf("Pulse: %.6g to %.6g s; stop time: %.6g s\n", ...
    protocol.pulseStart_s, ...
    protocol.pulseStart_s + protocol.pulseDuration_s, ...
    protocol.stopTime_s);
fprintf("Output: %s\n\n", resultDirectory);

warningState = warning;
warning("off", "all");
warningCleanup = onCleanup(@() warning(warningState));

definitions = table(1, 0, "nominal", ...
    'VariableNames', {'CaseNumber','SignedForce_N','Condition'});
trajectories = cell(0, 1);
metricRows = cell(0, 1);

fprintf("[1] %s nominal reference\n", policyID);
nominal = runCase(0, protocol, agentObj, params, robotFull, ...
    u_shoulder_frontal, residualVariables);
trajectories{1, 1} = nominal;
metricRows{1, 1} = classifyNominal(nominal, protocol);

caseNumber = 1;
stopAfterLevel = false;
forceSigns = [1 -1];
for level = forceLevels_N
    levelRows = cell(2, 1);
    for directionIndex = 1:2
        signedForce = level * forceSigns(directionIndex);
        caseNumber = caseNumber + 1;
        condition = "positive";
        if signedForce < 0
            condition = "negative";
        end
        definitions(end + 1, :) = { ...
            caseNumber, signedForce, condition}; %#ok<AGROW>
        fprintf("[%d] %s, force %+.6g N\n", ...
            caseNumber, policyID, signedForce);
        trajectory = runCase(signedForce, protocol, agentObj, ...
            params, robotFull, u_shoulder_frontal, residualVariables);
        row = classifyDisturbed(trajectory, nominal, ...
            signedForce, protocol);
        trajectories{caseNumber, 1} = trajectory;
        metricRows{caseNumber, 1} = row;
        levelRows{directionIndex} = row;

        partialMetrics = vertcat(metricRows{:});
        writetable(partialMetrics, fullfile(resultDirectory, ...
            "boundary_metrics_partial.csv"));
        save(fullfile(resultDirectory, "boundary_checkpoint.mat"), ...
            "partialMetrics", "definitions", "protocol", ...
            "forceLevels_N", "frozenPolicy", "policyID");
    end
    pairMetrics = vertcat(levelRows{:});
    if ~all(pairMetrics.Recovered)
        fprintf("Stopping: +/-%.6g N was not recovered bidirectionally.\n", ...
            level);
        stopAfterLevel = true;
    end
    if stopAfterLevel
        break
    end
end

metrics = vertcat(metricRows{:});
summary = makeSummary(metrics);
writetable(metrics, fullfile(resultDirectory, "boundary_metrics.csv"));
writetable(summary, fullfile(resultDirectory, "boundary_summary.csv"));
save(fullfile(resultDirectory, "processed_results.mat"), ...
    "metrics", "summary", "definitions", "protocol", ...
    "forceLevels_N", "frozenPolicy", "policyID", ...
    "trajectories", "-v7.3");
makePlots(trajectories, definitions, resultDirectory, policyID);

results = struct( ...
    "resultDirectory", string(resultDirectory), ...
    "metrics", metrics, ...
    "summary", summary, ...
    "definitions", definitions, ...
    "protocol", protocol, ...
    "policyID", policyID, ...
    "trajectories", {trajectories});

fprintf("\nBoundary metrics:\n");
disp(metrics(:, ["SignedForce_N","Outcome", ...
    "PeakForceInducedAxisDisplacement_m", ...
    "PeakForceInducedAxisTilt_deg", ...
    "LateForceInducedAxisDisplacementRMS_m", ...
    "LateForceInducedAxisTiltRMS_deg", ...
    "ResidualTorqueRMS_Nm","PeakResidualTorque_Nm", ...
    "PeakNormalizedAction","ActuatorSaturation"]));
fprintf("\nBoundary summary:\n");
disp(summary);
fprintf("Results written to %s\n", resultDirectory);
end

function protocol = boundaryProtocol()
protocol = struct( ...
    "pulseStart_s", 2, ...
    "pulseDuration_s", 0.1, ...
    "stopTime_s", 11, ...
    "finalDwell_s", 2, ...
    "fallVerticalDrop_m", 0.5, ...
    "fallHorizontalTravel_m", 1, ...
    "fallRotation_deg", 30, ...
    "saturationTolerance_Nm", 1e-6, ...
    "recoveryDisplacementRMS_m", 0.025, ...
    "recoveryTiltRMS_deg", 2, ...
    "recoveryVelocityRMS_m_s", 0.02, ...
    "settlingRatioLimit", 0.75, ...
    "smallDisplacementRMS_m", 0.005, ...
    "smallTiltRMS_deg", 0.5);
end

function trajectory = runCase(signedForce_N, p, agentObj, ...
        params, robotFull, uShoulder, residualVariables)
model = "HumanoidModel_ResidualRL";
load_system(model);
modelCleanup = onCleanup(@() closeIfLoaded(model));

simIn = Simulink.SimulationInput(model);
[simIn, ~] = apply_frozen_pd_gravity_controller(simIn, "A", model);
simIn = simIn.setVariable("params", params);
simIn = simIn.setVariable("robotFull", robotFull);
simIn = simIn.setVariable("u_shoulder_frontal", uShoulder);
variableNames = string(fieldnames(residualVariables));
for variableIndex = 1:numel(variableNames)
    variableName = variableNames(variableIndex);
    simIn = simIn.setVariable(variableName, ...
        residualVariables.(variableName));
end
simIn = simIn.setVariable("agentObj", agentObj);
simIn = simIn.setVariable("residualRLEnabled", 1);
simIn = simIn.setVariable("residualRLUseAgent", 1);
simIn = simIn.setVariable( ...
    "robustnessForceVector_N", [signedForce_N; 0; 0]);
simIn = simIn.setVariable("robustnessForceStart_s", p.pulseStart_s);
simIn = simIn.setVariable( ...
    "robustnessForceDuration_s", p.pulseDuration_s);
simIn = simIn.setModelParameter( ...
    "StopTime", string(p.stopTime_s), ...
    "SignalLogging", "on", ...
    "SignalLoggingName", "logsout", ...
    "ReturnWorkspaceOutputs", "on", ...
    "UnconnectedInputMsg", "none", ...
    "UnconnectedOutputMsg", "none", ...
    "UnconnectedLineMsg", "none");
output = sim(simIn);
trajectory = extractTrajectory(output);
close_system(model, 0);
clear modelCleanup
end

function closeIfLoaded(model)
if getSimulinkBlockHandle(model) >= 0
    close_system(model, 0);
end
end

function trajectory = extractTrajectory(output)
assert(any(strcmp(output.who, "logsout")), ...
    "FrozenRL75Boundary:LogsMissing", ...
    "Simulation returned no logsout dataset.");
logs = output.logsout;
[qTime, q] = loggedMatrix(logs, "diag_q_rbt", "");
[torqueTime, totalTorque] = loggedMatrix(logs, ...
    "diag_tau_leg_command", "Final Actuator Torque Saturation");
[actionTime, action] = loggedMatrix( ...
    logs, "rl_action_normalized", "");
[residualTime, residualTorque] = loggedMatrix( ...
    logs, "rl_residual_hip_torque_Nm", "");
q = requireWidth(q, 22, "diag_q_rbt");
totalTorque = requireWidth(totalTorque, 12, "diag_tau_leg_command");
action = requireWidth(action, 2, "rl_action_normalized");
residualTorque = requireWidth( ...
    residualTorque, 2, "rl_residual_hip_torque_Nm");
trajectory = struct( ...
    "qTime", qTime, "q", q, ...
    "torqueTime", torqueTime, "totalTorque", totalTorque, ...
    "actionTime", actionTime, "action", action, ...
    "residualTime", residualTime, ...
    "residualTorque", residualTorque);
end

function row = classifyNominal(trajectory, p)
q = trajectory.q;
qInitial = q(1, :);
[fell, fallVertical, fallTravel, fallRotation, ...
    verticalDrop, horizontalTravel, rotationDeviation] = ...
    fallMetrics(q, qInitial, p);
outcome = "nominal_upright";
if fell
    outcome = "fell";
end
late = trajectory.qTime >= p.stopTime_s - p.finalDwell_s;
lateralVelocity = numericalDerivative( ...
    trajectory.qTime, q(:, 1) - qInitial(1));
row = metricRow(0, outcome, fell, false, ...
    fallVertical, fallTravel, fallRotation, verticalDrop, ...
    horizontalTravel, rotationDeviation, 0, 0, ...
    rmsAll(q(late, 1) - qInitial(1)), ...
    rad2deg(rmsAll(q(late, 5) - qInitial(5))), ...
    rmsAll(lateralVelocity(late)), 0, 0, trajectory, p);
end

function row = classifyDisturbed(disturbed, nominal, signedForce_N, p)
t = disturbed.qTime(:);
q = disturbed.q;
qNominal = interpolateMatrix(nominal.qTime, nominal.q, t);
qResidual = q - qNominal;
axisResidual = qResidual(:, 1);
tiltResidual = qResidual(:, 5);
velocityResidual = numericalDerivative(t, axisResidual);

pulseEnd = p.pulseStart_s + p.pulseDuration_s;
early = t >= pulseEnd & t <= pulseEnd + 2;
late = t >= p.stopTime_s - p.finalDwell_s;
earlyDisplacement = rmsAll(axisResidual(early));
lateDisplacement = rmsAll(axisResidual(late));
earlyTilt = rad2deg(rmsAll(tiltResidual(early)));
lateTilt = rad2deg(rmsAll(tiltResidual(late)));
displacementRatio = safeRatio(lateDisplacement, earlyDisplacement);
tiltRatio = safeRatio(lateTilt, earlyTilt);
lateVelocity = rmsAll(velocityResidual(late));

qInitial = q(1, :);
[fell, fallVertical, fallTravel, fallRotation, ...
    verticalDrop, horizontalTravel, rotationDeviation] = ...
    fallMetrics(q, qInitial, p);
withinEnvelope = lateDisplacement <= p.recoveryDisplacementRMS_m && ...
    lateTilt <= p.recoveryTiltRMS_deg && ...
    lateVelocity <= p.recoveryVelocityRMS_m_s;
displacementSettled = displacementRatio <= p.settlingRatioLimit || ...
    lateDisplacement <= p.smallDisplacementRMS_m;
tiltSettled = tiltRatio <= p.settlingRatioLimit || ...
    lateTilt <= p.smallTiltRMS_deg;
recovered = ~fell && withinEnvelope && ...
    displacementSettled && tiltSettled;
if fell
    outcome = "fell";
elseif recovered
    outcome = "recovered";
else
    outcome = "upright_not_recovered";
end

row = metricRow(signedForce_N, outcome, fell, recovered, ...
    fallVertical, fallTravel, fallRotation, verticalDrop, ...
    horizontalTravel, rotationDeviation, ...
    max(abs(axisResidual)), rad2deg(max(abs(tiltResidual))), ...
    lateDisplacement, lateTilt, lateVelocity, ...
    displacementRatio, tiltRatio, disturbed, p);
end

function row = metricRow(signedForce_N, outcome, fell, recovered, ...
        fallVertical, fallTravel, fallRotation, verticalDrop, ...
        horizontalTravel, rotationDeviation, peakAxis, peakTilt, ...
        lateDisplacement, lateTilt, lateVelocity, ...
        displacementRatio, tiltRatio, trajectory, p)
saturated = any(abs(trajectory.totalTorque) >= ...
    100 - p.saturationTolerance_Nm, 2);
row = table(signedForce_N, ...
    abs(signedForce_N) * p.pulseDuration_s, string(outcome), ...
    fell, recovered, fallVertical, fallTravel, fallRotation, ...
    verticalDrop, horizontalTravel, rotationDeviation, ...
    any(saturated), maskDuration(trajectory.torqueTime, saturated), ...
    peakAxis, peakTilt, lateDisplacement, lateTilt, lateVelocity, ...
    displacementRatio, tiltRatio, ...
    rmsAll(trajectory.totalTorque), ...
    max(abs(trajectory.totalTorque), [], "all"), ...
    rmsAll(trajectory.residualTorque), ...
    max(abs(trajectory.residualTorque), [], "all"), ...
    max(abs(trajectory.action), [], "all"), ...
    'VariableNames', {'SignedForce_N','Impulse_Ns','Outcome', ...
    'Fell','Recovered','FallByVerticalDrop', ...
    'FallByHorizontalTravel','FallByRotation','PeakVerticalDrop_m', ...
    'PeakHorizontalTravel_m','PeakAbsoluteRotationDeviation_deg', ...
    'ActuatorSaturation','ActuatorSaturationDuration_s', ...
    'PeakForceInducedAxisDisplacement_m', ...
    'PeakForceInducedAxisTilt_deg', ...
    'LateForceInducedAxisDisplacementRMS_m', ...
    'LateForceInducedAxisTiltRMS_deg', ...
    'LateForceInducedAxisVelocityRMS_m_s', ...
    'AxisDisplacementLateEarlyRatio','AxisTiltLateEarlyRatio', ...
    'AppliedTorqueRMS_Nm','PeakAppliedTorque_Nm', ...
    'ResidualTorqueRMS_Nm','PeakResidualTorque_Nm', ...
    'PeakNormalizedAction'});
end

function summary = makeSummary(metrics)
disturbed = metrics.SignedForce_N ~= 0;
levels = unique(abs(metrics.SignedForce_N(disturbed)));
positiveOutcome = strings(numel(levels), 1);
negativeOutcome = strings(numel(levels), 1);
bidirectionalRecovered = false(numel(levels), 1);
for index = 1:numel(levels)
    level = levels(index);
    positive = metrics.SignedForce_N == level;
    negative = metrics.SignedForce_N == -level;
    positiveOutcome(index) = metrics.Outcome(positive);
    negativeOutcome(index) = metrics.Outcome(negative);
    bidirectionalRecovered(index) = ...
        metrics.Recovered(positive) && metrics.Recovered(negative);
end
summary = table(levels, positiveOutcome, negativeOutcome, ...
    bidirectionalRecovered, ...
    'VariableNames', {'ForceMagnitude_N','PositiveOutcome', ...
    'NegativeOutcome','BidirectionallyRecovered'});
end

function makePlots(trajectories, definitions, resultDirectory, policyID)
nominal = trajectories{1};
for index = 2:numel(trajectories)
    trajectory = trajectories{index};
    definition = definitions(index, :);
    t = trajectory.qTime;
    nominalQ = interpolateMatrix(nominal.qTime, nominal.q, t);
    figureHandle = figure("Visible", "off", ...
        "Color", "white", "Position", [100 100 1050 760]);
    tiledlayout(3, 1, "TileSpacing", "compact");
    nexttile;
    plot(t, trajectory.q(:, 1) - nominalQ(:, 1), ...
        "LineWidth", 1.3);
    ylabel("Lateral residual (m)"); grid on;
    title(sprintf("Frozen %s, lateral force %+.6g N", ...
        policyID, definition.SignedForce_N));
    nexttile;
    plot(t, rad2deg(trajectory.q(:, 5) - nominalQ(:, 5)), ...
        "LineWidth", 1.3);
    ylabel("Tilt residual (deg)"); grid on;
    nexttile;
    plot(trajectory.actionTime, trajectory.action, "LineWidth", 1.1);
    ylabel("Normalized action"); xlabel("Time (s)"); grid on;
    legend("Right hip sagittal", "Left hip sagittal", ...
        "Location", "best");
    exportgraphics(figureHandle, fullfile(resultDirectory, ...
        sprintf("%s_force_%+gN.png", lower(policyID), ...
        definition.SignedForce_N)), ...
        "Resolution", 180);
    close(figureHandle);
end
end

function [fell, fallVertical, fallTravel, fallRotation, ...
        verticalDrop, horizontalTravel, rotationDeviation] = ...
        fallMetrics(q, qInitial, p)
verticalDrop = max(qInitial(3) - q(:, 3));
horizontalTravel = max(vecnorm( ...
    q(:, 1:2) - qInitial(1:2), 2, 2));
rotationDeviation = max(abs(rad2deg( ...
    q(:, 4:6) - qInitial(4:6))), [], "all");
fallVertical = verticalDrop > p.fallVerticalDrop_m;
fallTravel = horizontalTravel > p.fallHorizontalTravel_m;
fallRotation = rotationDeviation > p.fallRotation_deg;
fell = fallVertical || fallTravel || fallRotation;
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

function duration = maskDuration(time, mask)
if numel(time) < 2 || ~any(mask)
    duration = 0;
else
    duration = trapz(time, double(mask));
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
if isempty(values)
    value = NaN;
else
    value = sqrt(mean(values.^2, "all"));
end
end

function values = requireWidth(values, width, signalName)
if size(values, 2) == width
    return
end
if size(values, 1) == width
    values = values';
end
assert(size(values, 2) == width, ...
    "FrozenRL75Boundary:UnexpectedWidth", ...
    "Signal %s has width %d; expected %d.", ...
    signalName, size(values, 2), width);
end

function [time, values] = loggedMatrix(logs, signalName, blockHint)
matching = [];
for index = 1:logs.numElements
    candidate = logs.getElement(index);
    if strcmp(string(candidate.Name), signalName)
        if strlength(blockHint) == 0 || ...
                contains(elementBlockPath(candidate), blockHint)
            matching(end + 1) = index; %#ok<AGROW>
        end
    end
end
assert(~isempty(matching), ...
    "FrozenRL75Boundary:SignalMissing", ...
    "Logged signal %s from %s was not found.", ...
    signalName, blockHint);
element = logs.getElement(matching(1));
payload = element;
if isa(element, "Simulink.SimulationData.Signal")
    payload = element.Values;
end
[time, values] = flattenPayload(payload);
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
    "FrozenRL75Boundary:SignalShape", ...
    "Cannot align time and data for %s.", signalName);
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
error("FrozenRL75Boundary:UnsupportedPayload", ...
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
