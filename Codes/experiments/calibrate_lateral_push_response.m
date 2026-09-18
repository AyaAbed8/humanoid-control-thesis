function results = calibrate_lateral_push_response( ...
    forceMagnitude_N, stopTime_s, outputRoot, pulseStart_s, pulseDuration_s)
%CALIBRATE_LATERAL_PUSH_RESPONSE Compare A/D under small lateral pushes.
%   RESULTS = CALIBRATE_LATERAL_PUSH_RESPONSE() runs six cases:
%     1: A, zero force       4: D, zero force
%     2: A, +x force        5: D, +x force
%     3: A, -x force        6: D, -x force
%
%   The script logs the world-frame torso state, joint posture, balance
%   feedback torque, final applied torque, commanded force, and the six
%   normal contact forces under each foot. Contact logging is enabled only
%   in memory; the Simulink model is never saved by this function.
%
%   "ExploratoryPattern" is a trajectory-description aid, not the final
%   thesis fall/recovery classification. Formal thresholds must be frozen
%   after inspecting these calibration trajectories.

arguments
    forceMagnitude_N (1,1) double {mustBeFinite, mustBeNonnegative} = 5
    stopTime_s (1,1) double {mustBeFinite, mustBePositive} = 11
    outputRoot (1,1) string = ""
    pulseStart_s (1,1) double {mustBeFinite, mustBeNonnegative} = 2
    pulseDuration_s (1,1) double {mustBeFinite, mustBePositive} = 0.1
end

assert(pulseStart_s + pulseDuration_s < stopTime_s, ...
    "LateralCalibration:PulseOutsideRun", ...
    "The complete force pulse must occur before StopTime.");

model = "HumanoidModel_BaselineBalanceControl";
thisDirectory = string(project_codes_root());
modelFile = fullfile(thisDirectory, "models", model + ".slx");
assert(isfile(modelFile), "LateralCalibration:ModelMissing", ...
    "Model not found: %s", modelFile);

originalDirectory = string(pwd);
directoryCleanup = onCleanup(@() cd(originalDirectory));
cd(thisDirectory);

variablesBeforeSetup = who;
humanoid_walker_parameters;
setupVariables = setdiff(who, variablesBeforeSetup);
% Authoritative project construction path, matching inverseDynamics.m and
% the experiments used to freeze Controller D.
robotFull = build_manual_humanoid_tree_BFreversed();
robotFull.Gravity = [0 0 -9.80665];
setupVariables = [setupVariables; {"robotFull"}];

load_system(model);
% Deliberately leave the model open after the experiment so the user can
% inspect the final Simscape state and logged signals interactively.
assertForceInterface(model);
contactLoggingCleanup = enableContactLoggingInMemory(model); %#ok<NASGU>

if strlength(outputRoot) == 0
    outputRoot = fullfile(thisDirectory, "Results", ...
        "LateralPushCalibration");
end
runStamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
resultDirectory = fullfile(outputRoot, runStamp);
if ~isfolder(resultDirectory)
    mkdir(resultDirectory);
end

caseTable = table( ...
    (1:6).', ["A"; "A"; "A"; "D"; "D"; "D"], ...
    ["nominal"; "positive_x"; "negative_x"; ...
     "nominal"; "positive_x"; "negative_x"], ...
    [0; forceMagnitude_N; -forceMagnitude_N; ...
     0; forceMagnitude_N; -forceMagnitude_N], ...
    'VariableNames', ...
    {'CaseNumber', 'Controller', 'Condition', 'ForceX_N'});

requiredLogs = [ ...
    "diag_q_rbt"
    "Pure PD Torques"
    "diag_tau_leg_command"
    "robustness_force_command_N"
    "calibration_contact_normal_forces_N"];

rawOutputs = cell(height(caseTable), 1);
trajectories = cell(height(caseTable), 1);
metricRows = cell(height(caseTable), 1);

fprintf("\nLateral push calibration\n");
fprintf("Force: +/- %.6g N; pulse: %.6g to %.6g s\n", ...
    forceMagnitude_N, pulseStart_s, pulseStart_s + pulseDuration_s);
fprintf("Stop time: %.6g s\nResults: %s\n\n", ...
    stopTime_s, resultDirectory);

for caseIndex = 1:height(caseTable)
    definition = caseTable(caseIndex, :);
    forceVector = [definition.ForceX_N; 0; 0];
    fprintf("[%d/6] Controller %s, %s, Fx = %+.6g N\n", ...
        caseIndex, definition.Controller, ...
        definition.Condition, definition.ForceX_N);

    simIn = Simulink.SimulationInput(model);
    for variableIndex = 1:numel(setupVariables)
        variableName = setupVariables{variableIndex};
        simIn = simIn.setVariable(variableName, eval(variableName));
    end
    [simIn, ~] = apply_frozen_pd_gravity_controller( ...
        simIn, definition.Controller, model);
    simIn = simIn.setVariable( ...
        "robustnessForceStart_s", pulseStart_s);
    simIn = simIn.setVariable( ...
        "robustnessForceDuration_s", pulseDuration_s);
    simIn = simIn.setVariable( ...
        "robustnessForceVector_N", forceVector);
    simIn = simIn.setModelParameter( ...
        "StopTime", num2str(stopTime_s, 17), ...
        "SignalLogging", "on", ...
        "SignalLoggingName", "logsout", ...
        "ReturnWorkspaceOutputs", "on", ...
        "UnconnectedInputMsg", "none", ...
        "UnconnectedOutputMsg", "none", ...
        "UnconnectedLineMsg", "none");

    output = sim(simIn);
    rawOutputs{caseIndex} = output;
    assert(any(strcmp(output.who, "logsout")), ...
        "LateralCalibration:MissingLogsout", ...
        "Case %d returned no logsout dataset.", caseIndex);
    assertRequiredLogs(output.logsout, requiredLogs, caseIndex);

    [metricRows{caseIndex}, trajectories{caseIndex}] = ...
        processCase(output.logsout, definition, ...
        stopTime_s, pulseStart_s, pulseDuration_s);
end

metrics = vertcat(metricRows{:});
writetable(metrics, fullfile(resultDirectory, "case_metrics.csv"));
save(fullfile(resultDirectory, "processed_results.mat"), ...
    "metrics", "trajectories", "caseTable", ...
    "forceMagnitude_N", "stopTime_s", ...
    "pulseStart_s", "pulseDuration_s", "model");
save(fullfile(resultDirectory, "raw_simulation_outputs.mat"), ...
    "rawOutputs", "-v7.3");
createPlots(trajectories, pulseStart_s, pulseDuration_s, ...
    resultDirectory);

results = struct( ...
    "model", model, ...
    "resultDirectory", string(resultDirectory), ...
    "metrics", metrics, ...
    "trajectories", {trajectories}, ...
    "caseDefinitions", caseTable);

fprintf("\nCalibration metrics:\n");
disp(metrics);
fprintf("\nInterpretation guide:\n");
fprintf("  Late/Early ratios below 1 suggest decay; above 1 suggest growth.\n");
fprintf("  Near-zero minimum foot load indicates substantial unloading.\n");
fprintf("  ExploratoryPattern is descriptive and is not a frozen pass/fail rule.\n");
fprintf("Results written to %s\n", resultDirectory);
end

function assertForceInterface(model)
forceBlock = model + ...
    "/Humanoid Robot/Robustness External Force";
assert(getSimulinkBlockHandle(forceBlock) > 0, ...
    "LateralCalibration:ForceInterfaceMissing", ...
    "The provided baseline model is missing its robustness force interface.");
assert(strcmp(get_param(forceBlock, "EnableForce"), "on") && ...
    strcmp(get_param(forceBlock, "ForceResolutionFrame"), "World"), ...
    "LateralCalibration:ForceInterfaceInvalid", ...
    "The robustness force block must be enabled in the world frame.");
end

function cleanup = enableContactLoggingInMemory(model)
contactLines = find_system(model, "FindAll", "on", ...
    "Type", "line", "Name", "ContactSensing");
assert(isscalar(contactLines), ...
    "LateralCalibration:ContactSignalNotUnique", ...
    "Expected exactly one top-level ContactSensing signal.");
sourcePort = get_param(contactLines(1), "SrcPortHandle");
originalLogging = get_param(sourcePort, "DataLogging");
originalNameMode = get_param(sourcePort, "DataLoggingNameMode");
originalName = get_param(sourcePort, "DataLoggingName");
originalDirty = get_param(model, "Dirty");
set_param(sourcePort, ...
    "DataLogging", "on", ...
    "DataLoggingNameMode", "Custom", ...
    "DataLoggingName", "calibration_contact_normal_forces_N");
cleanup = onCleanup(@() restoreContactLogging( ...
    model, sourcePort, originalLogging, originalNameMode, ...
    originalName, originalDirty));
end

function restoreContactLogging(model, sourcePort, logging, nameMode, ...
        name, originalDirty)
set_param(sourcePort, ...
    "DataLogging", logging, ...
    "DataLoggingNameMode", nameMode, ...
    "DataLoggingName", name);
if strcmp(originalDirty, "off")
    set_param(model, "Dirty", "off");
end
end

function [row, trajectory] = processCase( ...
    logs, definition, stopTime, pulseStart, pulseDuration)
[qTime, q] = loggedMatrix(logs, "diag_q_rbt", "");
[pureTime, pureTorque] = loggedMatrix(logs, "Pure PD Torques", "");
[torqueTime, totalTorque] = ...
    loggedMatrix(logs, "diag_tau_leg_command", ...
    "HumanoidModel_BaselineBalanceControl/Final Actuator Torque Saturation");
[forceTime, force] = ...
    loggedMatrix(logs, "robustness_force_command_N", "");
[contactTime, contact] = ...
    loggedMatrix(logs, "calibration_contact_normal_forces_N", "");

q = requireWidth(q, 22, "diag_q_rbt");
pureTorque = requireWidth(pureTorque, 12, "Pure PD Torques");
totalTorque = requireWidth(totalTorque, 12, "diag_tau_leg_command");
force = requireWidth(force, 3, "robustness_force_command_N");
contact = requireWidth(contact, 12, ...
    "calibration_contact_normal_forces_N");

% Validated floating-base order: x, y, z, qx, qy, qz. In this project,
% qy is the lateral torso-tilt coordinate for an x-direction push.
torsoX = q(:, 1);
torsoLateralTilt = q(:, 5);
legAngles = q(:, 11:22);
torsoXDeviation = torsoX - torsoX(1);
torsoLateralTiltDeviation = torsoLateralTilt - torsoLateralTilt(1);
legDeviation = legAngles - legAngles(1, :);
torsoXVelocity = numericalDerivative(qTime, torsoX);

% ContactSensing is right-foot channels 1:6, then left-foot 7:12.
rightFootLoad = sum(max(contact(:, 1:6), 0), 2);
leftFootLoad = sum(max(contact(:, 7:12), 0), 2);

pulseEnd = pulseStart + pulseDuration;
earlyEnd = min(stopTime, pulseEnd + 2);
lateStart = max(pulseEnd, stopTime - 2);
earlyMask = qTime >= pulseEnd & qTime <= earlyEnd;
lateMask = qTime >= lateStart;
contactPostMask = contactTime >= pulseEnd;

earlyXRms = timeRms(qTime(earlyMask), ...
    torsoXDeviation(earlyMask));
lateXRms = timeRms(qTime(lateMask), ...
    torsoXDeviation(lateMask));
earlyTiltRms = timeRms(qTime(earlyMask), ...
    torsoLateralTiltDeviation(earlyMask));
lateTiltRms = timeRms(qTime(lateMask), ...
    torsoLateralTiltDeviation(lateMask));
xRatio = safeRatio(lateXRms, earlyXRms);
tiltRatio = safeRatio(lateTiltRms, earlyTiltRms);

finalTime = max([qTime(end), torqueTime(end), forceTime(end)]);
reachedStopTime = finalTime >= ...
    stopTime - max(1e-8, 1e-6 * stopTime);
minimumRightLoad = min(rightFootLoad(contactPostMask));
minimumLeftLoad = min(leftFootLoad(contactPostMask));
pattern = exploratoryPattern( ...
    definition.ForceX_N, xRatio, tiltRatio, ...
    torsoXDeviation(end), max(abs(torsoXDeviation)));

row = [definition, table( ...
    finalTime, reachedStopTime, ...
    rad2deg(max(abs(torsoLateralTiltDeviation))), ...
    max(abs(torsoXDeviation)), torsoXDeviation(end), ...
    torsoXVelocity(end), ...
    rmsAll(rad2deg(legDeviation)), ...
    max(abs(rad2deg(legDeviation)), [], "all"), ...
    earlyXRms, lateXRms, xRatio, ...
    rad2deg(earlyTiltRms), rad2deg(lateTiltRms), tiltRatio, ...
    minimumRightLoad, minimumLeftLoad, ...
    timeRms(pureTime, pureTorque), ...
    max(abs(pureTorque), [], "all"), ...
    timeRms(torqueTime, totalTorque), ...
    max(abs(totalTorque), [], "all"), pattern, ...
    'VariableNames', { ...
    'FinalTime_s', 'ReachedStopTime', ...
    'PeakTorsoLateralTiltDeviation_deg', ...
    'PeakTorsoXDeviation_m', 'FinalTorsoXDeviation_m', ...
    'FinalTorsoXVelocity_m_s', ...
    'PostureRMSError_deg', 'MaxJointDeviation_deg', ...
    'EarlyTorsoXRMS_m', 'LateTorsoXRMS_m', ...
    'TorsoXLateEarlyRatio', ...
    'EarlyTorsoLateralTiltRMS_deg', 'LateTorsoLateralTiltRMS_deg', ...
    'TorsoLateralTiltLateEarlyRatio', ...
    'MinimumRightFootLoad_N', 'MinimumLeftFootLoad_N', ...
    'PurePDTorqueRMS_Nm', 'PeakPurePDTorque_Nm', ...
    'TotalTorqueRMS_Nm', 'PeakTotalTorque_Nm', ...
    'ExploratoryPattern'})];

trajectory = struct( ...
    "definition", definition, ...
    "qTime", qTime, ...
    "q", q, ...
    "torsoXDeviation", torsoXDeviation, ...
    "torsoLateralTiltDeviation", torsoLateralTiltDeviation, ...
    "torsoXVelocity", torsoXVelocity, ...
    "legDeviation", legDeviation, ...
    "pureTime", pureTime, ...
    "pureTorque", pureTorque, ...
    "torqueTime", torqueTime, ...
    "totalTorque", totalTorque, ...
    "forceTime", forceTime, ...
    "force", force, ...
    "contactTime", contactTime, ...
    "rightFootLoad", rightFootLoad, ...
    "leftFootLoad", leftFootLoad);
end

function pattern = exploratoryPattern( ...
    forceX, xRatio, tiltRatio, finalX, peakX)
if forceX == 0
    pattern = "nominal reference";
    return
end
growthRatio = max(xRatio, tiltRatio);
if growthRatio > 1.25 && abs(finalX) > 0.5 * peakX
    pattern = "growing response";
elseif growthRatio > 0.75
    pattern = "not settled";
elseif growthRatio <= 0.25
    pattern = "substantial decay";
else
    pattern = "partial decay";
end
end

function assertRequiredLogs(logs, requiredNames, caseIndex)
available = strings(logs.numElements, 1);
for index = 1:logs.numElements
    available(index) = string(logs.getElement(index).Name);
end
missing = requiredNames(~ismember(requiredNames, available));
assert(isempty(missing), "LateralCalibration:MissingLogs", ...
    "Case %d is missing logged signals: %s", ...
    caseIndex, strjoin(missing, ", "));
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
assert(~isempty(matching), "LateralCalibration:LogMissing", ...
    "Logged signal '%s' from block '%s' was not found.", ...
    signalName, expectedBlock);
assert(isscalar(matching) || strlength(expectedBlock) == 0, ...
    "LateralCalibration:AmbiguousLog", ...
    "Logged signal '%s' from block '%s' appears more than once.", ...
    signalName, expectedBlock);
element = logs.getElement(matching(1));
payload = element;
if isa(element, "Simulink.SimulationData.Signal")
    payload = element.Values;
end
[time, values] = flattenPayload(payload);
assert(~isempty(time) && ~isempty(values), ...
    "LateralCalibration:EmptySignal", ...
    "Logged signal %s is empty.", signalName);
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
    "LateralCalibration:SignalShape", ...
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
        [itemTime, itemValues] = flattenPayload(payload.(fields{index}));
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
error("LateralCalibration:UnsupportedLogType", ...
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
    "LateralCalibration:UnexpectedWidth", ...
    "%s must contain %d columns; received %d.", ...
    signalName, width, size(values, 2));
end

function derivative = numericalDerivative(time, values)
if numel(time) < 2
    derivative = zeros(size(values));
else
    derivative = gradient(values, time);
end
end

function value = timeRms(time, values)
if isempty(values)
    value = NaN;
    return
end
if numel(time) < 2
    value = sqrt(mean(values(:).^2));
    return
end
meanSquare = trapz(time, sum(values.^2, 2)) / ...
    ((time(end) - time(1)) * size(values, 2));
value = sqrt(max(meanSquare, 0));
end

function ratio = safeRatio(numerator, denominator)
if denominator <= eps
    if numerator <= eps
        ratio = 1;
    else
        ratio = Inf;
    end
else
    ratio = numerator / denominator;
end
end

function value = rmsAll(values)
value = sqrt(mean(values(:).^2));
end

function createPlots(trajectories, pulseStart, pulseDuration, outputDirectory)
figureHandle = figure( ...
    "Name", "Lateral push calibration", ...
    "Color", "w", "Position", [80 80 1250 850]);
layout = tiledlayout(3, 2, "TileSpacing", "compact", ...
    "Padding", "compact");
colors = lines(numel(trajectories));

for index = 1:numel(trajectories)
    trajectory = trajectories{index};
    label = sprintf("%s %s", ...
        trajectory.definition.Controller, ...
        strrep(trajectory.definition.Condition, "_", " "));

    nexttile(1);
    plot(trajectory.qTime, trajectory.torsoXDeviation, ...
        "LineWidth", 1.2, "Color", colors(index, :), ...
        "DisplayName", label);
    hold on

    nexttile(2);
    plot(trajectory.qTime, ...
        rad2deg(trajectory.torsoLateralTiltDeviation), ...
        "LineWidth", 1.2, "Color", colors(index, :), ...
        "DisplayName", label);
    hold on

    nexttile(3);
    plot(trajectory.qTime, trajectory.torsoXVelocity, ...
        "LineWidth", 1.2, "Color", colors(index, :), ...
        "DisplayName", label);
    hold on

    nexttile(4);
    plot(trajectory.contactTime, ...
        min(trajectory.rightFootLoad, trajectory.leftFootLoad), ...
        "LineWidth", 1.2, "Color", colors(index, :), ...
        "DisplayName", label);
    hold on

    nexttile(5);
    plot(trajectory.pureTime, ...
        vecnorm(trajectory.pureTorque, 2, 2), ...
        "LineWidth", 1.2, "Color", colors(index, :), ...
        "DisplayName", label);
    hold on

    nexttile(6);
    plot(trajectory.forceTime, trajectory.force(:, 1), ...
        "LineWidth", 1.2, "Color", colors(index, :), ...
        "DisplayName", label);
    hold on
end

titles = [ ...
    "Lateral torso displacement"
    "Torso lateral-tilt deviation"
    "Lateral torso velocity"
    "Lower of the two foot loads"
    "Balance-feedback torque norm"
    "Commanded world-x force"];
ylabels = ["m", "deg", "m/s", "N", "N m", "N"];
for tileIndex = 1:6
    axisHandle = nexttile(tileIndex);
    grid(axisHandle, "on");
    title(axisHandle, titles(tileIndex));
    xlabel(axisHandle, "Time (s)");
    ylabel(axisHandle, ylabels(tileIndex));
    xline(axisHandle, pulseStart, "--k", ...
        "HandleVisibility", "off");
    xline(axisHandle, pulseStart + pulseDuration, "--k", ...
        "HandleVisibility", "off");
end
legend(nexttile(1), "Location", "best");
title(layout, "Frozen-controller lateral push calibration");
exportgraphics(figureHandle, ...
    fullfile(outputDirectory, "lateral_push_calibration.png"), ...
    "Resolution", 200);
savefig(figureHandle, ...
    fullfile(outputDirectory, "lateral_push_calibration.fig"));
close(figureHandle);
end
