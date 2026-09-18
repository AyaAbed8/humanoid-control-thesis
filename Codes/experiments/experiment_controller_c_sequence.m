function results = experiment_controller_c_sequence(alpha, nominalFractions, ...
    gainScales, stopTime, outputRoot)
%EXPERIMENT_CONTROLLER_C_SEQUENCE Design and test a PD+gravity controller.
%   RESULTS = EXPERIMENT_CONTROLLER_C_SEQUENCE() performs:
%     1. the frozen alpha=0 PD-only baseline;
%     2. a nominal-command sweep with gravity enabled and original gains;
%     3. an outer Kp/Kd scale sweep at the best nominal candidate.
%
%   Inner joint stiffness and damping are held fixed. The saved Simulink
%   model and parameter script are never changed; all experimental values
%   are supplied through Simulink.SimulationInput.
%
%   Nominal-command correction is based on initial torque matching:
%       delta(theta_ref) = -fraction * alpha * tau_g(q0) / jointStiffness
%   and is applied to the symmetric hip-frontal, knee, ankle-pitch, and
%   hip-sagittal nominal commands. Passive channels remain zero.

arguments
    alpha (1,1) double {mustBeFinite} = 1
    % Baseline revision: restart with a broad sweep because the dedicated
    % symmetric hip-sagittal mapping and its gravity-compatible nominal
    % command invalidate the old optimum near 1.30.
    nominalFractions (1,:) double {mustBeFinite} = ...
        [0 0.25 0.5 0.75 1 1.25 1.5]
    gainScales (:,2) double {mustBeFinite, mustBeNonnegative} = ...
        [1 1; 0.75 0.75; 0.5 0.5]
    stopTime (1,1) double = NaN
    outputRoot (1,1) string = ""
end

assert(alpha > 0, "ControllerC:AlphaMustBePositive", ...
    "Controller C requires a positive gravity-feedforward alpha.");
% Original validation for the initial fractional sweep:
% assert(all(nominalFractions >= 0 & nominalFractions <= 1), ...
%     "ControllerC:InvalidNominalFraction", ...
%     "Nominal correction fractions must lie between zero and one.");
%
% Fractions above one are now intentional over-correction candidates.
% validateNominalCommands still rejects any physical normalized command
% that would leave the permitted controller range [-1,1].
assert(all(nominalFractions >= 0), ...
    "ControllerC:InvalidNominalFraction", ...
    "Nominal correction fractions must be nonnegative.");

model = "HumanoidModel_BaselineBalanceControl";
thisDirectory = string(project_codes_root());
originalDirectory = string(pwd);
directoryCleanup = onCleanup(@() cd(originalDirectory)); %#ok<NASGU>
cd(thisDirectory);

variablesBeforeSetup = who;
humanoid_walker_parameters;
setupVariables = setdiff(who, variablesBeforeSetup);
robotFull = build_manual_humanoid_tree_BFreversed();
setupVariables = [setupVariables; {"robotFull"}];
setupData = struct();
for setupIndex = 1:numel(setupVariables)
    setupName = setupVariables{setupIndex};
    setupData.(setupName) = eval(setupName);
end

load_system(model);
% Leave the model open for post-run inspection.

controllerGainPath = model + "/Gain";
gravityGainPath = model + "/Gain2";
if isnan(stopTime)
    stopTime = str2double(get_param(model, "StopTime"));
end
assert(isfinite(stopTime) && stopTime > 0, ...
    "ControllerC:InvalidStopTime", "Stop time must be positive.");

if strlength(outputRoot) == 0
    outputRoot = fullfile(thisDirectory, "Results", "ControllerCSequence");
end
runStamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
resultDirectory = fullfile(outputRoot, runStamp);
if ~isfolder(resultDirectory)
    mkdir(resultDirectory);
end

pGainNames = ["kp_ankle_qx", "kp_hip_qx", "kp_ankle_y", ...
    "kp_hip_y", "kp_hip_dz", "kp_hip_sag_qy", "kp_hip_sag_x"];
dGainNames = ["kd_ankle_wx", "kd_hip_wx", "kd_ankle_vy", ...
    "kd_hip_vy", "kd_hip_vz", "kd_hip_sag_wy", "kd_hip_sag_vx"];
basePGains = captureVariables(pGainNames);
baseDGains = captureVariables(dGainNames);

baseNominal = struct( ...
    "hip", u_hip_frontal, ...
    "knee", u_knee, ...
    "ankle", u_ankle_pitch, ...
    "hipSagittal", u_hip_sagittal);

fprintf("\nController C sequence\n");
fprintf("Model: %s\nGravity alpha: %.6g\nStop time: %.6g s\n", ...
    model, alpha, stopTime);
fprintf("Results: %s\n\n", resultDirectory);

% Probe only the initial sample. The logged gravity vector is already in
% the controller torque-bus order and includes the validated sign mapping.
probeOutput = runControllerCCase(model, controllerGainPath, ...
    gravityGainPath, setupData, pGainNames, basePGains, ...
    dGainNames, baseDGains, 1, baseNominal, 1, 1, ...
    min(stopTime, 1e-3));
probeLogs = probeOutput.logsout;
[~, initialGravity] = loggedMatrix(probeLogs, ...
    "diag_tau_gravity_scaled", model + "/Gain2");
initialGravity = initialGravity(1, :);
assert(size(initialGravity, 2) == 12, "ControllerC:GravityWidth", ...
    "Expected 12 initial leg gravity torques.");

[nominalDelta, nominalDiagnostics] = nominalCommandCorrection( ...
    initialGravity, alpha, params);

fprintf("Initial gravity torque used for nominal correction (N m):\n");
disp(initialGravity);
fprintf("Full normalized nominal-command correction:\n");
disp(nominalDelta);
if nominalDiagnostics.maxLeftRightDifference_Nm > 1e-5
    warning("ControllerC:AsymmetricGravity", ...
        "Shared left/right nominal commands use the side-average gravity " + ...
        "torque. Maximum left/right difference is %.6g N m.", ...
        nominalDiagnostics.maxLeftRightDifference_Nm);
end

caseDefinitions = table("baseline", 0, 0, 1, 1, ...
    'VariableNames', {'Stage', 'Alpha', 'NominalFraction', ...
    'KpScale', 'KdScale'});
for fraction = nominalFractions
    caseDefinitions = addCase(caseDefinitions, "nominal_sweep", ...
        alpha, fraction, 1, 1);
end

% Remove an accidental duplicate only if the user supplied duplicate
% nominal fractions. The baseline remains separate because alpha differs.
caseDefinitions = unique(caseDefinitions, "rows", "stable");

outputs = cell(height(caseDefinitions), 1);
metricRows = cell(height(caseDefinitions), 1);
trajectories = cell(height(caseDefinitions), 1);

for caseIndex = 1:height(caseDefinitions)
    definition = caseDefinitions(caseIndex, :);
    nominal = applyNominalFraction( ...
        baseNominal, nominalDelta, definition.NominalFraction);
    validateNominalCommands(nominal, definition);
    fprintf("[%d] %s: alpha=%.4g, nominal fraction=%.4g, " + ...
        "Kp scale=%.4g, Kd scale=%.4g\n", ...
        caseIndex, definition.Stage, definition.Alpha, ...
        definition.NominalFraction, definition.KpScale, ...
        definition.KdScale);
    outputs{caseIndex} = runControllerCCase(model, ...
        controllerGainPath, gravityGainPath, setupData, ...
        pGainNames, basePGains, dGainNames, baseDGains, ...
        definition.Alpha, nominal, definition.KpScale, ...
        definition.KdScale, stopTime);
    [metricRows{caseIndex}, trajectories{caseIndex}] = processCase( ...
        outputs{caseIndex}, definition, stopTime, params);
end

metrics = vertcat(metricRows{:});
selectedFraction = selectNominalCandidate(metrics);
fprintf("\nSelected nominal fraction for gain sweep: %.6g\n", ...
    selectedFraction);

% Stage 2 deliberately reuses the selected Kp=Kd=1 result and adds only
% gain pairs that have not already been run.
for scaleIndex = 1:size(gainScales, 1)
    pScale = gainScales(scaleIndex, 1);
    dScale = gainScales(scaleIndex, 2);
    alreadyRun = metrics.Stage == "nominal_sweep" & ...
        metrics.NominalFraction == selectedFraction & ...
        metrics.KpScale == pScale & metrics.KdScale == dScale;
    if any(alreadyRun)
        continue;
    end

    definition = table("gain_sweep", alpha, selectedFraction, ...
        pScale, dScale, 'VariableNames', ...
        {'Stage', 'Alpha', 'NominalFraction', 'KpScale', 'KdScale'});
    nominal = applyNominalFraction( ...
        baseNominal, nominalDelta, selectedFraction);
    validateNominalCommands(nominal, definition);
    fprintf("[gain] alpha=%.4g, nominal fraction=%.4g, " + ...
        "Kp scale=%.4g, Kd scale=%.4g\n", ...
        alpha, selectedFraction, pScale, dScale);
    output = runControllerCCase(model, controllerGainPath, ...
        gravityGainPath, setupData, pGainNames, basePGains, ...
        dGainNames, baseDGains, alpha, nominal, pScale, dScale, stopTime);
    [row, trajectory] = processCase( ...
        output, definition, stopTime, params);
    metrics = [metrics; row]; %#ok<AGROW>
    outputs{end + 1, 1} = output; %#ok<AGROW>
    trajectories{end + 1, 1} = trajectory; %#ok<AGROW>
end

writetable(metrics, fullfile(resultDirectory, "controller_c_metrics.csv"));
writetable(caseDefinitions, ...
    fullfile(resultDirectory, "nominal_sweep_definitions.csv"));
save(fullfile(resultDirectory, "controller_c_processed.mat"), ...
    "metrics", "trajectories", "initialGravity", "nominalDelta", ...
    "nominalDiagnostics", "selectedFraction", "alpha", ...
    "nominalFractions", "gainScales", "stopTime", "baseNominal");
save(fullfile(resultDirectory, "controller_c_raw_outputs.mat"), ...
    "outputs", "probeOutput", "-v7.3");
createPlots(metrics, resultDirectory);

results = struct( ...
    "resultDirectory", string(resultDirectory), ...
    "metrics", metrics, ...
    "selectedNominalFraction", selectedFraction, ...
    "initialGravityTorque_Nm", initialGravity, ...
    "fullNominalCorrection", nominalDelta, ...
    "nominalDiagnostics", nominalDiagnostics, ...
    "trajectories", {trajectories});

fprintf("\nController C metrics:\n");
disp(metrics);
fprintf("Results written to %s\n", resultDirectory);
end

function output = runControllerCCase(model, controllerGainPath, ...
    gravityGainPath, setupData, pGainNames, basePGains, ...
    dGainNames, baseDGains, caseAlpha, nominal, pScale, dScale, duration)
simIn = Simulink.SimulationInput(model);
setupNames = fieldnames(setupData);
for variableIndex = 1:numel(setupNames)
    variableName = setupNames{variableIndex};
    simIn = simIn.setVariable(variableName, setupData.(variableName));
end
simIn = simIn.setVariable("alpha", caseAlpha);
simIn = simIn.setVariable("u_hip_frontal", nominal.hip);
simIn = simIn.setVariable("u_knee", nominal.knee);
simIn = simIn.setVariable("u_ankle_pitch", nominal.ankle);
simIn = simIn.setVariable("u_hip_sagittal", nominal.hipSagittal);
for gainIndex = 1:numel(pGainNames)
    simIn = simIn.setVariable(pGainNames(gainIndex), ...
        basePGains(gainIndex) * pScale);
end
for gainIndex = 1:numel(dGainNames)
    simIn = simIn.setVariable(dGainNames(gainIndex), ...
        baseDGains(gainIndex) * dScale);
end
simIn = simIn.setBlockParameter(controllerGainPath, "Gain", "1");
simIn = simIn.setBlockParameter(gravityGainPath, "Gain", "alpha");
simIn = simIn.setModelParameter( ...
    "StopTime", num2str(duration, 17), ...
    "SignalLogging", "on", "SignalLoggingName", "logsout", ...
    "UnconnectedInputMsg", "none", ...
    "UnconnectedOutputMsg", "none", ...
    "UnconnectedLineMsg", "none");
output = sim(simIn);
assert(any(strcmp(output.who, "logsout")), ...
    "ControllerC:NoLogsout", "Simulation produced no logsout.");
end

function values = captureVariables(names)
values = zeros(size(names));
for index = 1:numel(names)
    values(index) = evalin("caller", names(index));
end
end

function definitions = addCase(definitions, stage, alpha, fraction, p, d)
row = table(string(stage), alpha, fraction, p, d, ...
    'VariableNames', {'Stage', 'Alpha', 'NominalFraction', ...
    'KpScale', 'KdScale'});
definitions = [definitions; row];
end

function [delta, diagnostics] = nominalCommandCorrection(gravity, alpha, params)
% Controller order: RH frontal, RK, RA pitch, RH sagittal, RH transverse,
% RA roll, LH frontal, LK, LA pitch, LH sagittal, LH transverse, LA roll.
pairIndices = [1 7; 2 8; 3 9; 4 10];
pairGravity = [mean(gravity(pairIndices(1, :))), ...
    mean(gravity(pairIndices(2, :))), ...
    mean(gravity(pairIndices(3, :))), ...
    mean(gravity(pairIndices(4, :)))];
sideDifferences = [diff(gravity(pairIndices(1, :))), ...
    diff(gravity(pairIndices(2, :))), ...
    diff(gravity(pairIndices(3, :))), ...
    diff(gravity(pairIndices(4, :)))];

hipRange = (params.jointLimits.hipFrontalUpperLimit - ...
    params.jointLimits.hipFrontalLowerLimit) / 2;
kneeRange = (params.jointLimits.kneeUpperLimit - ...
    params.jointLimits.kneeLowerLimit) / 2;
ankleRange = (params.jointLimits.ankleUpperLimit - ...
    params.jointLimits.ankleLowerLimit) / 2;
hipSagittalRange = (params.jointLimits.hipSagittalUpperLimit - ...
    params.jointLimits.hipSagittalLowerLimit) / 2;
stiffness = [params.controller.hipFrontalStiffness, ...
    params.controller.kneeStiffness, ...
    params.controller.ankleStiffness, ...
    params.controller.hipSagittalStiffness];
rangeRadians = deg2rad([ ...
    hipRange, kneeRange, ankleRange, hipSagittalRange]);

normalizedShift = -alpha .* pairGravity ./ (stiffness .* rangeRadians);
delta = struct("hip", normalizedShift(1), ...
    "knee", normalizedShift(2), "ankle", normalizedShift(3), ...
    "hipSagittal", normalizedShift(4));
diagnostics = struct( ...
    "pairedGravityTorque_Nm", pairGravity, ...
    "leftRightDifference_Nm", sideDifferences, ...
    "maxLeftRightDifference_Nm", max(abs(sideDifferences)), ...
    "unmatchedGravityChannels_Nm", gravity([5 6 11 12]));
end

function nominal = applyNominalFraction(base, delta, fraction)
nominal = struct( ...
    "hip", base.hip + fraction * delta.hip, ...
    "knee", base.knee + fraction * delta.knee, ...
    "ankle", base.ankle + fraction * delta.ankle, ...
    "hipSagittal", ...
        base.hipSagittal + fraction * delta.hipSagittal);
end

function validateNominalCommands(nominal, definition)
values = [nominal.hip, nominal.knee, nominal.ankle, ...
    nominal.hipSagittal];
assert(all(isfinite(values)), "ControllerC:NonfiniteNominal", ...
    "A nominal command is nonfinite.");
assert(all(abs(values) <= 1), "ControllerC:NominalOutsideRange", ...
    "Case %s produces a normalized nominal command outside [-1,1]: " + ...
    "hip=%g, knee=%g, ankle=%g, hipSagittal=%g.", ...
    definition.Stage, values(1), values(2), values(3), values(4));
end

function fraction = selectNominalCandidate(metrics)
nominalRows = metrics.Stage == "nominal_sweep";
assert(any(nominalRows), "ControllerC:NoNominalCandidate", ...
    "No nominal-sweep cases were evaluated.");
candidates = nominalRows & metrics.ReachedStopTime;
if ~any(candidates)
    longestTime = max(metrics.FinalTime_s(nominalRows));
    candidates = nominalRows & metrics.FinalTime_s == longestTime;
    warning("ControllerC:NoNominalCaseReachedStopTime", ...
        "No nominal-sweep case reached the requested stop time. " + ...
        "Selecting by posture error among the longest-running cases.");
end
candidateRows = find(candidates);
[~, localIndex] = min(metrics.PostureRMSError_deg(candidates));
fraction = metrics.NominalFraction(candidateRows(localIndex));
end

function [row, trajectory] = processCase( ...
    output, definition, stopTime, params)
logs = output.logsout;
[pureTime, pureTorque] = loggedMatrix(logs, "Pure PD Torques", "");
[controllerTime, controllerTorque] = loggedMatrix( ...
    logs, "PD+NOM Torques", ...
    "HumanoidModel_BaselineBalanceControl/Joint Control");
[gravityTime, gravityTorque] = loggedMatrix( ...
    logs, "diag_tau_gravity_scaled", ...
    "HumanoidModel_BaselineBalanceControl/Gain2");
[totalTime, totalTorque] = loggedMatrix( ...
    logs, "diag_tau_leg_command", ...
    "HumanoidModel_BaselineBalanceControl/Final Actuator Torque Saturation");
[angleTime, angles] = loggedMatrix( ...
    logs, "confirm_joint_angles", "");
[torsoTime, torsoY] = loggedMatrix(logs, "torso_y", "");

pureTorque = requireWidth(pureTorque, 12, "Pure PD Torques");
controllerTorque = requireWidth( ...
    controllerTorque, 12, "PD+NOM Torques");
gravityTorque = requireWidth( ...
    gravityTorque, 12, "diag_tau_gravity_scaled");
totalTorque = requireWidth(totalTorque, 12, "diag_tau_leg_command");
angles = selectLegAngles(angles);
torsoY = torsoY(:, 1);

pureTorque = interpolateMatrix(pureTime, pureTorque, totalTime);
controllerTorque = interpolateMatrix( ...
    controllerTime, controllerTorque, totalTime);
gravityTorque = interpolateMatrix( ...
    gravityTime, gravityTorque, totalTime);
angleOnTotal = interpolateMatrix(angleTime, angles, totalTime);
torsoOnTotal = interpolateMatrix(torsoTime, torsoY, totalTime);

angleDeviation = angleOnTotal - angleOnTotal(1, :);
torsoDeviation = torsoOnTotal - torsoOnTotal(1);
unsaturatedTotal = controllerTorque + gravityTorque;
actuatorLimit = params.torqueLimits.legActuator;
expectedApplied = min(max( ...
    unsaturatedTotal, -actuatorLimit), actuatorLimit);
closure = totalTorque - expectedApplied;
finalTime = totalTime(end);

row = [definition, table( ...
    finalTime, finalTime >= stopTime - max(1e-8, 1e-6 * stopTime), ...
    rmsAll(rad2deg(angleDeviation)), ...
    max(abs(rad2deg(angleDeviation)), [], "all"), ...
    rmsAll(pureTorque), max(abs(pureTorque), [], "all"), ...
    rmsAll(controllerTorque), rmsAll(gravityTorque), ...
    rmsAll(totalTorque), max(abs(totalTorque), [], "all"), ...
    absoluteImpulse(totalTime, totalTorque), ...
    max(abs(closure), [], "all"), ...
    rmsAll(torsoDeviation), max(abs(torsoDeviation)), ...
    'VariableNames', {'FinalTime_s', 'ReachedStopTime', ...
    'PostureRMSError_deg', 'MaxJointDeviation_deg', ...
    'PurePDTorqueRMS_Nm', 'PeakPurePDTorque_Nm', ...
    'PDNomTorqueRMS_Nm', 'GravityTorqueRMS_Nm', ...
    'TotalTorqueRMS_Nm', 'PeakTotalTorque_Nm', ...
    'AbsTorqueImpulse_Nms', 'TorqueClosureMaxError_Nm', ...
    'TorsoYRMSError_m', 'TorsoYMaxDeviation_m'})];

trajectory = struct("definition", definition, "time", totalTime, ...
    "purePDTorque", pureTorque, "pdNomTorque", controllerTorque, ...
    "gravityTorque", gravityTorque, "totalTorque", totalTorque, ...
    "jointAngles", angleOnTotal, "torsoY", torsoOnTotal);
end

function [time, values] = loggedMatrix(logs, name, expectedBlock)
matching = [];
for index = 1:logs.numElements
    element = logs.getElement(index);
    if strcmp(string(element.Name), name)
        if strlength(expectedBlock) == 0 || ...
                strcmp(elementBlockPath(element), expectedBlock)
            matching(end + 1) = index; %#ok<AGROW>
        end
    end
end
assert(~isempty(matching), "ControllerC:LogMissing", ...
    "Log '%s' from '%s' was not found.", name, expectedBlock);
element = logs.getElement(matching(1));
[time, values] = valueToMatrix(element.Values, name);
end

function path = elementBlockPath(element)
path = "";
try
    blockPath = element.BlockPath;
    path = string(blockPath.getBlock(blockPath.getLength));
catch
end
end

function [time, values] = valueToMatrix(value, label)
if isa(value, "timeseries")
    time = double(value.Time(:));
    values = squeeze(double(value.Data));
    if numel(time) == 1
        values = reshape(values, 1, []);
    elseif isvector(values)
        values = values(:);
    elseif size(values, 1) ~= numel(time) && ...
            size(values, 2) == numel(time)
        values = values.';
    end
    return;
end
if isstruct(value)
    fields = fieldnames(value);
    assert(~isempty(fields), "ControllerC:EmptyStruct", ...
        "Log '%s' is empty.", label);
    first = value.(fields{1});
    time = double(first.Time(:));
    values = zeros(numel(time), numel(fields));
    for index = 1:numel(fields)
        signal = value.(fields{index});
        data = squeeze(double(signal.Data));
        data = data(:);
        values(:, index) = interpolateMatrix( ...
            double(signal.Time(:)), data, time);
    end
    return;
end
error("ControllerC:UnsupportedLog", ...
    "Log '%s' has unsupported type '%s'.", label, class(value));
end

function matrix = requireWidth(matrix, width, label)
assert(size(matrix, 2) == width, "ControllerC:WrongWidth", ...
    "%s has %d columns; expected %d.", label, size(matrix, 2), width);
end

function angles = selectLegAngles(angles)
if size(angles, 2) == 12
    return;
elseif size(angles, 2) == 22
    angles = angles(:, 11:22);
else
    error("ControllerC:AngleWidth", ...
        "Joint-angle log has %d columns; expected 12 or 22.", ...
        size(angles, 2));
end
end

function output = interpolateMatrix(time, values, targetTime)
if isequal(time, targetTime)
    output = values;
elseif numel(time) == 1
    output = repmat(values(1, :), numel(targetTime), 1);
else
    output = interp1(time, values, targetTime, "linear", "extrap");
end
end

function value = rmsAll(matrix)
value = sqrt(mean(matrix.^2, "all"));
end

function value = absoluteImpulse(time, torque)
if numel(time) < 2
    value = 0;
else
    value = trapz(time, sum(abs(torque), 2));
end
end

function createPlots(metrics, resultDirectory)
figureHandle = figure("Visible", "off", "Color", "white");
cleanup = onCleanup(@() close(figureHandle)); %#ok<NASGU>
tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile;
groupedPlot(metrics, "PostureRMSError_deg");
ylabel("Posture RMS error (deg)");
title("Balance performance");

nexttile;
groupedPlot(metrics, "PurePDTorqueRMS_Nm");
ylabel("Pure PD RMS torque (N m)");
title("Outer-feedback effort");

nexttile;
groupedPlot(metrics, "TotalTorqueRMS_Nm");
ylabel("Total RMS torque (N m)");
title("Applied effort");

nexttile;
groupedPlot(metrics, "TorsoYRMSError_m");
ylabel("Torso-y RMS error (m)");
title("Torso regulation");

exportgraphics(figureHandle, ...
    fullfile(resultDirectory, "controller_c_summary.png"), ...
    "Resolution", 200);
savefig(figureHandle, ...
    fullfile(resultDirectory, "controller_c_summary.fig"));
end

function groupedPlot(metrics, variableName)
hold on;
baseline = metrics.Stage == "baseline";
nominal = metrics.Stage == "nominal_sweep";
gain = metrics.Stage == "gain_sweep";
handles = gobjects(0);
labels = strings(0);
if any(baseline)
    handles(end + 1) = plot(metrics.NominalFraction(baseline), ...
        metrics.(variableName)(baseline), "kp", ...
        "MarkerSize", 10, "MarkerFaceColor", "k");
    labels(end + 1) = "PD-only baseline";
end
if any(nominal)
    handles(end + 1) = plot(metrics.NominalFraction(nominal), ...
        metrics.(variableName)(nominal), "-o", "LineWidth", 1.5);
    labels(end + 1) = "Nominal correction";
end
if any(gain)
    handles(end + 1) = scatter(metrics.KpScale(gain), ...
        metrics.(variableName)(gain), ...
        55, metrics.KdScale(gain), "filled");
    labels(end + 1) = "Gain sweep";
end
grid on;
xlabel("Nominal fraction (lines) / Kp scale (scatter)");
legend(handles, labels, "Location", "best");
end
