function results = experiment_matched_feedback_gain_authority( ...
    gainScales, nominalFraction, acceptanceMultiplier, stopTime, outputRoot)
%EXPERIMENT_MATCHED_FEEDBACK_GAIN_AUTHORITY Compare A and D gain authority.
%   RESULTS = EXPERIMENT_MATCHED_FEEDBACK_GAIN_AUTHORITY() applies the same
%   uniform scale to every outer torso-error Kp and Kd gain for:
%     A: alpha=0, original nominal commands
%     D: alpha=1, gravity-compatible nominal commands (fraction 1.15)
%
%   Inner joint stiffness, joint damping, torque limits, model settings,
%   initial conditions, and logging are unchanged. The saved model is never
%   modified. This experiment does not retune either controller.
%
%   A common acceptance envelope is fixed from the full-gain A metrics:
%     metric limit = acceptanceMultiplier * full-gain A metric.
%   A case is acceptable only if it reaches StopTime and remains within all
%   four common limits: posture RMS, maximum joint deviation, torso-y RMS,
%   and maximum torso-y deviation. The default multiplier is 1.25.

arguments
    gainScales (1,:) double {mustBeFinite, mustBePositive} = ...
        [1.0 0.9 0.8 0.7 0.6 0.5]
    nominalFraction (1,1) double {mustBeFinite, mustBeNonnegative} = 1.15
    acceptanceMultiplier (1,1) double {mustBeFinite, mustBeGreaterThanOrEqual(acceptanceMultiplier, 1)} = 1.25
    stopTime (1,1) double = NaN
    outputRoot (1,1) string = ""
end

assert(any(abs(gainScales - 1) < 1e-12), ...
    "GainAuthority:FullGainRequired", ...
    "gainScales must include 1.0 to define the common acceptance envelope.");
gainScales = unique(gainScales, "stable");

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
for index = 1:numel(setupVariables)
    setupData.(setupVariables{index}) = eval(setupVariables{index});
end

load_system(model);
% Leave the model open for post-run inspection.
controllerGainPath = model + "/Gain";
gravityGainPath = model + "/Gain2";
if isnan(stopTime)
    stopTime = str2double(get_param(model, "StopTime"));
end
assert(isfinite(stopTime) && stopTime > 0, ...
    "GainAuthority:InvalidStopTime", ...
    "Stop time must be a positive finite scalar.");

if strlength(outputRoot) == 0
    outputRoot = fullfile(thisDirectory, "Results", ...
        "MatchedFeedbackGainAuthority");
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
baseNominal = struct("hip", u_hip_frontal, "knee", u_knee, ...
    "ankle", u_ankle_pitch, "hipSagittal", u_hip_sagittal);

% Use the same initial gravity-based nominal correction as Controller C.
probeOutput = runCase(model, controllerGainPath, gravityGainPath, ...
    setupData, pGainNames, basePGains, dGainNames, baseDGains, ...
    1, baseNominal, 1, min(stopTime, 1e-3));
[~, initialGravity] = loggedMatrix(probeOutput.logsout, ...
    "diag_tau_gravity_scaled", model + "/Gain2");
initialGravity = requireWidth(initialGravity, 12, ...
    "diag_tau_gravity_scaled");
initialGravity = initialGravity(1, :);
fullCorrection = nominalCorrection(initialGravity, params);
retunedNominal = applyCorrection( ...
    baseNominal, fullCorrection, nominalFraction);
validateNominal(retunedNominal);

definitions = table(strings(0, 1), zeros(0, 1), false(0, 1), ...
    zeros(0, 1), zeros(0, 1), ...
    'VariableNames', {'Controller', 'Alpha', 'UsesRetunedNominal', ...
    'NominalFraction', 'GainScale'});
for scale = gainScales
    definitions = [definitions; ...
        table("A", 0, false, 0, scale, ...
        'VariableNames', definitions.Properties.VariableNames); ...
        table("D", 1, true, nominalFraction, scale, ...
        'VariableNames', definitions.Properties.VariableNames)]; %#ok<AGROW>
end

fprintf("\nMatched outer-feedback gain-authority experiment\n");
fprintf("Model: %s\nStop time: %.6g s\n", model, stopTime);
fprintf("Controller D nominal fraction: %.6g\n", nominalFraction);
fprintf("Common acceptance multiplier: %.6g\n", acceptanceMultiplier);
fprintf("Results: %s\n\n", resultDirectory);

nCases = height(definitions);
outputs = cell(nCases, 1);
trajectories = cell(nCases, 1);
metricRows = cell(nCases, 1);
for index = 1:nCases
    definition = definitions(index, :);
    if definition.UsesRetunedNominal
        nominal = retunedNominal;
    else
        nominal = baseNominal;
    end
    fprintf("[%d/%d] Controller %s, gain scale %.3g\n", ...
        index, nCases, definition.Controller, definition.GainScale);
    outputs{index} = runCase(model, controllerGainPath, ...
        gravityGainPath, setupData, pGainNames, basePGains, ...
        dGainNames, baseDGains, definition.Alpha, nominal, ...
        definition.GainScale, stopTime);
    [metricRows{index}, trajectories{index}] = processCase( ...
        outputs{index}, definition, stopTime, params);
end
metrics = vertcat(metricRows{:});

baseline = metrics.Controller == "A" & ...
    abs(metrics.GainScale - 1) < 1e-12;
assert(nnz(baseline) == 1, "GainAuthority:BaselineMissing", ...
    "Exactly one full-gain Controller A case is required.");
limits = struct( ...
    "PostureRMSError_deg", acceptanceMultiplier * ...
        metrics.PostureRMSError_deg(baseline), ...
    "MaxJointDeviation_deg", acceptanceMultiplier * ...
        metrics.MaxJointDeviation_deg(baseline), ...
    "TorsoYRMSError_m", acceptanceMultiplier * ...
        metrics.TorsoYRMSError_m(baseline), ...
    "TorsoYMaxDeviation_m", acceptanceMultiplier * ...
        metrics.TorsoYMaxDeviation_m(baseline));

metrics.Acceptable = metrics.ReachedStopTime & ...
    metrics.PostureRMSError_deg <= limits.PostureRMSError_deg & ...
    metrics.MaxJointDeviation_deg <= limits.MaxJointDeviation_deg & ...
    metrics.TorsoYRMSError_m <= limits.TorsoYRMSError_m & ...
    metrics.TorsoYMaxDeviation_m <= limits.TorsoYMaxDeviation_m;

summary = controllerSummary(metrics);
pairedDifferences = pairedComparison(metrics);

writetable(definitions, fullfile(resultDirectory, ...
    "gain_case_definitions.csv"));
writetable(metrics, fullfile(resultDirectory, ...
    "gain_authority_metrics.csv"));
writetable(summary, fullfile(resultDirectory, ...
    "gain_authority_summary.csv"));
writetable(pairedDifferences, fullfile(resultDirectory, ...
    "paired_D_minus_A.csv"));
save(fullfile(resultDirectory, "processed_results.mat"), ...
    "metrics", "summary", "pairedDifferences", "trajectories", ...
    "definitions", "limits", "acceptanceMultiplier", "gainScales", ...
    "nominalFraction", "baseNominal", "retunedNominal", ...
    "fullCorrection", "initialGravity", "stopTime");
save(fullfile(resultDirectory, "raw_simulation_outputs.mat"), ...
    "outputs", "probeOutput", "-v7.3");
createPlots(metrics, limits, resultDirectory);

results = struct("resultDirectory", string(resultDirectory), ...
    "metrics", metrics, "summary", summary, ...
    "pairedDifferences", pairedDifferences, ...
    "acceptanceLimits", limits, ...
    "acceptanceMultiplier", acceptanceMultiplier, ...
    "gainScales", gainScales, "nominalFraction", nominalFraction, ...
    "baseNominal", baseNominal, "retunedNominal", retunedNominal, ...
    "trajectories", {trajectories});

fprintf("\nCommon acceptance limits:\n");
disp(struct2table(limits));
fprintf("\nGain-authority metrics:\n");
disp(metrics);
fprintf("\nController summary:\n");
disp(summary);
fprintf("Results written to %s\n", resultDirectory);
end

function output = runCase(model, controllerGainPath, gravityGainPath, ...
    setupData, pGainNames, basePGains, dGainNames, baseDGains, ...
    alpha, nominal, gainScale, duration)
simIn = Simulink.SimulationInput(model);
names = fieldnames(setupData);
for index = 1:numel(names)
    simIn = simIn.setVariable(names{index}, setupData.(names{index}));
end
simIn = simIn.setVariable("alpha", alpha);
simIn = simIn.setVariable("robustnessForceVector_N", zeros(3, 1));
simIn = simIn.setVariable("u_hip_frontal", nominal.hip);
simIn = simIn.setVariable("u_knee", nominal.knee);
simIn = simIn.setVariable("u_ankle_pitch", nominal.ankle);
simIn = simIn.setVariable("u_hip_sagittal", nominal.hipSagittal);
for index = 1:numel(pGainNames)
    simIn = simIn.setVariable( ...
        pGainNames(index), basePGains(index) * gainScale);
end
for index = 1:numel(dGainNames)
    simIn = simIn.setVariable( ...
        dGainNames(index), baseDGains(index) * gainScale);
end
simIn = simIn.setBlockParameter(controllerGainPath, "Gain", "1");
simIn = simIn.setBlockParameter(gravityGainPath, "Gain", "alpha");
simIn = simIn.setModelParameter("StopTime", num2str(duration, 17), ...
    "SignalLogging", "on", "SignalLoggingName", "logsout", ...
    "UnconnectedInputMsg", "none", ...
    "UnconnectedOutputMsg", "none", ...
    "UnconnectedLineMsg", "none");
output = sim(simIn);
assert(any(strcmp(output.who, "logsout")), ...
    "GainAuthority:NoLogsout", "Simulation produced no logsout.");
end

function values = captureVariables(names)
values = zeros(size(names));
for index = 1:numel(names)
    values(index) = evalin("caller", names(index));
end
end

function correction = nominalCorrection(gravity, params)
pairs = [1 7; 2 8; 3 9; 4 10];
pairedGravity = [mean(gravity(pairs(1, :))), ...
    mean(gravity(pairs(2, :))), mean(gravity(pairs(3, :))), ...
    mean(gravity(pairs(4, :)))];
ranges = [ ...
    (params.jointLimits.hipFrontalUpperLimit - ...
        params.jointLimits.hipFrontalLowerLimit) / 2, ...
    (params.jointLimits.kneeUpperLimit - ...
        params.jointLimits.kneeLowerLimit) / 2, ...
    (params.jointLimits.ankleUpperLimit - ...
        params.jointLimits.ankleLowerLimit) / 2, ...
    (params.jointLimits.hipSagittalUpperLimit - ...
        params.jointLimits.hipSagittalLowerLimit) / 2];
stiffness = [params.controller.hipFrontalStiffness, ...
    params.controller.kneeStiffness, ...
    params.controller.ankleStiffness, ...
    params.controller.hipSagittalStiffness];
shift = -pairedGravity ./ (stiffness .* deg2rad(ranges));
correction = struct("hip", shift(1), "knee", shift(2), ...
    "ankle", shift(3), "hipSagittal", shift(4));
end

function nominal = applyCorrection(base, correction, fraction)
nominal = struct("hip", base.hip + fraction * correction.hip, ...
    "knee", base.knee + fraction * correction.knee, ...
    "ankle", base.ankle + fraction * correction.ankle, ...
    "hipSagittal", ...
        base.hipSagittal + fraction * correction.hipSagittal);
end

function validateNominal(nominal)
values = [nominal.hip, nominal.knee, nominal.ankle, ...
    nominal.hipSagittal];
assert(all(isfinite(values)) && all(abs(values) <= 1), ...
    "GainAuthority:InvalidNominal", ...
    "Retuned normalized commands must be finite and within [-1,1].");
end

function [row, trajectory] = processCase( ...
    output, definition, stopTime, params)
logs = output.logsout;
[pureTime, pure] = loggedMatrix(logs, "Pure PD Torques", "");
[controllerTime, controller] = loggedMatrix(logs, ...
    "PD+NOM Torques", ...
    "HumanoidModel_BaselineBalanceControl/Joint Control");
[gravityTime, gravity] = loggedMatrix(logs, ...
    "diag_tau_gravity_scaled", ...
    "HumanoidModel_BaselineBalanceControl/Gain2");
[totalTime, total] = loggedMatrix(logs, ...
    "diag_tau_leg_command", ...
    "HumanoidModel_BaselineBalanceControl/Final Actuator Torque Saturation");
[angleTime, angles] = loggedMatrix(logs, ...
    "confirm_joint_angles", "");
[torsoTime, torsoY] = loggedMatrix(logs, "torso_y", "");

pure = requireWidth(pure, 12, "Pure PD Torques");
controller = requireWidth(controller, 12, "PD+NOM Torques");
gravity = requireWidth(gravity, 12, "diag_tau_gravity_scaled");
total = requireWidth(total, 12, "diag_tau_leg_command");
angles = selectLegAngles(angles);
torsoY = torsoY(:, 1);

pure = interpolateMatrix(pureTime, pure, totalTime);
controller = interpolateMatrix(controllerTime, controller, totalTime);
gravity = interpolateMatrix(gravityTime, gravity, totalTime);
angles = interpolateMatrix(angleTime, angles, totalTime);
torsoY = interpolateMatrix(torsoTime, torsoY, totalTime);
angleDeviation = angles - angles(1, :);
torsoDeviation = torsoY - torsoY(1);
unsaturatedTotal = controller + gravity;
actuatorLimit = params.torqueLimits.legActuator;
expectedApplied = min(max( ...
    unsaturatedTotal, -actuatorLimit), actuatorLimit);
closure = total - expectedApplied;
finalTime = totalTime(end);

row = [definition, table(finalTime, ...
    finalTime >= stopTime - max(1e-8, 1e-6 * stopTime), ...
    rmsAll(rad2deg(angleDeviation)), ...
    max(abs(rad2deg(angleDeviation)), [], "all"), ...
    rmsAll(pure), max(abs(pure), [], "all"), ...
    rmsAll(controller), rmsAll(gravity), rmsAll(total), ...
    max(abs(total), [], "all"), absoluteImpulse(totalTime, total), ...
    max(abs(closure), [], "all"), rmsAll(torsoDeviation), ...
    max(abs(torsoDeviation)), ...
    'VariableNames', {'FinalTime_s', 'ReachedStopTime', ...
    'PostureRMSError_deg', 'MaxJointDeviation_deg', ...
    'PurePDTorqueRMS_Nm', 'PeakPurePDTorque_Nm', ...
    'PDNomTorqueRMS_Nm', 'GravityTorqueRMS_Nm', ...
    'TotalTorqueRMS_Nm', 'PeakTotalTorque_Nm', ...
    'AbsTorqueImpulse_Nms', 'TorqueClosureMaxError_Nm', ...
    'TorsoYRMSError_m', 'TorsoYMaxDeviation_m'})];
trajectory = struct("definition", definition, "time", totalTime, ...
    "purePDTorque", pure, "pdNomTorque", controller, ...
    "gravityTorque", gravity, "totalTorque", total, ...
    "jointAngles", angles, "torsoY", torsoY);
end

function summary = controllerSummary(metrics)
controllers = ["A"; "D"];
minimumAcceptable = nan(2, 1);
acceptableCount = zeros(2, 1);
for index = 1:2
    selected = metrics.Controller == controllers(index);
    acceptable = selected & metrics.Acceptable;
    acceptableCount(index) = nnz(acceptable);
    if any(acceptable)
        minimumAcceptable(index) = min(metrics.GainScale(acceptable));
    end
end
summary = table(controllers, minimumAcceptable, acceptableCount, ...
    'VariableNames', {'Controller', 'MinimumAcceptableGainScale', ...
    'AcceptableCaseCount'});
end

function paired = pairedComparison(metrics)
scales = unique(metrics.GainScale, "stable");
names = ["PostureRMSError_deg", "MaxJointDeviation_deg", ...
    "PurePDTorqueRMS_Nm", "PeakPurePDTorque_Nm", ...
    "TotalTorqueRMS_Nm", "PeakTotalTorque_Nm", ...
    "AbsTorqueImpulse_Nms", "TorsoYRMSError_m", ...
    "TorsoYMaxDeviation_m"];
paired = table(scales, 'VariableNames', {'GainScale'});
for name = names
    differences = nan(numel(scales), 1);
    for index = 1:numel(scales)
        a = metrics.Controller == "A" & metrics.GainScale == scales(index);
        d = metrics.Controller == "D" & metrics.GainScale == scales(index);
        assert(nnz(a) == 1 && nnz(d) == 1, ...
            "GainAuthority:UnpairedCases", ...
            "Each gain scale must have exactly one A and one D case.");
        differences(index) = metrics.(name)(d) - metrics.(name)(a);
    end
    paired.("DminusA_" + name) = differences;
end
end

function [time, values] = loggedMatrix(logs, name, expectedBlock)
matches = [];
for index = 1:logs.numElements
    element = logs.getElement(index);
    if strcmp(string(element.Name), name) && ...
            (strlength(expectedBlock) == 0 || ...
            strcmp(elementBlockPath(element), expectedBlock))
        matches(end + 1) = index; %#ok<AGROW>
    end
end
assert(~isempty(matches), "GainAuthority:LogMissing", ...
    "Log '%s' from '%s' was not found.", name, expectedBlock);
element = logs.getElement(matches(1));
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
    assert(~isempty(fields), "GainAuthority:EmptyStruct", ...
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
error("GainAuthority:UnsupportedLog", ...
    "Log '%s' has unsupported type '%s'.", label, class(value));
end

function values = requireWidth(values, width, label)
assert(size(values, 2) == width, "GainAuthority:WrongWidth", ...
    "%s has %d columns; expected %d.", label, size(values, 2), width);
end

function angles = selectLegAngles(angles)
if size(angles, 2) == 22
    angles = angles(:, 11:22);
elseif size(angles, 2) ~= 12
    error("GainAuthority:AngleWidth", ...
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

function value = rmsAll(values)
value = sqrt(mean(values.^2, "all"));
end

function value = absoluteImpulse(time, torque)
if numel(time) < 2
    value = 0;
else
    value = trapz(time, sum(abs(torque), 2));
end
end

function createPlots(metrics, limits, resultDirectory)
figureHandle = figure("Visible", "off", "Color", "white");
cleanup = onCleanup(@() close(figureHandle)); %#ok<NASGU>
tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact");
gainPlot(metrics, "PostureRMSError_deg", ...
    limits.PostureRMSError_deg, "Posture RMS error (deg)");
gainPlot(metrics, "PurePDTorqueRMS_Nm", NaN, ...
    "Pure PD RMS torque (N m)");
gainPlot(metrics, "TotalTorqueRMS_Nm", NaN, ...
    "Total RMS torque (N m)");
gainPlot(metrics, "TorsoYRMSError_m", ...
    limits.TorsoYRMSError_m, "Torso-y RMS error (m)");
exportgraphics(figureHandle, fullfile(resultDirectory, ...
    "matched_gain_authority.png"), "Resolution", 200);
savefig(figureHandle, fullfile(resultDirectory, ...
    "matched_gain_authority.fig"));
end

function gainPlot(metrics, name, limit, yLabel)
nexttile;
hold on;
for controller = ["A", "D"]
    selected = metrics.Controller == controller;
    [scales, order] = sort(metrics.GainScale(selected));
    values = metrics.(name)(selected);
    plot(scales, values(order), "-o", "LineWidth", 1.5, ...
        "DisplayName", "Controller " + controller);
end
if isfinite(limit)
    yline(limit, "--", "Common limit", "HandleVisibility", "off");
end
grid on;
xlabel("Uniform outer Kp/Kd scale");
ylabel(yLabel);
legend("Location", "best");
end
