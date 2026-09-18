function results = evaluate_residual_rl_checkpoints( ...
    trainingDirectory, mode, selectedEpisode, outputRoot, ...
    residualTorqueLimit_Nm)
%EVALUATE_RESIDUAL_RL_CHECKPOINTS Compare trained SAC policies with A.
%   SCREEN evaluates checkpoints 25/50/75/100 for nominal and +/-125 N
%   lateral pushes over 6 s. FORMAL evaluates Controller A and one selected
%   checkpoint for nominal and +/-[100 125 150] N pushes over 11 s. TIE_BREAK
%   applies nominal and +/-150 N over the same 6 s screening horizon to a
%   caller-selected subset when the initial screen does not rank them.
%   AUTHORITY_FORMAL evaluates one selected checkpoint at nominal and
%   +/-[100 150 185] N under the final frozen 12 s protocol. FULL
%   applies the formal grid to every checkpoint and is intentionally not
%   the default because it is computationally expensive.
%
%   Evaluation is deterministic: SAC exploration is disabled. Each policy
%   has its own matched zero-force trajectory. Classification uses the
%   frozen force-benchmark fall and recovery definitions. Models are loaded
%   for simulation but are never saved.

arguments
    trainingDirectory (1,1) string = fullfile( ...
        project_codes_root(), "Results", ...
        "ResidualRLTraining", "20260728_105334")
    mode (1,1) string {mustBeMember(mode, ...
        ["smoke","screen","tie_break","formal", ...
        "authority_formal","full"])} = "screen"
    selectedEpisode (1,:) double {mustBeInteger,mustBePositive} = 100
    outputRoot (1,1) string = fullfile( ...
        project_codes_root(), "Results", ...
        "ResidualRLEvaluation")
    residualTorqueLimit_Nm (1,1) double ...
        {mustBeFinite,mustBePositive} = 15
end

thisDirectory = string(project_codes_root());
cd(thisDirectory);
assert(isfolder(trainingDirectory), ...
    "ResidualRLEvaluation:TrainingDirectoryMissing", ...
    "Training directory does not exist: %s", trainingDirectory);

checkpointEpisodes = [25 50 75 100];
switch mode
    case "smoke"
        evaluatedEpisodes = selectedEpisode;
        forceCases_N = 0;
        stopTime_s = 2.5;
    case "screen"
        evaluatedEpisodes = checkpointEpisodes;
        forceCases_N = [0 125 -125];
        stopTime_s = 6;
    case "tie_break"
        assert(all(ismember(selectedEpisode,checkpointEpisodes)), ...
            "ResidualRLEvaluation:UnknownCheckpoint", ...
            "Every selected episode must be one of %s.", ...
            mat2str(checkpointEpisodes));
        assert(numel(unique(selectedEpisode))==numel(selectedEpisode), ...
            "ResidualRLEvaluation:DuplicateCheckpoint", ...
            "Tie-break checkpoint episodes must be unique.");
        evaluatedEpisodes = selectedEpisode;
        forceCases_N = [0 150 -150];
        stopTime_s = 6;
    case "formal"
        assert(isscalar(selectedEpisode) && ...
            any(selectedEpisode == checkpointEpisodes), ...
            "ResidualRLEvaluation:UnknownCheckpoint", ...
            "selectedEpisode must be one of %s.", ...
            mat2str(checkpointEpisodes));
        evaluatedEpisodes = selectedEpisode;
        forceCases_N = [0 100 -100 125 -125 150 -150];
        stopTime_s = 11;
    case "authority_formal"
        assert(isscalar(selectedEpisode) && ...
            any(selectedEpisode == checkpointEpisodes), ...
            "ResidualRLEvaluation:UnknownCheckpoint", ...
            "selectedEpisode must be one of %s.", ...
            mat2str(checkpointEpisodes));
        evaluatedEpisodes = selectedEpisode;
        forceCases_N = [0 100 -100 150 -150 185 -185];
        stopTime_s = 12;
    otherwise
        evaluatedEpisodes = checkpointEpisodes;
        forceCases_N = [0 100 -100 125 -125 150 -150];
        stopTime_s = 11;
end

if mode == "authority_formal"
    protocol = frozen_robustness_protocol();
else
    protocol = evaluationProtocol(stopTime_s);
end
runStamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
resultDirectory = fullfile(outputRoot, mode + "_" + runStamp);
mkdir(resultDirectory);

humanoid_walker_parameters;
residual_rl_parameters;
inverseDynamics;
residualVariableNames = string(who("residualRL*"));
residualVariables = struct();
for variableIndex = 1:numel(residualVariableNames)
    variableName = residualVariableNames(variableIndex);
    residualVariables.(variableName) = eval(variableName);
end
% Preserve the physical action mapping used during the evaluated training
% run. Without this explicit override, residual_rl_parameters would deploy
% every checkpoint with its historical default of +/-15 Nm.
residualVariables.residualRLTorqueLimit_Nm = residualTorqueLimit_Nm;

definitions = makeDefinitions(evaluatedEpisodes, forceCases_N, ...
    trainingDirectory);
fprintf("\nResidual-RL checkpoint evaluation\n");
fprintf("Mode: %s; stop time: %.6g s\n", mode, stopTime_s);
fprintf("Training evidence: %s\n", trainingDirectory);
fprintf("Residual torque limit: +/-%g Nm\n",residualTorqueLimit_Nm);
fprintf("Cases: %d simulations\n", height(definitions));
fprintf("Output: %s\n\n", resultDirectory);

trajectories = cell(height(definitions), 1);
metricRows = cell(height(definitions), 1);
nominalByController = struct();
warningState = warning;
warning("off", "all");
warningCleanup = onCleanup(@() warning(warningState));

for caseIndex = 1:height(definitions)
    definition = definitions(caseIndex, :);
    fprintf("[%d/%d] %s, force %+.6g N\n", ...
        caseIndex, height(definitions), definition.Controller, ...
        definition.SignedForce_N);
    agentObj = [];
    if definition.Controller ~= "A"
        agentObj = loadCheckpointAgent(definition.CheckpointFile);
        agentObj.UseExplorationPolicy = false;
    end
    trajectory = runEvaluationCase(definition, protocol, agentObj, ...
        params, robotFull, u_shoulder_frontal, ...
        residualVariables);
    trajectories{caseIndex} = trajectory;

    controllerKey = matlab.lang.makeValidName( ...
        char(definition.Controller));
    if definition.SignedForce_N == 0
        nominalByController.(controllerKey) = trajectory;
        metricRows{caseIndex} = classifyNominal( ...
            trajectory, definition, protocol);
    else
        assert(isfield(nominalByController, controllerKey), ...
            "ResidualRLEvaluation:NominalOrdering", ...
            "Matched nominal trajectory must precede disturbed cases.");
        metricRows{caseIndex} = classifyDisturbed(trajectory, ...
            nominalByController.(controllerKey), definition, protocol);
    end

    partialMetrics = vertcat(metricRows{1:caseIndex});
    save(fullfile(resultDirectory, "evaluation_checkpoint.mat"), ...
        "partialMetrics", "definitions", "protocol", ...
        "trainingDirectory", "mode", "selectedEpisode", ...
        "residualTorqueLimit_Nm");
    writetable(partialMetrics, fullfile( ...
        resultDirectory, "evaluation_metrics_partial.csv"));
end

metrics = vertcat(metricRows{:});
summary = makeSummary(metrics);
writetable(metrics, fullfile(resultDirectory, "evaluation_metrics.csv"));
writetable(summary, fullfile(resultDirectory, "evaluation_summary.csv"));
save(fullfile(resultDirectory, "processed_results.mat"), ...
    "metrics", "summary", "definitions", "protocol", ...
    "trainingDirectory", "mode", "selectedEpisode", ...
    "residualTorqueLimit_Nm", ...
    "trajectories", "-v7.3");
makePlots(trajectories, definitions, resultDirectory);

results = struct( ...
    "resultDirectory", string(resultDirectory), ...
    "metrics", metrics, ...
    "summary", summary, ...
    "definitions", definitions, ...
    "protocol", protocol, ...
    "residualTorqueLimit_Nm", residualTorqueLimit_Nm, ...
    "trajectories", {trajectories});

fprintf("\nEvaluation metrics:\n");
disp(metrics(:, ["Controller","SignedForce_N","Outcome", ...
    "PeakForceInducedAxisDisplacement_m", ...
    "PeakForceInducedAxisTilt_deg", ...
    "LateForceInducedAxisDisplacementRMS_m", ...
    "LateForceInducedAxisTiltRMS_deg", ...
    "ResidualTorqueRMS_Nm","PeakResidualTorque_Nm", ...
    "PeakNormalizedAction","ActuatorSaturation"]));
fprintf("\nController summary:\n");
disp(summary);
fprintf("Results written to %s\n", resultDirectory);
end

function protocol = evaluationProtocol(stopTime_s)
protocol = struct( ...
    "pulseStart_s", 2, ...
    "pulseDuration_s", 0.1, ...
    "stopTime_s", stopTime_s, ...
    "finalDwell_s", min(2, stopTime_s - 2.1), ...
    "fallVerticalDrop_m", 0.5, ...
    "fallHorizontalTravel_m", 1, ...
    "fallRotation_deg", 30, ...
    "contactLoadThreshold_N", 1, ...
    "saturationTolerance_Nm", 1e-6, ...
    "recoveryDisplacementRMS_m", 0.025, ...
    "recoveryTiltRMS_deg", 2, ...
    "recoveryVelocityRMS_m_s", 0.02, ...
    "settlingRatioLimit", 0.75, ...
    "smallDisplacementRMS_m", 0.005, ...
    "smallTiltRMS_deg", 0.5);
end

function definitions = makeDefinitions(episodes, forces, trainingDirectory)
controllers = "A";
checkpointEpisodes = NaN;
checkpointFiles = "";
for episode = episodes
    controllers(end + 1, 1) = "RL" + episode; %#ok<AGROW>
    checkpointEpisodes(end + 1, 1) = episode; %#ok<AGROW>
    checkpointFiles(end + 1, 1) = fullfile(trainingDirectory, ...
        "checkpoints", "Agent" + episode + ".mat"); %#ok<AGROW>
end

controllerColumn = strings(0, 1);
episodeColumn = zeros(0, 1);
fileColumn = strings(0, 1);
forceColumn = zeros(0, 1);
for index = 1:numel(controllers)
    assert(index == 1 || isfile(checkpointFiles(index)), ...
        "ResidualRLEvaluation:CheckpointMissing", ...
        "Checkpoint not found: %s", checkpointFiles(index));
    for force = forces
        controllerColumn(end + 1, 1) = controllers(index); %#ok<AGROW>
        episodeColumn(end + 1, 1) = checkpointEpisodes(index); %#ok<AGROW>
        fileColumn(end + 1, 1) = checkpointFiles(index); %#ok<AGROW>
        forceColumn(end + 1, 1) = force; %#ok<AGROW>
    end
end
definitions = table((1:numel(forceColumn))', controllerColumn, ...
    episodeColumn, fileColumn, forceColumn, ...
    'VariableNames', {'CaseNumber','Controller','CheckpointEpisode', ...
    'CheckpointFile','SignedForce_N'});
end

function agentObj = loadCheckpointAgent(checkpointFile)
loaded = load(checkpointFile, "saved_agent");
assert(isfield(loaded, "saved_agent") && ...
    isa(loaded.saved_agent, "rl.agent.rlSACAgent"), ...
    "ResidualRLEvaluation:InvalidCheckpoint", ...
    "Checkpoint does not contain saved_agent: %s", checkpointFile);
agentObj = loaded.saved_agent;
end

function trajectory = runEvaluationCase(definition, p, agentObj, ...
        params, robotFull, uShoulder, residualVariables)
baselineModel = "HumanoidModel_BaselineBalanceControl";
rlModel = "HumanoidModel_ResidualRL";
if definition.Controller == "A"
    model = baselineModel;
else
    model = rlModel;
end
load_system(model);
modelCleanup = onCleanup(@() closeIfLoaded(model));
contactLoggingCleanup = enableContactLoggingInMemory(model); %#ok<NASGU>

simIn = Simulink.SimulationInput(model);
[simIn, ~] = apply_frozen_pd_gravity_controller(simIn, "A", model);
simIn = simIn.setVariable("params", params);
simIn = simIn.setVariable("robotFull", robotFull);
simIn = simIn.setVariable("u_shoulder_frontal", uShoulder);
simIn = simIn.setVariable( ...
    "robustnessForceVector_N", [definition.SignedForce_N; 0; 0]);
simIn = simIn.setVariable("robustnessForceStart_s", p.pulseStart_s);
simIn = simIn.setVariable( ...
    "robustnessForceDuration_s", p.pulseDuration_s);
if definition.Controller ~= "A"
    variableNames = string(fieldnames(residualVariables));
    for variableIndex = 1:numel(variableNames)
        variableName = variableNames(variableIndex);
        simIn = simIn.setVariable(variableName, ...
            residualVariables.(variableName));
    end
    simIn = simIn.setVariable("agentObj", agentObj);
    simIn = simIn.setVariable("residualRLEnabled", 1);
    simIn = simIn.setVariable("residualRLUseAgent", 1);
end
simIn = simIn.setModelParameter( ...
    "StopTime", string(p.stopTime_s), ...
    "SignalLogging", "on", ...
    "SignalLoggingName", "logsout", ...
    "ReturnWorkspaceOutputs", "on");
output = sim(simIn);
trajectory = extractTrajectory(output, definition);
close_system(model, 0);
end

function closeIfLoaded(model)
if getSimulinkBlockHandle(model) >= 0
    close_system(model, 0);
end
end

function cleanup = enableContactLoggingInMemory(model)
contactLines = find_system(model, "FindAll", "on", ...
    "Type", "line", "Name", "ContactSensing");
sourcePorts = unique(arrayfun(@(lineHandle) ...
    get_param(lineHandle, "SrcPortHandle"), contactLines));
assert(isscalar(sourcePorts), ...
    "ResidualRLEvaluation:ContactSignalNotUnique", ...
    "Expected ContactSensing branches to share one source in %s.", model);
sourcePort = sourcePorts(1);
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
if getSimulinkBlockHandle(model) < 0
    return
end
set_param(sourcePort, ...
    "DataLogging", logging, ...
    "DataLoggingNameMode", nameMode, ...
    "DataLoggingName", name);
if strcmp(originalDirty, "off")
    set_param(model, "Dirty", "off");
end
end

function trajectory = extractTrajectory(output, definition)
assert(any(strcmp(output.who, "logsout")), ...
    "ResidualRLEvaluation:LogsMissing", ...
    "Case %d produced no logsout.", definition.CaseNumber);
logs = output.logsout;
[qTime, q] = loggedMatrix(logs, "diag_q_rbt", "");
[pureTime, pureTorque] = loggedMatrix(logs, "Pure PD Torques", "");
[torqueTime, totalTorque] = loggedMatrix(logs, ...
    "diag_tau_leg_command", "Final Actuator Torque Saturation");
[contactTime, contact] = loggedMatrix(logs, ...
    "calibration_contact_normal_forces_N", "");
q = requireWidth(q, 22, "diag_q_rbt");
pureTorque = requireWidth(pureTorque, 12, "Pure PD Torques");
totalTorque = requireWidth(totalTorque, 12, "diag_tau_leg_command");
contact = requireWidth(contact, 12, ...
    "calibration_contact_normal_forces_N");

trajectory = struct( ...
    "qTime", qTime, "q", q, ...
    "pureTime", pureTime, "pureTorque", pureTorque, ...
    "torqueTime", torqueTime, "totalTorque", totalTorque, ...
    "contactTime", contactTime, ...
    "rightFootLoad", sum(max(contact(:, 1:6), 0), 2), ...
    "leftFootLoad", sum(max(contact(:, 7:12), 0), 2));

if definition.Controller == "A"
    trajectory.actionTime = qTime;
    trajectory.action = zeros(numel(qTime), 2);
    trajectory.residualTime = qTime;
    trajectory.residualTorque = zeros(numel(qTime), 2);
else
    [actionTime, action] = loggedMatrix( ...
        logs, "rl_action_normalized", "");
    [residualTime, residualTorque] = loggedMatrix( ...
        logs, "rl_residual_hip_torque_Nm", "");
    trajectory.actionTime = actionTime;
    trajectory.action = requireWidth( ...
        action, 2, "rl_action_normalized");
    trajectory.residualTime = residualTime;
    trajectory.residualTorque = requireWidth( ...
        residualTorque, 2, "rl_residual_hip_torque_Nm");
end
end

function row = classifyNominal(trajectory, definition, p)
q = trajectory.q;
qInitial = q(1, :);
[fell, fallVertical, fallTravel, fallRotation, ...
    verticalDrop, horizontalTravel, rotationDeviation] = ...
    fallMetrics(q, qInitial, p);
if fell
    outcome = "fell";
else
    outcome = "nominal_upright";
end
late = trajectory.qTime >= p.stopTime_s - p.finalDwell_s;
legDeviation = q(:, 11:22) - qInitial(11:22);
nominalLateralVelocity = numericalDerivative( ...
    trajectory.qTime, q(:, 1) - qInitial(1));
row = commonMetricRow(definition, outcome, fell, false, ...
    fallVertical, fallTravel, fallRotation, verticalDrop, ...
    horizontalTravel, rotationDeviation, 0, 0, ...
    rmsAll(q(late, 1) - qInitial(1)), ...
    rad2deg(rmsAll(q(late, 5) - qInitial(5))), ...
    rmsAll(nominalLateralVelocity(late)), 0, 0, ...
    rmsAll(rad2deg(legDeviation)), ...
    max(abs(rad2deg(legDeviation)), [], "all"), ...
    trajectory, p);
end

function row = classifyDisturbed(disturbed, nominal, definition, p)
t = disturbed.qTime(:);
q = disturbed.q;
qNominal = interpolateMatrix(nominal.qTime, nominal.q, t);
qResidual = q - qNominal;
axisResidual = qResidual(:, 1);
tiltResidual = qResidual(:, 5);
legResidual = qResidual(:, 11:22);
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

row = commonMetricRow(definition, outcome, fell, recovered, ...
    fallVertical, fallTravel, fallRotation, verticalDrop, ...
    horizontalTravel, rotationDeviation, ...
    max(abs(axisResidual)), rad2deg(max(abs(tiltResidual))), ...
    lateDisplacement, lateTilt, lateVelocity, ...
    displacementRatio, tiltRatio, ...
    rmsAll(rad2deg(legResidual)), ...
    max(abs(rad2deg(legResidual)), [], "all"), ...
    disturbed, p);
end

function row = commonMetricRow(definition, outcome, fell, recovered, ...
        fallVertical, fallTravel, fallRotation, verticalDrop, ...
        horizontalTravel, rotationDeviation, peakAxis, peakTilt, ...
        lateDisplacement, lateTilt, lateVelocity, ...
        displacementRatio, tiltRatio, postureRMS, maxJoint, trajectory, p)
postContact = trajectory.contactTime >= ...
    p.pulseStart_s + p.pulseDuration_s;
bothUnloaded = trajectory.rightFootLoad(:) <= ...
    p.contactLoadThreshold_N & trajectory.leftFootLoad(:) <= ...
    p.contactLoadThreshold_N & postContact;
saturated = any(abs(trajectory.totalTorque) >= ...
    100 - p.saturationTolerance_Nm, 2);

row = table(definition.Controller, definition.CheckpointEpisode, ...
    definition.SignedForce_N, ...
    abs(definition.SignedForce_N) * p.pulseDuration_s, ...
    string(outcome), fell, recovered, ...
    fallVertical, fallTravel, fallRotation, ...
    verticalDrop, horizontalTravel, rotationDeviation, ...
    any(bothUnloaded), maskDuration( ...
        trajectory.contactTime, bothUnloaded), ...
    any(saturated), maskDuration(trajectory.torqueTime, saturated), ...
    peakAxis, peakTilt, lateDisplacement, lateTilt, lateVelocity, ...
    displacementRatio, tiltRatio, postureRMS, maxJoint, ...
    rmsAll(trajectory.pureTorque), ...
    max(abs(trajectory.pureTorque), [], "all"), ...
    rmsAll(trajectory.totalTorque), ...
    max(abs(trajectory.totalTorque), [], "all"), ...
    rmsAll(trajectory.residualTorque), ...
    max(abs(trajectory.residualTorque), [], "all"), ...
    max(abs(trajectory.action), [], "all"), ...
    'VariableNames', {'Controller','CheckpointEpisode', ...
    'SignedForce_N','Impulse_Ns','Outcome','Fell','Recovered', ...
    'FallByVerticalDrop','FallByHorizontalTravel','FallByRotation', ...
    'PeakVerticalDrop_m','PeakHorizontalTravel_m', ...
    'PeakAbsoluteRotationDeviation_deg','ContactLoss', ...
    'ContactLossDuration_s','ActuatorSaturation', ...
    'ActuatorSaturationDuration_s', ...
    'PeakForceInducedAxisDisplacement_m', ...
    'PeakForceInducedAxisTilt_deg', ...
    'LateForceInducedAxisDisplacementRMS_m', ...
    'LateForceInducedAxisTiltRMS_deg', ...
    'LateForceInducedAxisVelocityRMS_m_s', ...
    'AxisDisplacementLateEarlyRatio','AxisTiltLateEarlyRatio', ...
    'ForceInducedPostureRMS_deg', ...
    'MaxForceInducedJointDeviation_deg','PurePDTorqueRMS_Nm', ...
    'PeakPurePDTorque_Nm','AppliedTorqueRMS_Nm', ...
    'PeakAppliedTorque_Nm','ResidualTorqueRMS_Nm', ...
    'PeakResidualTorque_Nm','PeakNormalizedAction'});
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

function summary = makeSummary(metrics)
controllers = unique(metrics.Controller, "stable");
recovered = zeros(numel(controllers), 1);
upright = zeros(numel(controllers), 1);
falls = zeros(numel(controllers), 1);
nominalFalls = false(numel(controllers), 1);
for index = 1:numel(controllers)
    selected = metrics.Controller == controllers(index);
    disturbed = selected & metrics.SignedForce_N ~= 0;
    recovered(index) = nnz(metrics.Outcome(disturbed) == "recovered");
    upright(index) = nnz( ...
        metrics.Outcome(disturbed) == "upright_not_recovered");
    falls(index) = nnz(metrics.Outcome(disturbed) == "fell");
    nominalFalls(index) = any(metrics.Fell( ...
        selected & metrics.SignedForce_N == 0));
end
summary = table(controllers, nominalFalls, recovered, upright, falls, ...
    'VariableNames', {'Controller','NominalFall', ...
    'RecoveredDisturbedCases','UprightNotRecoveredCases','FallCases'});
end

function makePlots(trajectories, definitions, resultDirectory)
colors = lines(numel(unique(definitions.Controller, "stable")));
controllers = unique(definitions.Controller, "stable");
forces = unique(definitions.SignedForce_N, "stable");
for force = forces'
    figureHandle = figure("Visible", "off", ...
        "Color", "white", "Position", [100 100 1100 760]);
    tiledlayout(3, 1, "TileSpacing", "compact");
    for controllerIndex = 1:numel(controllers)
        selected = find(definitions.Controller == ...
            controllers(controllerIndex) & ...
            definitions.SignedForce_N == force, 1);
        if isempty(selected)
            continue
        end
        trajectory = trajectories{selected};
        nominalIndex = find(definitions.Controller == ...
            controllers(controllerIndex) & ...
            definitions.SignedForce_N == 0, 1);
        nominal = trajectories{nominalIndex};
        t = trajectory.qTime;
        nominalQ = interpolateMatrix( ...
            nominal.qTime, nominal.q, t);
        nexttile(1);
        plot(t, trajectory.q(:, 1) - nominalQ(:, 1), ...
            "LineWidth", 1.2, "Color", colors(controllerIndex, :));
        hold on;
        nexttile(2);
        plot(t, rad2deg(trajectory.q(:, 5) - nominalQ(:, 5)), ...
            "LineWidth", 1.2, "Color", colors(controllerIndex, :));
        hold on;
        nexttile(3);
        plot(trajectory.actionTime, ...
            vecnorm(trajectory.action, 2, 2), ...
            "LineWidth", 1.2, "Color", colors(controllerIndex, :));
        hold on;
    end
    nexttile(1); ylabel("Lateral residual (m)"); grid on;
    title(sprintf("Lateral force: %+.6g N", force));
    legend(controllers, "Location", "best");
    nexttile(2); ylabel("Tilt residual (deg)"); grid on;
    nexttile(3); ylabel("Action norm"); xlabel("Time (s)"); grid on;
    exportgraphics(figureHandle, fullfile(resultDirectory, ...
        sprintf("trajectory_force_%+gN.png", force)), ...
        "Resolution", 180);
    close(figureHandle);
end
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
    "ResidualRLEvaluation:UnexpectedWidth", ...
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
    "ResidualRLEvaluation:SignalMissing", ...
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
    "ResidualRLEvaluation:SignalShape", ...
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
error("ResidualRLEvaluation:UnsupportedPayload", ...
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
