function results = analyze_benchmark_protocol_sensitivity( ...
        outputRoot, studyMode, residualTorqueLimit_Nm, runPreflight)
%ANALYZE_BENCHMARK_PROTOCOL_SENSITIVITY Test horizon/threshold dependence.
%   This is a protocol-design study, not a controller-tuning experiment.
%   It leaves both SLX files unchanged, verifies that zero-residual A/D in
%   HumanoidModel_ResidualRL reproduce the baseline model, reproduces the
%   frozen 11 s nominal metrics, then runs representative cases once to
%   12 s and reclassifies the same trajectories at 8, 10 and 12 s.
%
%   Threshold sensitivity is one-factor-at-a-time around the declared
%   central recovery envelope. It must not be used to choose thresholds
%   that favour a controller.

arguments
    outputRoot (1,1) string = fullfile( ...
        project_codes_root(), "Results", ...
        "BenchmarkProtocolSensitivity")
    studyMode (1,1) string {mustBeMember(studyMode, ...
        ["full","d_sagittal_extension","rl75_final", ...
        "rl75_authority"])} = "full"
    residualTorqueLimit_Nm (1,1) double ...
        {mustBeFinite,mustBePositive} = 15
    runPreflight (1,1) logical = true
end

codeDirectory = string(project_codes_root());
originalDirectory = string(pwd);
directoryCleanup = onCleanup(@() cd(originalDirectory)); %#ok<NASGU>
cd(codeDirectory);

baselineModel = "HumanoidModel_BaselineBalanceControl";
residualModel = "HumanoidModel_ResidualRL";
baselineFile = fullfile(codeDirectory, "models", baselineModel + ".slx");
residualFile = fullfile(codeDirectory, "models", residualModel + ".slx");
assert(isfile(baselineFile) && isfile(residualFile), ...
    "ProtocolSensitivity:ModelMissing", ...
    "Both baseline and residual-RL model files are required.");

humanoid_walker_parameters;
residual_rl_parameters;
inverseDynamics;
frozenControllers = frozen_pd_gravity_controllers();
[agentObj, agentHash] = loadPolicyWithoutModelHashGate(codeDirectory);
agentObj.UseExplorationPolicy = false;

residualVariableNames = string(who("residualRL*"));
residualVariables = struct();
for index = 1:numel(residualVariableNames)
    name = residualVariableNames(index);
    residualVariables.(name) = eval(name);
end
% Deployment-only authority override. The frozen policy still outputs the
% same normalized actions; only their conversion to physical residual
% torque is changed. This does not retrain or modify the policy.
residualVariables.residualRLTorqueLimit_Nm = residualTorqueLimit_Nm;

protocol = centralProtocol();
if studyMode == "full"
    protocol.horizons_s = [8 10 12];
    protocol.comparisonHorizons_s = [10 12];
    protocol.maximumSimulationTime_s = 12;
elseif studyMode == "d_sagittal_extension"
    protocol.horizons_s = [12 14 16];
    protocol.comparisonHorizons_s = [12 16];
    protocol.maximumSimulationTime_s = 16;
else
    protocol.horizons_s = 12;
    protocol.comparisonHorizons_s = [12 12];
    protocol.maximumSimulationTime_s = 12;
end
protocol.thresholdVariants = thresholdVariants(protocol);
protocol.horizonSelectionRule = ...
    "Choose the shorter comparison horizon if classifications match and threshold-" + ...
    "normalized late-metric change <= 0.10 for every non-fall case; " + ...
    "otherwise choose the longer comparison horizon.";

stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
resultDirectory = fullfile(outputRoot, stamp);
mkdir(resultDirectory);

provenance = struct( ...
    "baselineModelFile", baselineFile, ...
    "baselineModelCurrentSha256", fileHash(baselineFile), ...
    "baselineModelRecordedSha256", ...
        frozenControllers.provenance.modelSha256, ...
    "residualModelFile", residualFile, ...
    "residualModelCurrentSha256", fileHash(residualFile), ...
    "residualModelRecordedSha256", ...
        "27702354E89CA7A87BA809894D74EDD480461586C843F6D7456F3F1053422170", ...
    "agentSha256", agentHash, ...
    "agentExpectedSha256", ...
        "A10111430D4EF578E826BDF116AA8DDFD0D6E2E519027B04F47197B6B1635A43", ...
    "residualTorqueLimit_Nm", residualTorqueLimit_Nm, ...
    "modelsByteIdenticalToRecordedFreeze", false);
provenance.modelsByteIdenticalToRecordedFreeze = ...
    provenance.baselineModelCurrentSha256 == ...
        provenance.baselineModelRecordedSha256 && ...
    provenance.residualModelCurrentSha256 == ...
        provenance.residualModelRecordedSha256;

fprintf("\nBenchmark protocol sensitivity analysis\n");
fprintf("Output: %s\n", resultDirectory);
fprintf("Candidate horizons: %s s\n", mat2str(protocol.horizons_s));
fprintf("Study mode: %s\n", studyMode);
fprintf("Frozen-policy deployment limit: +/-%g Nm\n", ...
    residualTorqueLimit_Nm);
fprintf("Baseline current/recorded model hashes match: %d\n", ...
    provenance.baselineModelCurrentSha256 == ...
    provenance.baselineModelRecordedSha256);
fprintf("Residual current/recorded model hashes match: %d\n", ...
    provenance.residualModelCurrentSha256 == ...
    provenance.residualModelRecordedSha256);
fprintf("The hash mismatch is not waived; numerical gates follow.\n\n");

warningState = warning;
warning("off", "all");
warningCleanup = onCleanup(@() warning(warningState));

if runPreflight
    preflight = runEquivalencePreflight( ...
        baselineModel, residualModel, protocol, agentObj, params, robotFull, ...
        u_shoulder_frontal, residualVariables);
    writetable(preflight, fullfile(resultDirectory, ...
        "model_equivalence_preflight.csv"));
    disp("Model equivalence and frozen-nominal preflight:");
    disp(preflight);
    assert(all(preflight.ModelEquivalent), ...
        "ProtocolSensitivity:ModelEquivalenceFailed", ...
        "Baseline and zero-residual models are not numerically equivalent.");
    assert(all(preflight.FrozenNominalReproduced), ...
        "ProtocolSensitivity:FrozenNominalNotReproduced", ...
        ["Current model does not reproduce the frozen 11 s nominal metrics. " ...
         "Do not use it for final evidence until the difference is resolved."]);
else
    preflight = table();
    fprintf("Model-equivalence preflight reused from the first authority run.\n");
end

definitions = makeDefinitions(studyMode);
trajectories = cell(height(definitions), 1);
nominalByController = struct();
fprintf("\nRepresentative sensitivity cases: %d simulations to %g s\n", ...
    height(definitions), protocol.maximumSimulationTime_s);
for caseIndex = 1:height(definitions)
    definition = definitions(caseIndex, :);
    fprintf("[%d/%d] %s, %s, force %+.6g N\n", ...
        caseIndex, height(definitions), definition.Controller, ...
        definition.Axis, definition.SignedForce_N);
    trajectory = runCase(residualModel, definition.Controller, ...
        definition.Axis, definition.SignedForce_N, ...
        protocol.maximumSimulationTime_s, protocol.pulseStart_s, ...
        protocol.pulseDuration_s, agentObj, params, robotFull, ...
        u_shoulder_frontal, residualVariables);
    trajectories{caseIndex} = trajectory;
    if definition.SignedForce_N == 0
        key = matlab.lang.makeValidName(char(definition.Controller));
        nominalByController.(key) = trajectory;
    end
    partialDefinitions = definitions(1:caseIndex, :); %#ok<NASGU>
    save(fullfile(resultDirectory, "simulation_checkpoint.mat"), ...
        "partialDefinitions", "trajectories", "protocol", ...
        "provenance", "preflight", "-v7.3");
end

[primaryMetrics, sensitivityMetrics] = classifyAll( ...
    definitions, trajectories, nominalByController, protocol);
horizonComparison = compareHorizons(primaryMetrics, protocol);
thresholdSummary = summarizeThresholdSensitivity(sensitivityMetrics);
recommendation = recommendHorizon(horizonComparison);

writetable(definitions, fullfile(resultDirectory, "case_definitions.csv"));
writetable(primaryMetrics, fullfile(resultDirectory, ...
    "horizon_primary_metrics.csv"));
writetable(sensitivityMetrics, fullfile(resultDirectory, ...
    "threshold_sensitivity_metrics.csv"));
writetable(horizonComparison, fullfile(resultDirectory, ...
    "horizon_comparison.csv"));
writetable(thresholdSummary, fullfile(resultDirectory, ...
    "threshold_sensitivity_summary.csv"));
save(fullfile(resultDirectory, "processed_results.mat"), ...
    "definitions", "trajectories", "primaryMetrics", ...
    "sensitivityMetrics", "horizonComparison", "thresholdSummary", ...
    "recommendation", "protocol", "provenance", "preflight", ...
    "-v7.3");
writeProtocol(resultDirectory, protocol, provenance, recommendation);
makePlots(primaryMetrics, sensitivityMetrics, resultDirectory, protocol);

results = struct( ...
    "resultDirectory", string(resultDirectory), ...
    "preflight", preflight, ...
    "definitions", definitions, ...
    "primaryMetrics", primaryMetrics, ...
    "sensitivityMetrics", sensitivityMetrics, ...
    "horizonComparison", horizonComparison, ...
    "thresholdSummary", thresholdSummary, ...
    "recommendation", recommendation, ...
    "protocol", protocol, ...
    "provenance", provenance, ...
    "trajectories", {trajectories});

fprintf("\nHorizon comparison (%g s versus %g s):\n", ...
    protocol.comparisonHorizons_s);
disp(horizonComparison);
fprintf("\nAutomated protocol recommendation: %s\n", recommendation);
fprintf("This recommendation requires trajectory/plot review before freeze.\n");
fprintf("Results written to %s\n", resultDirectory);
clear warningCleanup
open_system(residualModel);
end

function p = centralProtocol()
p = frozen_robustness_protocol();
end

function variants = thresholdVariants(p)
parameter = ["central"; "displacement"; "displacement"; ...
    "tilt"; "tilt"; "velocity"; "velocity"; ...
    "settling_ratio"; "settling_ratio"];
label = ["central"; "displacement_20mm"; "displacement_30mm"; ...
    "tilt_1p5deg"; "tilt_2p5deg"; "velocity_0p015"; ...
    "velocity_0p025"; "ratio_0p70"; "ratio_0p80"];
value = [NaN; 0.020; 0.030; 1.5; 2.5; 0.015; 0.025; 0.70; 0.80];
variants = table((1:numel(label))', label, parameter, value, ...
    'VariableNames', {'VariantIndex','Variant','Parameter','Value'});
variants.Settings = repmat({p}, height(variants), 1);
for index = 2:height(variants)
    settings = p;
    switch variants.Parameter(index)
        case "displacement"
            settings.recoveryDisplacementRMS_m = variants.Value(index);
        case "tilt"
            settings.recoveryTiltRMS_deg = variants.Value(index);
        case "velocity"
            settings.recoveryVelocityRMS_m_s = variants.Value(index);
        case "settling_ratio"
            settings.settlingRatioLimit = variants.Value(index);
    end
    variants.Settings{index} = settings;
end
end

function definitions = makeDefinitions(studyMode)
% Deliberately sample nominal, clearly recoverable, and boundary/failure
% cases already identified in the frozen-controller studies. This gives
% the protocol test cases that can expose a classification change without
% rerunning the entire historical force grid.
if studyMode == "d_sagittal_extension"
    definitions = table((1:3)', ["D";"D";"D"], ...
        ["nominal";"sagittal";"sagittal"], [0;100;-100], ...
        'VariableNames', {'CaseNumber','Controller','Axis','SignedForce_N'});
    return
end
if studyMode == "rl75_final"
    lateral = [25 -25 50 -50 75 -75 100 -100];
    sagittal = [50 -50 75 -75 100 -100];
    axisName = ["nominal"; repmat("lateral",numel(lateral),1); ...
        repmat("sagittal",numel(sagittal),1)];
    force = [0; lateral(:); sagittal(:)];
    definitions = table((1:numel(force))', ...
        repmat("RL75",numel(force),1),axisName,force, ...
        'VariableNames', {'CaseNumber','Controller','Axis','SignedForce_N'});
    return
end
if studyMode == "rl75_authority"
    % Same frozen policy at several deployment bounds. These levels cover
    % A's established 100 N boundary, the original RL comparison at 150 N,
    % and RL75's demonstrated 185 N bidirectional boundary.
    lateral = [100 -100 150 -150 185 -185];
    axisName = ["nominal"; repmat("lateral",numel(lateral),1)];
    force = [0; lateral(:)];
    definitions = table((1:numel(force))', ...
        repmat("RL75",numel(force),1),axisName,force, ...
        'VariableNames', {'CaseNumber','Controller','Axis','SignedForce_N'});
    return
end
controllers = ["A","D","RL75"];
controller = strings(0,1); axisName = strings(0,1); force = zeros(0,1);
for name = controllers
    controller(end+1,1) = name; axisName(end+1,1) = "nominal"; force(end+1,1) = 0; %#ok<AGROW>
    if name == "A"
        lateral = [100 -100 125 -125];
    elseif name == "D"
        lateral = [25 -25 50 -50];
    else
        lateral = [185 -185 190 -190];
    end
    for f = lateral
        controller(end+1,1) = name; axisName(end+1,1) = "lateral"; force(end+1,1) = f; %#ok<AGROW>
    end
    for f = [100 -100]
        controller(end+1,1) = name; axisName(end+1,1) = "sagittal"; force(end+1,1) = f; %#ok<AGROW>
    end
end
definitions = table((1:numel(force))', controller, axisName, force, ...
    'VariableNames', {'CaseNumber','Controller','Axis','SignedForce_N'});
end

function preflight = runEquivalencePreflight( ...
        baselineModel, residualModel, p, agentObj, params, robotFull, ...
        uShoulder, residualVariables)
controllers = ["A"; "D"];
maxQError = zeros(2,1); maxTorqueError = zeros(2,1);
postureRMS = zeros(2,1); appliedRMS = zeros(2,1); appliedPeak = zeros(2,1);
modelEquivalent = false(2,1); frozenNominalReproduced = false(2,1);
expectedPosture = [2.2044; 2.4525];
expectedAppliedRMS = [5.1203; 4.8850];
expectedAppliedPeak = [24.815; 23.679];
for index = 1:2
    name = controllers(index);
    baseline = runCase(baselineModel, name, "nominal", 0, 11, ...
        p.pulseStart_s, p.pulseDuration_s, agentObj, params, robotFull, ...
        uShoulder, residualVariables);
    residual = runCase(residualModel, name, "nominal", 0, 11, ...
        p.pulseStart_s, p.pulseDuration_s, agentObj, params, robotFull, ...
        uShoulder, residualVariables); %#ok<NASGU>
    qAligned = alignToTime(residual.qTime, residual.q, baseline.qTime);
    torqueAligned = alignToTime(residual.torqueTime, ...
        residual.totalTorque, baseline.torqueTime);
    maxQError(index) = max(abs(baseline.q - qAligned), [], "all");
    maxTorqueError(index) = max(abs( ...
        baseline.totalTorque - torqueAligned), [], "all");
    modelEquivalent(index) = maxQError(index) <= 1e-9 && ...
        maxTorqueError(index) <= 1e-8;
    legDeviation = baseline.q(:,11:22) - baseline.q(1,11:22);
    postureRMS(index) = rad2deg(rmsAll(legDeviation));
    appliedRMS(index) = rmsAll(baseline.totalTorque);
    appliedPeak(index) = max(abs(baseline.totalTorque), [], "all");
    frozenNominalReproduced(index) = ...
        abs(postureRMS(index) - expectedPosture(index)) <= 0.02 && ...
        abs(appliedRMS(index) - expectedAppliedRMS(index)) <= 0.02 && ...
        abs(appliedPeak(index) - expectedAppliedPeak(index)) <= 0.05;
end
preflight = table(controllers, maxQError, maxTorqueError, ...
    postureRMS, expectedPosture, appliedRMS, expectedAppliedRMS, ...
    appliedPeak, expectedAppliedPeak, modelEquivalent, ...
    frozenNominalReproduced, ...
    'VariableNames', {'Controller','MaxStateDifferenceBetweenModels', ...
    'MaxTorqueDifferenceBetweenModels_Nm','PostureRMS_deg', ...
    'ExpectedPostureRMS_deg','AppliedTorqueRMS_Nm', ...
    'ExpectedAppliedTorqueRMS_Nm','PeakAppliedTorque_Nm', ...
    'ExpectedPeakAppliedTorque_Nm','ModelEquivalent', ...
    'FrozenNominalReproduced'});
end

function trajectory = runCase(model, controller, axisName, signedForce, ...
        stopTime, pulseStart, pulseDuration, agentObj, params, robotFull, ...
        uShoulder, residualVariables)
load_system(model);
simIn = Simulink.SimulationInput(model);
baseController = controller;
if controller == "RL75"
    baseController = "A";
end
[simIn, ~] = apply_frozen_pd_gravity_controller( ...
    simIn, baseController, model);
simIn = simIn.setVariable("params", params);
simIn = simIn.setVariable("robotFull", robotFull);
simIn = simIn.setVariable("u_shoulder_frontal", uShoulder);
names = string(fieldnames(residualVariables));
for index = 1:numel(names)
    simIn = simIn.setVariable(names(index), residualVariables.(names(index)));
end
forceVector = zeros(3,1);
if axisName == "lateral"
    forceVector(1) = signedForce;
elseif axisName == "sagittal"
    forceVector(2) = signedForce;
end
simIn = simIn.setVariable("robustnessForceVector_N", forceVector);
simIn = simIn.setVariable("robustnessForceStart_s", pulseStart);
simIn = simIn.setVariable("robustnessForceDuration_s", pulseDuration);
simIn = simIn.setVariable("residualRLEnabled", double(controller == "RL75"));
simIn = simIn.setVariable("residualRLUseAgent", double(controller == "RL75"));
simIn = simIn.setVariable("residualRLManualAction", zeros(2,1));
% The RL Agent block evaluates its Agent parameter while the residual model
% compiles, even when residualRLEnabled is zero. Always provide the frozen
% policy to that model; the two enable variables above still guarantee that
% A and D apply exactly zero residual torque.
if model == "HumanoidModel_ResidualRL"
    simIn = simIn.setVariable("agentObj", agentObj);
end
simIn = simIn.setModelParameter( ...
    "StopTime", string(stopTime), ...
    "SignalLogging", "on", "SignalLoggingName", "logsout", ...
    "ReturnWorkspaceOutputs", "on", ...
    "UnconnectedInputMsg", "none", ...
    "UnconnectedOutputMsg", "none", ...
    "UnconnectedLineMsg", "none");
output = sim(simIn);
trajectory = extractTrajectory(output, controller);
end

function trajectory = extractTrajectory(output, controller)
logs = output.logsout;
[qTime,q] = loggedMatrix(logs,"diag_q_rbt","");
[torqueTime,totalTorque] = loggedMatrix( ...
    logs,"diag_tau_leg_command","Final Actuator Torque Saturation");
[pureTime,pureTorque] = loggedMatrix(logs,"Pure PD Torques","");
q = requireWidth(q,22,"diag_q_rbt");
totalTorque = requireWidth(totalTorque,12,"diag_tau_leg_command");
pureTorque = requireWidth(pureTorque,12,"Pure PD Torques");
actionTime=qTime; action=zeros(numel(qTime),2);
residualTime=qTime; residualTorque=zeros(numel(qTime),2);
if controller == "RL75"
    [actionTime,action] = loggedMatrix(logs,"rl_action_normalized","");
    [residualTime,residualTorque] = loggedMatrix( ...
        logs,"rl_residual_hip_torque_Nm","");
    action=requireWidth(action,2,"rl_action_normalized");
    residualTorque=requireWidth(residualTorque,2, ...
        "rl_residual_hip_torque_Nm");
end
trajectory=struct("qTime",qTime,"q",q,"torqueTime",torqueTime, ...
    "totalTorque",totalTorque,"pureTime",pureTime, ...
    "pureTorque",pureTorque,"actionTime",actionTime,"action",action, ...
    "residualTime",residualTime,"residualTorque",residualTorque);
end

function [primary,sensitivity] = classifyAll( ...
        definitions,trajectories,nominalByController,p)
primaryRows={}; sensitivityRows={}; pi=0; si=0;
for caseIndex=1:height(definitions)
    d=definitions(caseIndex,:); x=trajectories{caseIndex};
    key=matlab.lang.makeValidName(char(d.Controller));
    nominal=nominalByController.(key);
    for horizon=p.horizons_s
        central=p.thresholdVariants.Settings{1};
        pi=pi+1; primaryRows{pi,1}=classify( ...
            x,nominal,d,horizon,central,"central"); %#ok<AGROW>
        for variantIndex=1:height(p.thresholdVariants)
            si=si+1; sensitivityRows{si,1}=classify( ...
                x,nominal,d,horizon, ...
                p.thresholdVariants.Settings{variantIndex}, ...
                p.thresholdVariants.Variant(variantIndex)); %#ok<AGROW>
        end
    end
end
primary=vertcat(primaryRows{:}); sensitivity=vertcat(sensitivityRows{:});
end

function row = classify(x,nominal,d,horizon,p,variant)
keep=x.qTime<=horizon+1e-9; t=x.qTime(keep); q=x.q(keep,:);
qNominal=alignToTime(nominal.qTime,nominal.q,t);
residual=q-qNominal;
if d.Axis=="lateral"; axisIndex=1; tiltIndex=5;
elseif d.Axis=="sagittal"; axisIndex=2; tiltIndex=4;
else; axisIndex=1; tiltIndex=5; end
axisResidual=residual(:,axisIndex); tiltResidual=residual(:,tiltIndex);
velocity=gradient(axisResidual,t);
pulseEnd=p.pulseStart_s+p.pulseDuration_s;
early=t>=pulseEnd & t<=pulseEnd+2; late=t>=horizon-p.finalDwell_s;
lateDisplacement=rmsAll(axisResidual(late));
lateTilt=rad2deg(rmsAll(tiltResidual(late)));
lateVelocity=rmsAll(velocity(late));
displacementRatio=safeRatio(lateDisplacement,rmsAll(axisResidual(early)));
tiltRatio=safeRatio(lateTilt,rad2deg(rmsAll(tiltResidual(early))));
qInitial=q(1,:); verticalDrop=max(qInitial(3)-q(:,3));
horizontalTravel=max(vecnorm(q(:,1:2)-qInitial(1:2),2,2));
rotation=max(abs(rad2deg(q(:,4:6)-qInitial(4:6))),[],"all");
fell=verticalDrop>p.fallVerticalDrop_m || ...
    horizontalTravel>p.fallHorizontalTravel_m || rotation>p.fallRotation_deg;
within=lateDisplacement<=p.recoveryDisplacementRMS_m && ...
    lateTilt<=p.recoveryTiltRMS_deg && lateVelocity<=p.recoveryVelocityRMS_m_s;
settled=(displacementRatio<=p.settlingRatioLimit || ...
    lateDisplacement<=p.smallDisplacementRMS_m) && ...
    (tiltRatio<=p.settlingRatioLimit || lateTilt<=p.smallTiltRMS_deg);
recovered=~fell && within && settled && d.SignedForce_N~=0;
if d.SignedForce_N==0
    outcome="nominal_upright"; if fell; outcome="fell"; end
elseif fell; outcome="fell";
elseif recovered; outcome="recovered";
else; outcome="upright_not_recovered"; end
[firstEntryTime,recoveryTime]=recoveryTimesFromEnvelope( ...
    t,axisResidual,tiltResidual,velocity,pulseEnd,p);
if ~recovered
    recoveryTime=NaN;
end
torqueMask=x.torqueTime<=horizon+1e-9;
pureMask=x.pureTime<=horizon+1e-9;
residualMask=x.residualTime<=horizon+1e-9;
actionMask=x.actionTime<=horizon+1e-9;
saturated=any(abs(x.totalTorque(torqueMask,:))>= ...
    100-p.saturationTolerance_Nm,"all");
row=table(d.CaseNumber,d.Controller,d.Axis,d.SignedForce_N,horizon, ...
    string(variant),outcome,fell,recovered,firstEntryTime,recoveryTime, ...
    verticalDrop,horizontalTravel,rotation,max(abs(axisResidual)), ...
    rad2deg(max(abs(tiltResidual))),lateDisplacement,lateTilt, ...
    lateVelocity,displacementRatio,tiltRatio, ...
    rad2deg(rmsAll(residual(:,11:22))), ...
    max(abs(rad2deg(residual(:,11:22))),[],"all"), ...
    rmsAll(x.pureTorque(pureMask,:)), ...
    max(abs(x.pureTorque(pureMask,:)),[],"all"), ...
    rmsAll(x.totalTorque(torqueMask,:)), ...
    max(abs(x.totalTorque(torqueMask,:)),[],"all"), ...
    rmsAll(x.residualTorque(residualMask,:)), ...
    max(abs(x.residualTorque(residualMask,:)),[],"all"), ...
    max(abs(x.action(actionMask,:)),[],"all"),saturated, ...
    'VariableNames',metricNames());
end

function names=metricNames()
names={'CaseNumber','Controller','Axis','SignedForce_N','Horizon_s', ...
    'ThresholdVariant','Outcome','Fell','Recovered', ...
    'FirstEnvelopeEntryTime_s','RecoveryTime_s', ...
    'PeakVerticalDrop_m','PeakHorizontalTravel_m', ...
    'PeakAbsoluteRotationDeviation_deg','PeakAxisDisplacement_m', ...
    'PeakAxisTilt_deg','LateAxisDisplacementRMS_m', ...
    'LateAxisTiltRMS_deg','LateAxisVelocityRMS_m_s', ...
    'AxisDisplacementLateEarlyRatio','AxisTiltLateEarlyRatio', ...
    'ForceInducedPostureRMS_deg','MaxForceInducedJointDeviation_deg', ...
    'PurePDTorqueRMS_Nm','PeakPurePDTorque_Nm', ...
    'AppliedTorqueRMS_Nm','PeakAppliedTorque_Nm', ...
    'ResidualTorqueRMS_Nm','PeakResidualTorque_Nm', ...
    'PeakNormalizedAction','ActuatorSaturation'};
end

function [firstEntry,sustainedRecovery]=recoveryTimesFromEnvelope( ...
        t,x,angle,velocity,pulseEnd,p)
firstEntry=NaN; sustainedRecovery=NaN;
starts=find(t>=pulseEnd & t<=t(end)-p.recoveryTimeDwell_s);
valid=false(size(starts)); completionTime=nan(size(starts));
for position=1:numel(starts)
    index=starts(position);
    finish=find(t>=t(index)+p.recoveryTimeDwell_s,1);
    if isempty(finish); continue; end
    window=index:finish;
    valid(position)=rmsAll(x(window))<=p.recoveryDisplacementRMS_m && ...
        rad2deg(rmsAll(angle(window)))<=p.recoveryTiltRMS_deg && ...
        rmsAll(velocity(window))<=p.recoveryVelocityRMS_m_s;
    completionTime(position)=t(finish)-pulseEnd;
end
firstValid=find(valid,1,"first");
if ~isempty(firstValid); firstEntry=completionTime(firstValid); end
for position=1:numel(valid)
    if valid(position) && all(valid(position:end))
        sustainedRecovery=completionTime(position);
        break
    end
end
end

function comparison=compareHorizons(metrics,p)
shortHorizon=p.comparisonHorizons_s(1);
longHorizon=p.comparisonHorizons_s(2);
m10=metrics(metrics.Horizon_s==shortHorizon,:);
m12=metrics(metrics.Horizon_s==longHorizon,:);
[~,i10,i12]=intersect(m10.CaseNumber,m12.CaseNumber,"stable");
m10=m10(i10,:); m12=m12(i12,:);
classificationMatch=m10.Outcome==m12.Outcome;
normalizedMetricChange=max([ ...
    abs(m10.LateAxisDisplacementRMS_m-m12.LateAxisDisplacementRMS_m) ...
        /p.recoveryDisplacementRMS_m, ...
    abs(m10.LateAxisTiltRMS_deg-m12.LateAxisTiltRMS_deg) ...
        /p.recoveryTiltRMS_deg, ...
    abs(m10.LateAxisVelocityRMS_m_s-m12.LateAxisVelocityRMS_m_s) ...
        /p.recoveryVelocityRMS_m_s],[],2);
metricStable=normalizedMetricChange<=0.10 | m10.Fell | m12.Fell;
comparison=table(m10.CaseNumber,m10.Controller,m10.Axis, ...
    m10.SignedForce_N,m10.Outcome,m12.Outcome,classificationMatch, ...
    normalizedMetricChange,metricStable, ...
    'VariableNames',{'CaseNumber','Controller','Axis','SignedForce_N', ...
    'OutcomeAtShortHorizon','OutcomeAtLongHorizon','ClassificationMatch', ...
    'MaxThresholdNormalizedLateMetricChange','LateMetricsStable'});
comparison.ShortHorizon_s=repmat(shortHorizon,height(comparison),1);
comparison.LongHorizon_s=repmat(longHorizon,height(comparison),1);
end

function summary=summarizeThresholdSensitivity(metrics)
central=metrics(metrics.ThresholdVariant=="central", ...
    {'CaseNumber','Horizon_s','Outcome'});
other=metrics(metrics.ThresholdVariant~="central",:);
keyCentral=central.CaseNumber*100+central.Horizon_s;
keyOther=other.CaseNumber*100+other.Horizon_s;
[found,location]=ismember(keyOther,keyCentral);
assert(all(found),"ProtocolSensitivity:ThresholdJoinFailed", ...
    "Could not match threshold variants to central classifications.");
changed=other.Outcome~=central.Outcome(location);
groups=findgroups(other.Horizon_s,other.ThresholdVariant);
horizon=splitapply(@(x)x(1),other.Horizon_s,groups);
variant=splitapply(@(x)x(1),other.ThresholdVariant,groups);
changedCount=splitapply(@nnz,changed,groups);
caseCount=splitapply(@numel,changed,groups);
summary=table(horizon,variant,changedCount,caseCount, ...
    100*changedCount./caseCount, ...
    'VariableNames',{'Horizon_s','ThresholdVariant', ...
    'ChangedClassificationCount','CaseCount','ChangedCasePercent'});
end

function recommendation=recommendHorizon(comparison)
if all(comparison.ClassificationMatch) && all(comparison.LateMetricsStable)
    recommendation=string(comparison.ShortHorizon_s(1))+" s";
else
    recommendation=string(comparison.LongHorizon_s(1))+" s";
end
end

function makePlots(primary,sensitivity,directory,p)
fig=figure("Visible","off","Color","w","Position",[100 100 1100 520]);
tiledlayout(1,2,"Padding","compact","TileSpacing","compact");
nexttile; hold on;
controllers=unique(primary.Controller,"stable"); colors=lines(numel(controllers));
for index=1:numel(controllers)
    selected=primary.Controller==controllers(index) & ...
        primary.SignedForce_N~=0;
    scatter(primary.Horizon_s(selected), ...
        primary.LateAxisDisplacementRMS_m(selected),45,colors(index,:), ...
        "filled","DisplayName",controllers(index));
end
yline(0.025,"k-","Primary limit"); grid on;
xlabel("Evaluation horizon (s)"); ylabel("Late displacement RMS (m)");
title("Horizon sensitivity"); legend("Location","best");
nexttile;
summary=summarizeThresholdSensitivity(sensitivity);
selectedSummary=summary( ...
    summary.Horizon_s==p.comparisonHorizons_s(1),:);
bar(categorical(selectedSummary.ThresholdVariant), ...
    selectedSummary.ChangedCasePercent);
ylabel("Classifications changed from central (%)");
title(sprintf("One-factor threshold sensitivity at %g s", ...
    p.comparisonHorizons_s(1))); grid on; xtickangle(35);
exportgraphics(fig,fullfile(directory,"protocol_sensitivity_summary.png"), ...
    "Resolution",180); close(fig);
end

function writeProtocol(directory,p,provenance,recommendation)
file=fopen(fullfile(directory,"protocol.txt"),"w");
assert(file>=0,"ProtocolSensitivity:ProtocolWriteFailed", ...
    "Could not write protocol.txt.");
cleanup=onCleanup(@()fclose(file)); %#ok<NASGU>
fprintf(file,"Benchmark protocol sensitivity analysis\n");
fprintf(file,"Candidate horizons: %s s\n",mat2str(p.horizons_s));
fprintf(file,"Pulse: %.17g to %.17g s\n",p.pulseStart_s, ...
    p.pulseStart_s+p.pulseDuration_s);
fprintf(file,"Final dwell: %.17g s\n",p.finalDwell_s);
fprintf(file,"Automated recommendation: %s\n",recommendation);
fprintf(file,"Selection rule: %s\n",p.horizonSelectionRule);
fprintf(file,"Baseline current hash: %s\n", ...
    provenance.baselineModelCurrentSha256);
fprintf(file,"Baseline recorded hash: %s\n", ...
    provenance.baselineModelRecordedSha256);
fprintf(file,"Residual current hash: %s\n", ...
    provenance.residualModelCurrentSha256);
fprintf(file,"Residual recorded hash: %s\n", ...
    provenance.residualModelRecordedSha256);
fprintf(file,"Policy hash: %s\n",provenance.agentSha256);
end

function [agentObj,hash]=loadPolicyWithoutModelHashGate(codeDirectory)
file=fullfile(codeDirectory,"FrozenControllers","ResidualRL75", ...
    "RL75_agent.mat");
assert(isfile(file),"ProtocolSensitivity:PolicyMissing", ...
    "Frozen RL75 policy is missing: %s",file);
hash=fileHash(file);
expected="A10111430D4EF578E826BDF116AA8DDFD0D6E2E519027B04F47197B6B1635A43";
assert(hash==expected,"ProtocolSensitivity:PolicyHashMismatch", ...
    "Frozen RL75 policy hash changed.");
loaded=load(file);
assert(isfield(loaded,"saved_agent"), ...
    "ProtocolSensitivity:PolicyVariableMissing", ...
    "RL75_agent.mat does not contain saved_agent.");
agentObj=loaded.saved_agent;
end

function hash=fileHash(path)
hash=release_file_sha256(path);
end

function output=alignToTime(sourceTime,values,targetTime)
sourceTime=sourceTime(:);
if isequal(sourceTime,targetTime); output=values;
elseif isscalar(sourceTime); output=repmat(values(1,:),numel(targetTime),1);
else; output=interp1(sourceTime,values,targetTime,"linear","extrap"); end
end

function ratio=safeRatio(numerator,denominator)
if denominator<=1e-12; ratio=double(numerator>1e-12)*Inf;
else; ratio=numerator/denominator; end
end

function value=rmsAll(values)
value=sqrt(mean(values.^2,"all"));
end

function values=requireWidth(values,width,signalName)
if size(values,1)==width && size(values,2)~=width; values=values'; end
assert(size(values,2)==width,"ProtocolSensitivity:UnexpectedWidth", ...
    "Signal %s has width %d; expected %d.", ...
    signalName,size(values,2),width);
end

function [time,values]=loggedMatrix(logs,signalName,blockHint)
matches=[];
for index=1:logs.numElements
    candidate=logs.getElement(index);
    if string(candidate.Name)==signalName && ...
            (strlength(blockHint)==0 || ...
            contains(elementBlockPath(candidate),blockHint))
        matches(end+1)=index; %#ok<AGROW>
    end
end
assert(~isempty(matches),"ProtocolSensitivity:SignalMissing", ...
    "Logged signal %s was not found.",signalName);
element=logs.getElement(matches(1)); payload=element;
if isa(element,"Simulink.SimulationData.Signal"); payload=element.Values; end
[time,values]=flattenPayload(payload); time=double(time(:));
values=double(squeeze(values));
if isvector(values)
    if isscalar(time); values=reshape(values,1,[]); else; values=values(:); end
elseif size(values,1)~=numel(time) && size(values,2)==numel(time)
    values=values';
end
end

function path=elementBlockPath(element)
path="";
try; bp=element.BlockPath; path=string(bp.getBlock(bp.getLength)); catch; end
end

function [time,values]=flattenPayload(payload)
if isa(payload,"timeseries"); time=payload.Time; values=squeeze(payload.Data); return; end
if istimetable(payload)
    time=seconds(payload.Properties.RowTimes-payload.Properties.RowTimes(1));
    values=payload.Variables; return
end
if isa(payload,"Simulink.SimulationData.Signal")
    [time,values]=flattenPayload(payload.Values); return
end
if isa(payload,"Simulink.SimulationData.Dataset")
    time=[]; values=[];
    for index=1:payload.numElements
        [itemTime,itemValues]=flattenPayload(payload.getElement(index));
        if isempty(time); time=itemTime(:); values=itemValues;
        else; values=[values,alignToTime(itemTime,itemValues,time)]; end %#ok<AGROW>
    end
    return
end
if isstruct(payload)
    if isfield(payload,"time") && isfield(payload,"signals")
        time=payload.time; values=squeeze(payload.signals.values); return
    end
    fields=fieldnames(payload); time=[]; values=[];
    for index=1:numel(fields)
        [itemTime,itemValues]=flattenPayload(payload.(fields{index}));
        if isempty(time); time=itemTime(:); values=itemValues;
        else; values=[values,alignToTime(itemTime,itemValues,time)]; end %#ok<AGROW>
    end
    return
end
error("ProtocolSensitivity:UnsupportedPayload", ...
    "Unsupported logged payload type: %s",class(payload));
end
