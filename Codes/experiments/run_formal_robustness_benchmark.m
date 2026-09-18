function results = run_formal_robustness_benchmark(outputRoot)
%RUN_FORMAL_ROBUSTNESS_BENCHMARK Frozen A/D signed-force comparison.
%   Runs the calibrated lateral and sagittal force grids using identical
%   12 s simulations and 100 ms pulses beginning at 2 s. Disturbed motion
%   is evaluated relative to the matched zero-force trajectory.
%
%   Outcomes are:
%     recovered              upright and inside the sustained envelope;
%     upright_not_recovered  no fall, but recovery conditions not met;
%     fell                   a project-native fall limit was exceeded.
%
%   Contact loss and actuator saturation are independent flags: a robot
%   can briefly lose contact and later recover without being labelled a
%   fall. The Simulink model and frozen controllers are not modified.

arguments
    outputRoot (1,1) string = ""
end

thisDirectory = string(project_codes_root());
if strlength(outputRoot) == 0
    outputRoot = fullfile(thisDirectory, "Results", ...
        "FormalFrozenControllerRobustness");
end
runStamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
resultDirectory = fullfile(outputRoot, runStamp);
mkdir(resultDirectory);

protocol = frozen_robustness_protocol();
% Recovery is assessed over the final 2 s, not at one endpoint. The
% 25 mm, 2 deg and 0.02 m/s limits define a small standing envelope.
% A late/early ratio <= 0.75 additionally requires meaningful decay.
% The 5 mm and 0.5 deg floors prevent ratios from declaring failure when
% both the early and late residuals are already practically small.

fprintf("\nFormal frozen-controller robustness benchmark\n");
fprintf("Output: %s\n", resultDirectory);
fprintf("Lateral levels: %s N\n", mat2str(protocol.lateralForceLevels_N));
fprintf("Sagittal levels: %s N\n\n", ...
    mat2str(protocol.sagittalForceLevels_N));

rows = {};
evidence = {};
rowIndex = 0;
for force = protocol.lateralForceLevels_N
    batchRoot = fullfile(resultDirectory, "lateral_" + force + "N");
    batch = calibrate_lateral_push_response(force, ...
        protocol.stopTime_s, batchRoot, protocol.pulseStart_s, ...
        protocol.pulseDuration_s);
    evidence{end + 1, 1} = batch; %#ok<AGROW>
    [rows, rowIndex] = appendBatch(rows, rowIndex, batch, ...
        "lateral", force, protocol);
end
for force = protocol.sagittalForceLevels_N
    batchRoot = fullfile(resultDirectory, "sagittal_" + force + "N");
    batch = calibrate_sagittal_push_response(force, ...
        protocol.stopTime_s, batchRoot, protocol.pulseStart_s, ...
        protocol.pulseDuration_s);
    evidence{end + 1, 1} = batch; %#ok<AGROW>
    [rows, rowIndex] = appendBatch(rows, rowIndex, batch, ...
        "sagittal", force, protocol);
end

metrics = vertcat(rows{:});
summary = makeSummary(metrics);
writetable(metrics, fullfile(resultDirectory, "robustness_metrics.csv"));
writetable(summary, fullfile(resultDirectory, "robustness_summary.csv"));
save(fullfile(resultDirectory, "processed_results.mat"), ...
    "metrics", "summary", "protocol", "evidence");

results = struct("resultDirectory", string(resultDirectory), ...
    "metrics", metrics, "summary", summary, "protocol", protocol, ...
    "evidence", {evidence});
fprintf("\nFormal robustness metrics:\n");
disp(metrics);
fprintf("\nController/axis summary:\n");
disp(summary);
fprintf("Results written to %s\n", resultDirectory);
end

function [rows, rowIndex] = appendBatch( ...
        rows, rowIndex, batch, axisName, force, protocol)
% Each calibration batch is ordered A nominal, A +, A -, D nominal, D +,
% D -. Use the matched nominal trajectory from the same batch.
for item = [2 3 5 6]
    if item <= 3
        nominal = batch.trajectories{1};
    else
        nominal = batch.trajectories{4};
    end
    disturbed = batch.trajectories{item};
    rowIndex = rowIndex + 1;
    rows{rowIndex, 1} = classifyTrajectory( ...
        disturbed, nominal, axisName, force, protocol);
end
end

function row = classifyTrajectory(disturbed, nominal, axisName, force, p)
t = disturbed.qTime(:);
q = disturbed.q;
qNominal = interpolateMatrix(nominal.qTime, nominal.q, t);
qResidual = q - qNominal;
if axisName == "lateral"
    axisIndex = 1;
    tiltIndex = 5;
else
    axisIndex = 2;
    tiltIndex = 4;
end
axisResidual = qResidual(:, axisIndex);
tiltResidual = qResidual(:, tiltIndex);
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
verticalDrop = max(qInitial(3) - q(:, 3));
horizontalTravel = max(vecnorm(q(:, 1:2) - qInitial(1:2), 2, 2));
rotationDeviation = max(abs(rad2deg( ...
    q(:, 4:6) - qInitial(4:6))), [], "all");
fallVertical = verticalDrop > p.fallVerticalDrop_m;
fallTravel = horizontalTravel > p.fallHorizontalTravel_m;
fallRotation = rotationDeviation > p.fallRotation_deg;
fell = fallVertical || fallTravel || fallRotation;

contactTime = disturbed.contactTime(:);
postContact = contactTime >= pulseEnd;
postRightLoad = disturbed.rightFootLoad(postContact);
postLeftLoad = disturbed.leftFootLoad(postContact);
bothUnloaded = disturbed.rightFootLoad(:) <= ...
    p.contactLoadThreshold_N & disturbed.leftFootLoad(:) <= ...
    p.contactLoadThreshold_N & postContact;
contactLossDuration = maskDuration(contactTime, bothUnloaded);
contactLoss = any(bothUnloaded);

torqueTime = disturbed.torqueTime(:);
saturated = any(abs(disturbed.totalTorque) >= ...
    100 - p.saturationTolerance_Nm, 2);
saturationDuration = maskDuration(torqueTime, saturated);

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
[firstEnvelopeEntryTime,recoveryTime] = recoveryTimesFromEnvelope( ...
    t, axisResidual, tiltResidual, velocityResidual, pulseEnd, p);
if ~recovered
    recoveryTime = NaN;
end

definition = disturbed.definition;
signedForce = force;
if contains(definition.Condition, "negative")
    signedForce = -force;
end
row = table(string(definition.Controller), string(axisName), ...
    sign(signedForce), signedForce, abs(signedForce) * p.pulseDuration_s, ...
    outcome, fell, recovered, firstEnvelopeEntryTime, recoveryTime, ...
    fallVertical, fallTravel, fallRotation, ...
    verticalDrop, horizontalTravel, rotationDeviation, ...
    contactLoss, contactLossDuration, any(saturated), saturationDuration, ...
    max(abs(axisResidual)), rad2deg(max(abs(tiltResidual))), ...
    lateDisplacement, lateTilt, lateVelocity, ...
    displacementRatio, tiltRatio, ...
    rmsAll(rad2deg(legResidual)), ...
    max(abs(rad2deg(legResidual)), [], "all"), ...
    rmsAll(disturbed.pureTorque), ...
    max(abs(disturbed.pureTorque), [], "all"), ...
    rmsAll(disturbed.totalTorque), ...
    max(abs(disturbed.totalTorque), [], "all"), ...
    min(postRightLoad), min(postLeftLoad), ...
    'VariableNames', {'Controller', 'Axis', 'DirectionSign', ...
    'SignedForce_N', 'Impulse_Ns', 'Outcome', 'Fell', 'Recovered', ...
    'FirstEnvelopeEntryTime_s', 'RecoveryTime_s', ...
    'FallByVerticalDrop', 'FallByHorizontalTravel', 'FallByRotation', ...
    'PeakVerticalDrop_m', 'PeakHorizontalTravel_m', ...
    'PeakAbsoluteRotationDeviation_deg', ...
    'ContactLoss', 'ContactLossDuration_s', ...
    'ActuatorSaturation', 'ActuatorSaturationDuration_s', ...
    'PeakForceInducedAxisDisplacement_m', ...
    'PeakForceInducedAxisTilt_deg', ...
    'LateForceInducedAxisDisplacementRMS_m', ...
    'LateForceInducedAxisTiltRMS_deg', ...
    'LateForceInducedAxisVelocityRMS_m_s', ...
    'AxisDisplacementLateEarlyRatio', 'AxisTiltLateEarlyRatio', ...
    'ForceInducedPostureRMS_deg', ...
    'MaxForceInducedJointDeviation_deg', ...
    'PurePDTorqueRMS_Nm', 'PeakPurePDTorque_Nm', ...
    'AppliedTorqueRMS_Nm', 'PeakAppliedTorque_Nm', ...
    'MinimumRightFootLoad_N', 'MinimumLeftFootLoad_N'});
end

function [firstEntry, sustainedRecovery] = recoveryTimesFromEnvelope( ...
        t, displacement, tilt, velocity, pulseEnd, p)
firstEntry = NaN;
sustainedRecovery = NaN;
starts = find(t >= pulseEnd & t <= t(end) - p.recoveryTimeDwell_s);
valid = false(size(starts));
completionTime = nan(size(starts));
for position = 1:numel(starts)
    index = starts(position);
    finish = find(t >= t(index) + p.recoveryTimeDwell_s, 1);
    if isempty(finish)
        continue
    end
    window = index:finish;
    valid(position) = ...
        rmsAll(displacement(window)) <= p.recoveryDisplacementRMS_m && ...
        rad2deg(rmsAll(tilt(window))) <= p.recoveryTiltRMS_deg && ...
        rmsAll(velocity(window)) <= p.recoveryVelocityRMS_m_s;
    completionTime(position) = t(finish) - pulseEnd;
end
firstValid = find(valid, 1, "first");
if ~isempty(firstValid)
    firstEntry = completionTime(firstValid);
end
for position = 1:numel(valid)
    if valid(position) && all(valid(position:end))
        sustainedRecovery = completionTime(position);
        break
    end
end
end

function summary = makeSummary(metrics)
controllers = ["A"; "A"; "D"; "D"];
axes = ["lateral"; "sagittal"; "lateral"; "sagittal"];
recoveredCount = zeros(4, 1);
uprightCount = zeros(4, 1);
fallCount = zeros(4, 1);
maxRecovered = nan(4, 1);
maxBidirectionalRecovered = nan(4, 1);
for index = 1:4
    selected = metrics.Controller == controllers(index) & ...
        metrics.Axis == axes(index);
    recoveredCount(index) = nnz(metrics.Outcome(selected) == "recovered");
    uprightCount(index) = nnz( ...
        metrics.Outcome(selected) == "upright_not_recovered");
    fallCount(index) = nnz(metrics.Outcome(selected) == "fell");
    recoveredForces = abs(metrics.SignedForce_N(selected & metrics.Recovered));
    if ~isempty(recoveredForces)
        maxRecovered(index) = max(recoveredForces);
    end
    testedMagnitudes = unique(abs(metrics.SignedForce_N(selected)));
    bidirectional = false(size(testedMagnitudes));
    for forceIndex = 1:numel(testedMagnitudes)
        magnitude = testedMagnitudes(forceIndex);
        atMagnitude = selected & ...
            abs(metrics.SignedForce_N) == magnitude;
        bidirectional(forceIndex) = nnz(atMagnitude) == 2 && ...
            all(metrics.Recovered(atMagnitude));
    end
    if any(bidirectional)
        maxBidirectionalRecovered(index) = ...
            max(testedMagnitudes(bidirectional));
    end
end
summary = table(controllers, axes, recoveredCount, uprightCount, ...
    fallCount, maxRecovered, maxBidirectionalRecovered, ...
    'VariableNames', ...
    {'Controller', 'Axis', 'RecoveredCaseCount', ...
    'UprightNotRecoveredCaseCount', 'FallCaseCount', ...
    'MaximumEitherDirectionRecoveredForce_N', ...
    'MaximumBidirectionallyRecoveredForce_N'});
end

function output = interpolateMatrix(sourceTime, values, targetTime)
if isequal(sourceTime, targetTime)
    output = values;
else
    output = interp1(sourceTime, values, targetTime, "linear", "extrap");
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
value = sqrt(mean(values.^2, "all"));
end
