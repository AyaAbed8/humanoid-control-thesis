function results = run_project_demo()
%RUN_PROJECT_DEMO Reproduce the headline Controller A versus RL75 result.
%   Runs matched nominal and +150 N lateral-push cases for the frozen
%   model-based baseline and deterministic residual-RL controller. The
%   command prints the formal finite-horizon outcomes and plots the
%   force-induced lateral torso displacement.

repositoryRoot = fileparts(mfilename("fullpath"));
cd(repositoryRoot);
setup_project;
verify_release_deployment;

protocol = frozen_robustness_protocol();
force_N = 150;

humanoid_walker_parameters;
inverseDynamics;
residual_rl_parameters;
[agentObj, frozenPolicy] = load_frozen_residual_rl_agent();

residualNames = string(who("residualRL*"));
residualVariables = struct();
for index = 1:numel(residualNames)
    name = residualNames(index);
    residualVariables.(name) = eval(name);
end

fprintf("\nHumanoid balance-control demonstration\n");
fprintf("Frozen policy: %s\n", frozenPolicy.id);
fprintf("Disturbance: +%g N lateral, applied from %.1f to %.1f s\n", ...
    force_N, protocol.pulseStart_s, ...
    protocol.pulseStart_s + protocol.pulseDuration_s);
fprintf("Horizon: %g s\n\n", protocol.stopTime_s);

controllers = ["A", "RL75"];
nominal = cell(2, 1);
disturbed = cell(2, 1);
outcomes = strings(2, 1);

for index = 1:2
    controller = controllers(index);
    fprintf("[%d/4] %s, nominal\n", 2*index-1, controller);
    nominal{index} = simulateCase(controller, 0, protocol, agentObj, ...
        params, robotFull, u_shoulder_frontal, residualVariables);
    fprintf("[%d/4] %s, +%g N lateral push\n", ...
        2*index, controller, force_N);
    disturbed{index} = simulateCase(controller, force_N, protocol, ...
        agentObj, params, robotFull, u_shoulder_frontal, ...
        residualVariables);
    outcomes(index) = classifyCase( ...
        disturbed{index}, nominal{index}, protocol);
end

summary = table(controllers(:), repmat(force_N, 2, 1), outcomes, ...
    'VariableNames', {'Controller','SignedForce_N','Outcome'});
disp(summary);

figureHandle = figure('Color', 'w', 'Name', ...
    'Humanoid balance-control demonstration');
axesHandle = axes(figureHandle);
hold(axesHandle, 'on');
colours = [0.0000 0.4470 0.7410; 0.8500 0.3250 0.0980];
for index = 1:2
    time = disturbed{index}.time;
    nominalQ = interp1(nominal{index}.time, nominal{index}.q, ...
        time, 'linear', 'extrap');
    displacement_cm = 100 * ...
        (disturbed{index}.q(:, 1) - nominalQ(:, 1));
    plot(axesHandle, time, displacement_cm, 'LineWidth', 1.8, ...
        'Color', colours(index, :));
end
xline(axesHandle, protocol.pulseStart_s, ':', 'Push', ...
    'HandleVisibility', 'off');
xline(axesHandle, protocol.pulseStart_s + protocol.pulseDuration_s, ...
    ':', 'HandleVisibility', 'off');
yline(axesHandle, 100, ':', '1 m fall threshold', ...
    'HandleVisibility', 'off');
grid(axesHandle, 'on');
xlabel(axesHandle, 'Time (s)');
ylabel(axesHandle, 'Force-induced lateral displacement (cm)');
legend(axesHandle, {'Controller A','Controller A + RL75'}, ...
    'Location', 'best');
title(axesHandle, '+150 N lateral-push response');

passed = outcomes(1) == "fell" && outcomes(2) == "recovered";
if passed
    fprintf("Demo verification passed: Controller A fell and RL75 recovered.\n");
else
    warning("ProjectDemo:UnexpectedOutcome", ...
        "Expected A='fell' and RL75='recovered'; observed %s and %s.", ...
        outcomes(1), outcomes(2));
end

results = struct('summary', summary, 'protocol', protocol, ...
    'force_N', force_N, 'nominal', {nominal}, ...
    'disturbed', {disturbed}, 'figure', figureHandle, ...
    'verificationPassed', passed);
end

function trajectory = simulateCase(controller, force_N, protocol, ...
        agentObj, params, robotFull, uShoulder, residualVariables)
model = "HumanoidModel_ResidualRL";
load_system(model);
cleanup = onCleanup(@() closeIfLoaded(model));

simIn = Simulink.SimulationInput(model);
[simIn, ~] = apply_frozen_pd_gravity_controller(simIn, "A", model);
simIn = simIn.setVariable("params", params);
simIn = simIn.setVariable("robotFull", robotFull);
simIn = simIn.setVariable("u_shoulder_frontal", uShoulder);
names = string(fieldnames(residualVariables));
for index = 1:numel(names)
    name = names(index);
    simIn = simIn.setVariable(name, residualVariables.(name));
end
simIn = simIn.setVariable("agentObj", agentObj);
if controller == "RL75"
    simIn = simIn.setVariable("residualRLEnabled", 1);
    simIn = simIn.setVariable("residualRLUseAgent", 1);
else
    simIn = simIn.setVariable("residualRLEnabled", 0);
    simIn = simIn.setVariable("residualRLUseAgent", 0);
end
simIn = simIn.setVariable("robustnessForceVector_N", [force_N; 0; 0]);
simIn = simIn.setVariable("robustnessForceStart_s", ...
    protocol.pulseStart_s);
simIn = simIn.setVariable("robustnessForceDuration_s", ...
    protocol.pulseDuration_s);
simIn = simIn.setModelParameter( ...
    "StopTime", string(protocol.stopTime_s), ...
    "SignalLogging", "on", ...
    "SignalLoggingName", "logsout", ...
    "ReturnWorkspaceOutputs", "on", ...
    "UnconnectedInputMsg", "none", ...
    "UnconnectedOutputMsg", "none", ...
    "UnconnectedLineMsg", "none");
output = sim(simIn);
assert(any(strcmp(output.who, "logsout")), ...
    "ProjectDemo:LogsMissing", "Simulation returned no logsout dataset.");
[time, q] = loggedMatrix(output.logsout, "diag_q_rbt");
assert(size(q, 2) == 22, "ProjectDemo:StateWidth", ...
    "Expected 22 rigidBodyTree coordinates; received %d.", size(q, 2));
trajectory = struct('time', time, 'q', q);
close_system(model, 0);
clear cleanup
end

function outcome = classifyCase(disturbed, nominal, protocol)
time = disturbed.time;
q = disturbed.q;
nominalQ = interp1(nominal.time, nominal.q, time, 'linear', 'extrap');
residual = q - nominalQ;
axisResidual = residual(:, 1);
tiltResidual = residual(:, 5);
velocityResidual = gradient(axisResidual, time);

initial = q(1, :);
verticalDrop = initial(3) - min(q(:, 3));
horizontalTravel = max(vecnorm(q(:, 1:2) - initial(1:2), 2, 2));
rotationDeviation = max(abs(rad2deg(q(:, 4:6) - initial(4:6))), ...
    [], 'all');
fell = verticalDrop > protocol.fallVerticalDrop_m || ...
    horizontalTravel > protocol.fallHorizontalTravel_m || ...
    rotationDeviation > protocol.fallRotation_deg;

pulseEnd = protocol.pulseStart_s + protocol.pulseDuration_s;
early = time >= pulseEnd & time <= pulseEnd + 2;
late = time >= protocol.stopTime_s - protocol.finalDwell_s;
earlyDisplacement = rmsAll(axisResidual(early));
lateDisplacement = rmsAll(axisResidual(late));
earlyTilt = rad2deg(rmsAll(tiltResidual(early)));
lateTilt = rad2deg(rmsAll(tiltResidual(late)));
lateVelocity = rmsAll(velocityResidual(late));

withinEnvelope = ...
    lateDisplacement <= protocol.recoveryDisplacementRMS_m && ...
    lateTilt <= protocol.recoveryTiltRMS_deg && ...
    lateVelocity <= protocol.recoveryVelocityRMS_m_s;
displacementSettled = safeRatio(lateDisplacement, earlyDisplacement) ...
    <= protocol.settlingRatioLimit || ...
    lateDisplacement <= protocol.smallDisplacementRMS_m;
tiltSettled = safeRatio(lateTilt, earlyTilt) ...
    <= protocol.settlingRatioLimit || ...
    lateTilt <= protocol.smallTiltRMS_deg;
recovered = ~fell && withinEnvelope && ...
    displacementSettled && tiltSettled;

if fell
    outcome = "fell";
elseif recovered
    outcome = "recovered";
else
    outcome = "upright_not_recovered";
end
end

function value = safeRatio(numerator, denominator)
if denominator <= eps
    value = double(numerator > eps) * inf;
    if numerator <= eps
        value = 0;
    end
else
    value = numerator / denominator;
end
end

function value = rmsAll(values)
value = sqrt(mean(values.^2, 'all'));
end

function [time, values] = loggedMatrix(logs, signalName)
element = logs.get(signalName);
assert(~isempty(element), "ProjectDemo:SignalMissing", ...
    "Required logged signal '%s' was not found.", signalName);
[time, values] = flattenPayload(element.Values);
time = double(time(:));
values = double(values);
if isvector(values)
    values = values(:);
elseif size(values, 1) ~= numel(time) && size(values, 2) == numel(time)
    values = values.';
end
assert(size(values, 1) == numel(time), "ProjectDemo:SignalLength", ...
    "Signal '%s' has incompatible time and data lengths.", signalName);
end

function [time, values] = flattenPayload(payload)
if isa(payload, 'timeseries')
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
if isa(payload, 'Simulink.SimulationData.Signal')
    [time, values] = flattenPayload(payload.Values);
    return
end
if isa(payload, 'Simulink.SimulationData.Dataset')
    time = [];
    values = [];
    for index = 1:payload.numElements
        [itemTime, itemValues] = ...
            flattenPayload(payload.getElement(index));
        if isempty(time)
            time = itemTime(:);
            values = itemValues;
        else
            values = [values, ...
                alignToTime(itemTime, itemValues, time)]; %#ok<AGROW>
        end
    end
    return
end
if isstruct(payload)
    if isfield(payload, 'time') && isfield(payload, 'signals')
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
            values = [values, ...
                alignToTime(itemTime, itemValues, time)]; %#ok<AGROW>
        end
    end
    return
end
error("ProjectDemo:PayloadType", ...
    "Unsupported logged payload type: %s.", class(payload));
end

function aligned = alignToTime(sourceTime, sourceValues, targetTime)
sourceTime = sourceTime(:);
if isscalar(sourceTime)
    aligned = repmat(sourceValues(1, :), numel(targetTime), 1);
else
    aligned = interp1(sourceTime, sourceValues, ...
        targetTime, 'linear', 'extrap');
end
end

function closeIfLoaded(model)
if getSimulinkBlockHandle(model) >= 0
    close_system(model, 0);
end
end
