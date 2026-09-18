function results = audit_rl75_all_motor_drive_nominal_action(stopTime_s,outputRoot)
%AUDIT_RL75_ALL_MOTOR_DRIVE_NOMINAL_ACTION Inspect residual activity.
% Runs only frozen deterministic RL75 with all 14 Motor & Drive paths
% enabled and zero disturbance. No model or controller is modified.

arguments
    stopTime_s (1,1) double {mustBePositive}=20
    outputRoot (1,1) string=fullfile(project_codes_root(), ...
        "Results","RL75AllMotorDriveNominalActionAudit")
end
codeDirectory=string(project_codes_root()); cd(codeDirectory);
model="HumanoidModel_ActuatorNonideal"; load_system(model);
enableBlocks=string(find_system(model,"LookUnderMasks","all", ...
    "FollowLinks","off","RegExp","on","BlockType","Constant", ...
    "Name","^Actuator Nonideality Enable"));
assert(numel(enableBlocks)==14,"RL75MotorAudit:EnableCount", ...
    "Expected 14 actuator-enable blocks; found %d.",numel(enableBlocks));
humanoid_walker_parameters; inverseDynamics; residual_rl_parameters;
residualNames=string(who("residualRL*")); residual=struct();
for index=1:numel(residualNames)
    name=residualNames(index); residual.(name)=eval(name);
end
agentData=load(fullfile(codeDirectory,"FrozenControllers", ...
    "ResidualRL75","RL75_agent.mat"),"saved_agent");
agentObj=agentData.saved_agent; agentObj.UseExplorationPolicy=false;

simIn=Simulink.SimulationInput(model);
[simIn,~]=apply_frozen_pd_gravity_controller(simIn,"A",model);
simIn=simIn.setVariable("params",params).setVariable("robotFull",robotFull) ...
    .setVariable("u_shoulder_frontal",u_shoulder_frontal) ...
    .setVariable("agentObj",agentObj);
for index=1:numel(residualNames)
    name=residualNames(index); simIn=simIn.setVariable(name,residual.(name));
end
simIn=simIn.setVariable("residualRLEnabled",1) ...
    .setVariable("residualRLUseAgent",1) ...
    .setVariable("residualRLManualAction",zeros(2,1)) ...
    .setVariable("robustnessForceVector_N",zeros(3,1)) ...
    .setVariable("robustnessForceStart_s",2) ...
    .setVariable("robustnessForceDuration_s",0.1);
for index=1:numel(enableBlocks)
    simIn=simIn.setBlockParameter(enableBlocks(index),"Value","1");
end
simIn=simIn.setModelParameter("StopTime",string(stopTime_s), ...
    "MaxStep","0.001","SignalLogging","on", ...
    "SignalLoggingName","logsout","ReturnWorkspaceOutputs","on", ...
    "UnconnectedInputMsg","none","UnconnectedOutputMsg","none", ...
    "UnconnectedLineMsg","none");
fprintf("[1/1] RL75, all Motor & Drive paths, zero disturbance\n");
output=sim(simIn);
[qTime,q]=loggedMatrix(output.logsout,"diag_q_rbt"); q=requireWidth(q,22);
[residualTime,residualTorque]=loggedMatrix( ...
    output.logsout,"rl_residual_hip_torque_Nm");
residualTorque=requireWidth(residualTorque,2);
torqueLimit_Nm=residual.residualRLTorqueLimit_Nm;
normalizedAction=residualTorque/torqueLimit_Nm;

windows=table([0;2;8;stopTime_s-2],[2;4;10;stopTime_s], ...
    ["initial";"early";"middle";"late"], ...
    'VariableNames',{'Start_s','End_s','Window'});
rows=cell(height(windows),1);
for index=1:height(windows)
    w=windows(index,:);
    residualMask=residualTime>=w.Start_s & residualTime<=w.End_s;
    qMask=qTime>=w.Start_s & qTime<=w.End_s;
    tau=residualTorque(residualMask,:); action=normalizedAction(residualMask,:);
    qWindow=q(qMask,:); tWindow=qTime(qMask);
    delta=qWindow-qWindow(1,:);
    verticalTravel=sum(abs(diff(qWindow(:,3))));
    jointPath=sum(vecnorm(diff(rad2deg(qWindow(:,7:22))),2,2));
    verticalVelocity=gradient(qWindow(:,3),tWindow);
    rows{index}=table(w.Window,w.Start_s,w.End_s,rms(tau,"all"), ...
        max(abs(tau),[],"all"),rms(action,"all"), ...
        max(abs(action),[],"all"),100*mean(any(abs(action)>=0.99,2)), ...
        verticalTravel,jointPath,rms(verticalVelocity), ...
        max(abs(rad2deg(delta(:,7:22))),[],"all"), ...
        'VariableNames',{'Window','Start_s','End_s', ...
        'ResidualTorqueRMS_Nm','PeakResidualTorque_Nm', ...
        'NormalizedActionRMS','PeakNormalizedAction', ...
        'SamplesNearActionBound_percent','TorsoVerticalCumulativeTravel_m', ...
        'JointConfigurationPathLength_deg','TorsoVerticalVelocityRMS_m_s', ...
        'MaximumWithinWindowJointChange_deg'});
end
metrics=vertcat(rows{:});

figureHandle=figure("Color","w","Name","RL75 nonideal-actuator action audit");
layout=tiledlayout(3,1,"TileSpacing","compact","Padding","compact");
nexttile(layout); plot(residualTime,residualTorque,"LineWidth",1.2); grid on
yline(torqueLimit_Nm,":"); yline(-torqueLimit_Nm,":");
ylabel("Residual torque (N m)"); legend("Right hip","Left hip", ...
    "Location","best"); title("Frozen RL75 residual action");
nexttile(layout); plot(qTime,1000*(q(:,3)-q(1,3)),"LineWidth",1.2); grid on
ylabel("Torso Delta z (mm)"); title("Vertical torso motion");
nexttile(layout); plot(qTime,rad2deg(q(:,[14 20])-q(1,[14 20])), ...
    "LineWidth",1.2); grid on; ylabel("Knee change (deg)");
xlabel("Time (s)"); legend("Right","Left","Location","best");
title("Bilateral knee motion");

stamp=string(datetime("now","Format","yyyyMMdd_HHmmss"));
resultDirectory=fullfile(outputRoot,stamp); mkdir(resultDirectory);
writetable(metrics,fullfile(resultDirectory,"window_metrics.csv"));
exportgraphics(figureHandle,fullfile(resultDirectory,"action_audit.png"), ...
    "Resolution",200);
save(fullfile(resultDirectory,"processed_results.mat"),"metrics", ...
    "qTime","q","residualTime","residualTorque","normalizedAction", ...
    "torqueLimit_Nm","windows","-v7.3");
results=struct("metrics",metrics,"qTime",qTime,"q",q, ...
    "residualTime",residualTime,"residualTorque",residualTorque, ...
    "normalizedAction",normalizedAction,"figure",figureHandle, ...
    "resultDirectory",string(resultDirectory));
fprintf("\nRL75 nominal residual-action audit:\n"); disp(metrics);
fprintf("Results written to %s\n",resultDirectory);
end

function values=requireWidth(values,n)
if size(values,2)~=n && size(values,1)==n; values=values.'; end
assert(size(values,2)==n,"RL75MotorAudit:Width", ...
    "Expected width %d; received %d.",n,size(values,2));
end

function [time,values]=loggedMatrix(logs,name)
item=logs.getElement(name);
if isa(item,"Simulink.SimulationData.Signal"); item=item.Values; end
[time,values]=flatten(item); time=double(time(:)); values=double(values);
if isvector(values); values=values(:);
elseif size(values,1)~=numel(time)&&size(values,2)==numel(time)
    values=values.';
end
end

function [time,values]=flatten(x)
if isa(x,"timeseries"); time=x.Time; values=squeeze(x.Data); return; end
if isa(x,"Simulink.SimulationData.Signal"); [time,values]=flatten(x.Values); return; end
if istimetable(x)
    time=seconds(x.Properties.RowTimes-x.Properties.RowTimes(1));
    values=x.Variables; return
end
if isa(x,"Simulink.SimulationData.Dataset")
    time=[]; values=[];
    for index=1:x.numElements
        [itemTime,itemValues]=flatten(x.getElement(index));
        if isempty(time); time=itemTime(:); values=itemValues;
        else; values=[values,align(itemTime,itemValues,time)]; end %#ok<AGROW>
    end
    return
end
if isstruct(x)
    f=string(fieldnames(x)); l=lower(f); tf=f(l=="time"); df=f(l=="data");
    sf=f(l=="signals"); vf=f(l=="values");
    if ~isempty(tf)&&~isempty(df); time=x.(tf(1)); values=squeeze(x.(df(1))); return; end
    if ~isempty(tf)&&~isempty(sf)
        s=x.(sf(1)); n=string(fieldnames(s)); n=n(lower(n)=="values");
        time=x.(tf(1)); values=squeeze(s.(n(1))); return
    end
    if ~isempty(vf); [time,values]=flatten(x.(vf(1))); return; end
    time=[]; values=[];
    for index=1:numel(f)
        try
            [itemTime,itemValues]=flatten(x.(f(index)));
        catch exception
            if exception.identifier=="RL75MotorAudit:Payload"; continue; end
            rethrow(exception)
        end
        if isempty(time); time=itemTime(:); values=itemValues;
        else; values=[values,align(itemTime,itemValues,time)]; end %#ok<AGROW>
    end
    if ~isempty(time); return; end
end
error("RL75MotorAudit:Payload","Unsupported payload %s.",class(x));
end

function values=align(sourceTime,sourceValues,targetTime)
sourceTime=sourceTime(:);
if isscalar(sourceTime)
    values=repmat(sourceValues(1,:),numel(targetTime),1);
else
    values=interp1(sourceTime,sourceValues,targetTime,"linear","extrap");
end
end
