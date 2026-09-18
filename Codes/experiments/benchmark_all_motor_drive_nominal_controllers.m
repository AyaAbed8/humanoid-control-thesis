function results = benchmark_all_motor_drive_nominal_controllers(outputRoot,options)
%BENCHMARK_ALL_MOTOR_DRIVE_NOMINAL_CONTROLLERS Nominal A-D-RL75 gate.
% Runs the frozen controllers in the actuator-study copy with all 14
% Motor & Drive paths bypassed and enabled. No source model is edited.

arguments
    outputRoot (1,1) string = fullfile(project_codes_root(), ...
        "Results","AllMotorDriveNominalControllerGate")
    options.StopTime_s (1,1) double {mustBePositive} = 12
    options.ActuationModes (1,:) string = ["ideal_bypass","all_motor_drives"]
end
codeDirectory = string(project_codes_root());
cd(codeDirectory);
model = "HumanoidModel_ActuatorNonideal";
protocol = frozen_robustness_protocol();
protocol.stopTime_s = options.StopTime_s;
maxStep_s = 0.001;
load_system(model);

enableBlocks = string(find_system(model,"LookUnderMasks","all", ...
    "FollowLinks","off","RegExp","on","BlockType","Constant", ...
    "Name","^Actuator Nonideality Enable"));
assert(numel(enableBlocks)==14,"MotorNominalGate:EnableCount", ...
    "Expected 14 actuator-enable blocks; found %d.",numel(enableBlocks));

humanoid_walker_parameters; inverseDynamics; residual_rl_parameters;
residualNames = string(who("residualRL*"));
residual = struct();
for index=1:numel(residualNames)
    name=residualNames(index); residual.(name)=eval(name);
end
agentData=load(fullfile(codeDirectory,"FrozenControllers", ...
    "ResidualRL75","RL75_agent.mat"),"saved_agent");
agentObj=agentData.saved_agent; agentObj.UseExplorationPolicy=false;

allowedModes=["ideal_bypass","all_motor_drives"];
assert(all(ismember(options.ActuationModes,allowedModes)) && ...
    numel(unique(options.ActuationModes))==numel(options.ActuationModes), ...
    "MotorNominalGate:ActuationModes", ...
    "ActuationModes must contain unique values from: %s.", ...
    strjoin(allowedModes,", "));
controllers=repmat(["A";"D";"RL75"],numel(options.ActuationModes),1);
actuation=repelem(options.ActuationModes(:),3,1);
enable=double(actuation=="all_motor_drives");
definitions=table(controllers,actuation,enable, ...
    'VariableNames',{'Controller','Actuation','Enable'});
stamp=string(datetime("now","Format","yyyyMMdd_HHmmss"));
resultDirectory=fullfile(outputRoot,stamp); mkdir(resultDirectory);

trajectories=cell(height(definitions),1);
for runIndex=1:height(definitions)
    d=definitions(runIndex,:);
    simIn=Simulink.SimulationInput(model);
    baseController=d.Controller;
    if baseController=="RL75"; baseController="A"; end
    [simIn,~]=apply_frozen_pd_gravity_controller(simIn,baseController,model);
    simIn=simIn.setVariable("params",params) ...
        .setVariable("robotFull",robotFull) ...
        .setVariable("u_shoulder_frontal",u_shoulder_frontal) ...
        .setVariable("agentObj",agentObj);
    for index=1:numel(residualNames)
        name=residualNames(index);
        simIn=simIn.setVariable(name,residual.(name));
    end
    useRL=double(d.Controller=="RL75");
    simIn=simIn.setVariable("residualRLEnabled",useRL) ...
        .setVariable("residualRLUseAgent",useRL) ...
        .setVariable("residualRLManualAction",zeros(2,1)) ...
        .setVariable("robustnessForceVector_N",zeros(3,1)) ...
        .setVariable("robustnessForceStart_s",protocol.pulseStart_s) ...
        .setVariable("robustnessForceDuration_s",protocol.pulseDuration_s);
    for index=1:numel(enableBlocks)
        simIn=simIn.setBlockParameter(enableBlocks(index),"Value", ...
            string(d.Enable));
    end
    simIn=simIn.setModelParameter("StopTime",string(protocol.stopTime_s), ...
        "MaxStep",string(maxStep_s),"SignalLogging","on", ...
        "SignalLoggingName","logsout","ReturnWorkspaceOutputs","on", ...
        "UnconnectedInputMsg","none","UnconnectedOutputMsg","none", ...
        "UnconnectedLineMsg","none");
    fprintf("[%d/%d] %s, %s\n",runIndex,height(definitions), ...
        d.Controller,d.Actuation);
    trajectories{runIndex}=extractTrajectory(sim(simIn));
end

rows=cell(height(definitions),1);
for runIndex=1:height(definitions)
    rows{runIndex}=nominalMetrics(trajectories{runIndex}, ...
        definitions(runIndex,:),protocol);
end
metrics=vertcat(rows{:});
save(fullfile(resultDirectory,"raw_trajectories.mat"), ...
    "definitions","trajectories","protocol","maxStep_s","-v7.3");
writetable(metrics,fullfile(resultDirectory,"nominal_metrics.csv"));
save(fullfile(resultDirectory,"processed_results.mat"),"metrics", ...
    "definitions","trajectories","protocol","maxStep_s","-v7.3");
results=struct("metrics",metrics,"definitions",definitions, ...
    "trajectories",{trajectories},"protocol",protocol, ...
    "resultDirectory",string(resultDirectory));
fprintf("\nAll-actuator nominal controller gate:\n"); disp(metrics);
fprintf("Results written to %s\n",resultDirectory);
end

function row=nominalMetrics(x,d,p)
t=x.time; q=x.q; initial=q(1,:); delta=q-initial;
verticalDrop=max(initial(3)-q(:,3));
horizontalTravel=max(vecnorm(delta(:,1:2),2,2));
rotation=max(abs(rad2deg(delta(:,4:6))),[],"all");
fell=verticalDrop>p.fallVerticalDrop_m || ...
    horizontalTravel>p.fallHorizontalTravel_m || ...
    rotation>p.fallRotation_deg;
if fell; outcome="fell"; else; outcome="nominal_upright"; end
jointDelta=rad2deg(delta(:,7:22));
postureRMS=rms(jointDelta,"all");
maxJoint=max(abs(jointDelta),[],"all");
torsoTranslationRMS=rms(vecnorm(delta(:,1:3),2,2));
torsoRotationRMS=rms(vecnorm(rad2deg(delta(:,4:6)),2,2));
row=table(d.Controller,d.Actuation,d.Enable,t(end), ...
    t(end)>=p.stopTime_s-1e-9,outcome,fell,postureRMS,maxJoint, ...
    torsoTranslationRMS,torsoRotationRMS,verticalDrop, ...
    horizontalTravel,rotation,'VariableNames', ...
    {'Controller','Actuation','Enable','FinalTime_s','ReachedStopTime', ...
    'Outcome','Fell','PostureRMSError_deg','MaxJointDeviation_deg', ...
    'TorsoTranslationRMSError_m','TorsoRotationRMSError_deg', ...
    'PeakVerticalDrop_m','PeakHorizontalTravel_m', ...
    'PeakAbsoluteRotationDeviation_deg'});
end

function x=extractTrajectory(output)
[time,q]=loggedMatrix(output.logsout,"diag_q_rbt");
q=requireWidth(q,22);
x=struct("time",time,"q",q);
end

function values=requireWidth(values,n)
if size(values,2)~=n && size(values,1)==n; values=values.'; end
assert(size(values,2)==n,"MotorNominalGate:Width", ...
    "Expected width %d; received %d.",n,size(values,2));
end

function [time,values]=loggedMatrix(logs,name)
item=logs.getElement(name);
if isa(item,"Simulink.SimulationData.Signal"); item=item.Values; end
[time,values]=flattenPayload(item);
time=double(time(:)); values=double(values);
if isvector(values); values=values(:);
elseif size(values,1)~=numel(time) && size(values,2)==numel(time)
    values=values.';
end
end

function [time,values]=flattenPayload(x)
if isa(x,"timeseries"); time=x.Time; values=squeeze(x.Data); return; end
if isa(x,"Simulink.SimulationData.Signal")
    [time,values]=flattenPayload(x.Values); return
end
if istimetable(x)
    time=seconds(x.Properties.RowTimes-x.Properties.RowTimes(1));
    values=x.Variables; return
end
if isa(x,"Simulink.SimulationData.Dataset")
    time=[]; values=[];
    for index=1:x.numElements
        [itemTime,itemValues]=flattenPayload(x.getElement(index));
        if isempty(time); time=itemTime(:); values=itemValues;
        else; values=[values,align(itemTime,itemValues,time)]; end %#ok<AGROW>
    end
    return
end
if isstruct(x)
    fields=string(fieldnames(x)); lowerFields=lower(fields);
    tf=fields(lowerFields=="time"); df=fields(lowerFields=="data");
    sf=fields(lowerFields=="signals"); vf=fields(lowerFields=="values");
    if ~isempty(tf)&&~isempty(df)
        time=x.(tf(1)); values=squeeze(x.(df(1))); return
    end
    if ~isempty(tf)&&~isempty(sf)
        signal=x.(sf(1)); names=string(fieldnames(signal));
        names=names(lower(names)=="values");
        time=x.(tf(1)); values=squeeze(signal.(names(1))); return
    end
    if ~isempty(vf); [time,values]=flattenPayload(x.(vf(1))); return; end
    time=[]; values=[];
    for index=1:numel(fields)
        try
            [itemTime,itemValues]=flattenPayload(x.(fields(index)));
        catch exception
            if exception.identifier=="MotorNominalGate:Payload"; continue; end
            rethrow(exception)
        end
        if isempty(time); time=itemTime(:); values=itemValues;
        else; values=[values,align(itemTime,itemValues,time)]; end %#ok<AGROW>
    end
    if ~isempty(time); return; end
end
error("MotorNominalGate:Payload","Unsupported payload %s.",class(x));
end

function values=align(sourceTime,sourceValues,targetTime)
sourceTime=sourceTime(:);
if isscalar(sourceTime)
    values=repmat(sourceValues(1,:),numel(targetTime),1);
else
    values=interp1(sourceTime,sourceValues,targetTime,"linear","extrap");
end
end
