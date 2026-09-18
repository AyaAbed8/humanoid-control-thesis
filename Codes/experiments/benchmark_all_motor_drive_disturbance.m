function results = benchmark_all_motor_drive_disturbance(outputRoot)
%BENCHMARK_ALL_MOTOR_DRIVE_DISTURBANCE Final nonideal-actuation comparison.
% Compares frozen A, D and RL75 with all 14 Motor & Drive paths enabled.
% Each controller receives a matched zero-force reference and lateral
% +/-100 N and +/-150 N pulses under the frozen 12 s protocol.

arguments
    outputRoot (1,1) string=fullfile(project_codes_root(), ...
        "Results","AllMotorDriveDisturbanceBenchmark")
end
codeDirectory=string(project_codes_root()); cd(codeDirectory);
model="HumanoidModel_ActuatorNonideal"; protocol=frozen_robustness_protocol();
maxStep_s=0.001; load_system(model);
enableBlocks=string(find_system(model,"LookUnderMasks","all", ...
    "FollowLinks","off","RegExp","on","BlockType","Constant", ...
    "Name","^Actuator Nonideality Enable"));
assert(numel(enableBlocks)==14,"MotorBenchmark:EnableCount", ...
    "Expected 14 actuator-enable blocks; found %d.",numel(enableBlocks));
[~,commandVariables,deliveredVariables]=motorSignalMap();

humanoid_walker_parameters; inverseDynamics; residual_rl_parameters;
residualNames=string(who("residualRL*")); residual=struct();
for index=1:numel(residualNames)
    name=residualNames(index); residual.(name)=eval(name);
end
agentData=load(fullfile(codeDirectory,"FrozenControllers", ...
    "ResidualRL75","RL75_agent.mat"),"saved_agent");
agentObj=agentData.saved_agent; agentObj.UseExplorationPolicy=false;

controllers=["A";"D";"RL75"];
forces=[0 100 -100 150 -150];
[controllerGrid,forceGrid]=ndgrid(1:numel(controllers),1:numel(forces));
definitions=table(controllers(controllerGrid(:)),forces(forceGrid(:))', ...
    'VariableNames',{'Controller','SignedForce_N'});
stamp=string(datetime("now","Format","yyyyMMdd_HHmmss"));
resultDirectory=fullfile(outputRoot,stamp); mkdir(resultDirectory);
fprintf("\nAll-Motor-&-Drive disturbance benchmark\n");
fprintf("Controllers: A, D, RL75; lateral force: +/-100 and +/-150 N\n");
fprintf("Pulse: %.1f-%.1f s; horizon: %.1f s; cases: %d\n", ...
    protocol.pulseStart_s,protocol.pulseStart_s+protocol.pulseDuration_s, ...
    protocol.stopTime_s,height(definitions));

trajectories=cell(height(definitions),1);
for runIndex=1:height(definitions)
    d=definitions(runIndex,:); simIn=Simulink.SimulationInput(model);
    baseController=d.Controller;
    if baseController=="RL75"; baseController="A"; end
    [simIn,~]=apply_frozen_pd_gravity_controller(simIn,baseController,model);
    simIn=simIn.setVariable("params",params).setVariable("robotFull",robotFull) ...
        .setVariable("u_shoulder_frontal",u_shoulder_frontal) ...
        .setVariable("agentObj",agentObj);
    for index=1:numel(residualNames)
        name=residualNames(index); simIn=simIn.setVariable(name,residual.(name));
    end
    useRL=double(d.Controller=="RL75");
    simIn=simIn.setVariable("residualRLEnabled",useRL) ...
        .setVariable("residualRLUseAgent",useRL) ...
        .setVariable("residualRLManualAction",zeros(2,1)) ...
        .setVariable("robustnessForceVector_N",[d.SignedForce_N;0;0]) ...
        .setVariable("robustnessForceStart_s",protocol.pulseStart_s) ...
        .setVariable("robustnessForceDuration_s",protocol.pulseDuration_s);
    for index=1:numel(enableBlocks)
        simIn=simIn.setBlockParameter(enableBlocks(index),"Value","1");
    end
    simIn=simIn.setModelParameter("StopTime",string(protocol.stopTime_s), ...
        "MaxStep",string(maxStep_s),"SignalLogging","on", ...
        "SignalLoggingName","logsout","ReturnWorkspaceOutputs","on", ...
        "UnconnectedInputMsg","none","UnconnectedOutputMsg","none", ...
        "UnconnectedLineMsg","none");
    fprintf("[%d/%d] %s, Fx=%+g N\n",runIndex,height(definitions), ...
        d.Controller,d.SignedForce_N);
    trajectories{runIndex}=extract(sim(simIn),commandVariables, ...
        deliveredVariables,useRL,residual.residualRLTorqueLimit_Nm);
end
save(fullfile(resultDirectory,"raw_trajectories.mat"), ...
    "definitions","trajectories","protocol","maxStep_s","-v7.3");

rows=cell(height(definitions),1);
for runIndex=1:height(definitions)
    d=definitions(runIndex,:);
    nominalIndex=find(definitions.Controller==d.Controller & ...
        definitions.SignedForce_N==0,1);
    rows{runIndex}=classify(trajectories{runIndex}, ...
        trajectories{nominalIndex},d,protocol);
end
metrics=vertcat(rows{:}); summary=summarize(metrics);
writetable(metrics,fullfile(resultDirectory,"metrics.csv"));
writetable(summary,fullfile(resultDirectory,"summary.csv"));
save(fullfile(resultDirectory,"processed_results.mat"),"metrics", ...
    "summary","definitions","trajectories","protocol","maxStep_s","-v7.3");
results=struct("metrics",metrics,"summary",summary, ...
    "definitions",definitions,"trajectories",{trajectories}, ...
    "protocol",protocol,"resultDirectory",string(resultDirectory));
fprintf("\nAll-actuator disturbance metrics:\n"); disp(metrics);
fprintf("\nController summary:\n"); disp(summary);
fprintf("Results written to %s\n",resultDirectory);
end

function row=classify(x,nominal,d,p)
t=x.time; q=x.q; qNominal=interp1(nominal.time,nominal.q,t,"linear","extrap");
induced=q-qNominal; displacement=induced(:,1); tilt=induced(:,5);
velocity=gradient(displacement,t); pulseEnd=p.pulseStart_s+p.pulseDuration_s;
early=t>=pulseEnd & t<=pulseEnd+2; late=t>=p.stopTime_s-p.finalDwell_s;
earlyD=rms(displacement(early)); lateD=rms(displacement(late));
earlyT=rad2deg(rms(tilt(early))); lateT=rad2deg(rms(tilt(late)));
lateV=rms(velocity(late)); dRatio=safeRatio(lateD,earlyD);
tRatio=safeRatio(lateT,earlyT); initial=q(1,:);
verticalDrop=max(initial(3)-q(:,3));
travel=max(vecnorm(q(:,1:2)-initial(1:2),2,2));
rotation=max(abs(rad2deg(q(:,4:6)-initial(4:6))),[],"all");
fell=verticalDrop>p.fallVerticalDrop_m || ...
    travel>p.fallHorizontalTravel_m || rotation>p.fallRotation_deg;
within=lateD<=p.recoveryDisplacementRMS_m && ...
    lateT<=p.recoveryTiltRMS_deg && lateV<=p.recoveryVelocityRMS_m_s;
settled=(dRatio<=p.settlingRatioLimit || lateD<=p.smallDisplacementRMS_m) && ...
    (tRatio<=p.settlingRatioLimit || lateT<=p.smallTiltRMS_deg);
recovered=~fell && within && settled && d.SignedForce_N~=0;
if d.SignedForce_N==0; outcome="nominal_upright";
elseif fell; outcome="fell";
elseif recovered; outcome="recovered";
else; outcome="upright_not_recovered"; end
row=table(d.Controller,d.SignedForce_N,abs(d.SignedForce_N)*p.pulseDuration_s, ...
    outcome,fell,recovered,max(abs(displacement)),rad2deg(max(abs(tilt))), ...
    lateD,lateT,lateV,dRatio,tRatio,verticalDrop,travel,rotation, ...
    x.motorTrackingRMS_Nm,x.motorTrackingPeak_Nm,x.motorSameSignPercent, ...
    x.peakDeliveredTorque_Nm,x.motorTorqueLimitReached, ...
    x.residualRMS_Nm,x.residualPeak_Nm,x.peakNormalizedAction, ...
    x.residualNearBoundPercent,'VariableNames', ...
    {'Controller','SignedForce_N','Impulse_Ns','Outcome','Fell','Recovered', ...
    'PeakForceInducedLateralDisplacement_m','PeakForceInducedLateralTilt_deg', ...
    'LateForceInducedLateralDisplacementRMS_m', ...
    'LateForceInducedLateralTiltRMS_deg', ...
    'LateForceInducedLateralVelocityRMS_m_s', ...
    'LateralDisplacementLateEarlyRatio','LateralTiltLateEarlyRatio', ...
    'PeakAbsoluteVerticalDrop_m','PeakAbsoluteHorizontalTravel_m', ...
    'PeakAbsoluteRotationDeviation_deg','MotorTrackingRMS_Nm', ...
    'MotorTrackingPeak_Nm','MotorSameSignPercent', ...
    'PeakDeliveredMotorTorque_Nm','MotorTorqueLimitReached', ...
    'ResidualTorqueRMS_Nm','PeakResidualTorque_Nm', ...
    'PeakNormalizedAction','ResidualNearBoundSamplePercent'});
end

function summary=summarize(metrics)
controllers=["A";"D";"RL75"];
recovered=zeros(3,1); upright=zeros(3,1); fell=zeros(3,1);
for index=1:3
    selected=metrics.Controller==controllers(index) & metrics.SignedForce_N~=0;
    recovered(index)=nnz(metrics.Outcome(selected)=="recovered");
    upright(index)=nnz(metrics.Outcome(selected)=="upright_not_recovered");
    fell(index)=nnz(metrics.Outcome(selected)=="fell");
end
summary=table(controllers,recovered,upright,fell, ...
    'VariableNames',{'Controller','RecoveredCaseCount', ...
    'UprightNotRecoveredCaseCount','FallCaseCount'});
end

function x=extract(output,commandVariables,deliveredVariables,useRL,limit)
[time,q]=loggedMatrix(output.logsout,"diag_q_rbt"); q=requireWidth(q,22);
errors=[]; commands=[]; delivered=[];
for index=1:numel(commandVariables)
    command=output.get(commandVariables(index));
    motor=output.get(deliveredVariables(index));
    y=interp1(motor.Time,motor.Data,command.Time,"linear","extrap");
    u=command.Data(:); y=y(:); errors=[errors;u-y]; %#ok<AGROW>
    commands=[commands;u]; delivered=[delivered;y]; %#ok<AGROW>
end
active=abs(commands)>=0.05;
if any(active); signPercent=100*mean(sign(commands(active))==sign(delivered(active)));
else; signPercent=NaN; end
residual=zeros(numel(time),2);
if useRL
    [residualTime,residualRaw]=loggedMatrix( ...
        output.logsout,"rl_residual_hip_torque_Nm");
    residualRaw=requireWidth(residualRaw,2);
    residual=interp1(residualTime,residualRaw,time,"previous","extrap");
end
x=struct("time",time,"q",q,"motorTrackingRMS_Nm",rms(errors), ...
    "motorTrackingPeak_Nm",max(abs(errors)), ...
    "motorSameSignPercent",signPercent, ...
    "peakDeliveredTorque_Nm",max(abs(delivered)), ...
    "motorTorqueLimitReached",max(abs(delivered))>=99, ...
    "residualRMS_Nm",rms(residual,"all"), ...
    "residualPeak_Nm",max(abs(residual),[],"all"), ...
    "peakNormalizedAction",max(abs(residual),[],"all")/limit, ...
    "residualNearBoundPercent",100*mean(any(abs(residual)>=0.99*limit,2)));
end

function [tags,c,d]=motorSignalMap()
tags=["right_hip_transverse";"right_hip_frontal";"right_hip_sagittal"; ...
    "right_knee";"right_ankle_roll";"right_ankle_pitch"; ...
    "left_hip_transverse";"left_hip_frontal";"left_hip_sagittal"; ...
    "left_knee";"left_ankle_roll";"left_ankle_pitch"; ...
    "right_shoulder_frontal";"left_shoulder_frontal"];
base="motorDrive"+upper(extractBefore(tags,2))+extractAfter(tags,1);
c=matlab.lang.makeValidName(base+"Command");
d=matlab.lang.makeValidName(base+"Delivered");
c(tags=="right_hip_sagittal")="motorDriveRightHipSagittalCommand";
d(tags=="right_hip_sagittal")="motorDriveRightHipSagittalDelivered";
c(tags=="left_hip_sagittal")="motorDriveLeftHipSagittalCommand";
d(tags=="left_hip_sagittal")="motorDriveLeftHipSagittalDelivered";
end

function values=requireWidth(values,n)
if size(values,2)~=n && size(values,1)==n; values=values.'; end
assert(size(values,2)==n,"MotorBenchmark:Width", ...
    "Expected width %d; received %d.",n,size(values,2));
end

function ratio=safeRatio(late,early)
if early<=eps; if late<=eps; ratio=0; else; ratio=Inf; end
else; ratio=late/early; end
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
        [ti,vi]=flatten(x.getElement(index));
        if isempty(time); time=ti(:); values=vi;
        else; values=[values,align(ti,vi,time)]; end %#ok<AGROW>
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
        try; [ti,vi]=flatten(x.(f(index)));
        catch exception
            if exception.identifier=="MotorBenchmark:Payload"; continue; end
            rethrow(exception)
        end
        if isempty(time); time=ti(:); values=vi;
        else; values=[values,align(ti,vi,time)]; end %#ok<AGROW>
    end
    if ~isempty(time); return; end
end
error("MotorBenchmark:Payload","Unsupported payload %s.",class(x));
end

function values=align(sourceTime,sourceValues,targetTime)
sourceTime=sourceTime(:);
if isscalar(sourceTime); values=repmat(sourceValues(1,:),numel(targetTime),1);
else; values=interp1(sourceTime,sourceValues,targetTime,"linear","extrap"); end
end
