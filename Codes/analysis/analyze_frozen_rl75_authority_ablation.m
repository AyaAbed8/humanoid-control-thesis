function analysis = analyze_frozen_rl75_authority_ablation( ...
        resultDirectory, showFigures)
%ANALYZE_FROZEN_RL75_AUTHORITY_ABLATION Inspect saved authority evidence.
%   This function performs read-only post-processing. It does not run a
%   simulation, load or save a Simulink model, or modify a controller.
%
%   It checks three questions before matched-bound retraining:
%     1. How do +/-5, +/-10 and +/-15 Nm affect nominal standing?
%     2. How do the boundary-defining +150/+185 N trajectories differ?
%     3. Which final actuator saturates, when, and relative to fall onset?

arguments
    resultDirectory (1,1) string = fullfile( ...
        project_codes_root(),"Results", ...
        "FrozenRL75AuthorityAblation","20260812_110700")
    showFigures (1,1) logical = true
end

resultFile=fullfile(resultDirectory,"processed_results.mat");
assert(isfile(resultFile),"AuthorityAnalysis:ResultsMissing", ...
    "Saved authority-ablation results were not found: %s",resultFile);
saved=load(resultFile,"runs","combined","summary","torqueLimits_Nm");
assert(isfield(saved,"runs") && numel(saved.runs)==3, ...
    "AuthorityAnalysis:UnexpectedSavedFormat", ...
    "Expected three saved authority runs in %s.",resultFile);

outputDirectory=fullfile(resultDirectory,"postprocessing");
if ~isfolder(outputDirectory); mkdir(outputDirectory); end

limits=saved.torqueLimits_Nm(:);
nominalMetrics=makeNominalMetrics(saved.runs,limits);
saturationEvents=makeSaturationEvents(saved.runs,limits);
representativeCases=table([5;10;15],[150;185;185], ...
    'VariableNames',{'ResidualTorqueLimit_Nm','SignedForce_N'});

writetable(nominalMetrics,fullfile(outputDirectory, ...
    "nominal_standing_metrics.csv"));
writetable(saturationEvents,fullfile(outputDirectory, ...
    "saturation_chronology.csv"));
writetable(representativeCases,fullfile(outputDirectory, ...
    "representative_case_definitions.csv"));

[stateFigure,torqueFigure]=makeRepresentativePlots( ...
    saved.runs,limits,representativeCases,showFigures);
exportgraphics(stateFigure,fullfile(outputDirectory, ...
    "representative_state_and_residual_trajectories.png"), ...
    "Resolution",200);
savefig(stateFigure,fullfile(outputDirectory, ...
    "representative_state_and_residual_trajectories.fig"));
exportgraphics(torqueFigure,fullfile(outputDirectory, ...
    "representative_action_and_actuator_trajectories.png"), ...
    "Resolution",200);
savefig(torqueFigure,fullfile(outputDirectory, ...
    "representative_action_and_actuator_trajectories.fig"));

save(fullfile(outputDirectory,"postprocessed_analysis.mat"), ...
    "nominalMetrics","saturationEvents","representativeCases", ...
    "resultDirectory");

analysis=struct("resultDirectory",string(resultDirectory), ...
    "outputDirectory",string(outputDirectory), ...
    "nominalMetrics",nominalMetrics, ...
    "saturationEvents",saturationEvents, ...
    "representativeCases",representativeCases);

fprintf("\nFrozen RL75 authority-ablation post-processing\n");
fprintf("No simulations were run. Source: %s\n",resultDirectory);
fprintf("\nNominal-standing comparison:\n");
disp(nominalMetrics);
fprintf("\nSaturation chronology (all tested disturbances):\n");
disp(saturationEvents);
fprintf("Post-processing written to %s\n",outputDirectory);
end

function metrics=makeNominalMetrics(runs,limits)
n=numel(limits);
postureRMS=zeros(n,1); maxJointDeviation=zeros(n,1);
torsoLateralRMS=zeros(n,1); torsoTiltRMS=zeros(n,1);
residualRMS=zeros(n,1); residualPeak=zeros(n,1);
normalizedActionRMS=zeros(n,1); normalizedActionPeak=zeros(n,1);
appliedRMS=zeros(n,1); appliedPeak=zeros(n,1);
for index=1:n
    [trajectory,~]=caseTrajectory(runs{index},"nominal",0);
    qDeviation=trajectory.q(:,11:22)-trajectory.q(1,11:22);
    postureRMS(index)=rad2deg(rmsAll(qDeviation));
    maxJointDeviation(index)=max(abs(rad2deg(qDeviation)),[],"all");
    torsoLateralRMS(index)=rmsAll(trajectory.q(:,1)-trajectory.q(1,1));
    torsoTiltRMS(index)=rad2deg(rmsAll( ...
        trajectory.q(:,5)-trajectory.q(1,5)));
    residualRMS(index)=rmsAll(trajectory.residualTorque);
    residualPeak(index)=max(abs(trajectory.residualTorque),[],"all");
    normalizedActionRMS(index)=rmsAll(trajectory.action);
    normalizedActionPeak(index)=max(abs(trajectory.action),[],"all");
    appliedRMS(index)=rmsAll(trajectory.totalTorque);
    appliedPeak(index)=max(abs(trajectory.totalTorque),[],"all");
end
metrics=table(limits,postureRMS,maxJointDeviation,torsoLateralRMS, ...
    torsoTiltRMS,residualRMS,residualPeak,normalizedActionRMS, ...
    normalizedActionPeak,appliedRMS,appliedPeak,'VariableNames', ...
    {'ResidualTorqueLimit_Nm','PostureRMSFromInitial_deg', ...
    'MaxJointDeviationFromInitial_deg','TorsoLateralRMSFromInitial_m', ...
    'TorsoLateralTiltRMSFromInitial_deg','ResidualTorqueRMS_Nm', ...
    'PeakResidualTorque_Nm','NormalizedActionRMS', ...
    'PeakNormalizedAction','AppliedTorqueRMS_Nm', ...
    'PeakAppliedTorque_Nm'});
end

function events=makeSaturationEvents(runs,limits)
jointNames=["right_hip_frontal";"right_knee"; ...
    "right_ankle_pitch";"right_hip_sagittal"; ...
    "right_hip_transverse";"right_ankle_roll"; ...
    "left_hip_frontal";"left_knee";"left_ankle_pitch"; ...
    "left_hip_sagittal";"left_hip_transverse";"left_ankle_roll"];
rows=cell(0,1); rowIndex=0;
for limitIndex=1:numel(limits)
    run=runs{limitIndex};
    p=run.protocol;
    definitions=run.definitions(run.definitions.SignedForce_N~=0,:);
    for definitionIndex=1:height(definitions)
        d=definitions(definitionIndex,:);
        [x,metric]=caseTrajectory(run,d.Axis,d.SignedForce_N);
        absoluteTorque=abs(x.totalTorque);
        [peakTorque,linearPeak]=max(absoluteTorque,[],"all");
        [peakSample,peakChannel]=ind2sub(size(absoluteTorque),linearPeak);
        saturationMask=absoluteTorque>=100-p.saturationTolerance_Nm;
        firstLinear=find(saturationMask,1,"first");
        if isempty(firstLinear)
            firstTime=NaN; firstJoint="none"; firstTorque=NaN;
        else
            [firstSample,firstChannel]=ind2sub(size(saturationMask),firstLinear);
            firstTime=x.torqueTime(firstSample);
            firstJoint=jointNames(firstChannel);
            firstTorque=x.totalTorque(firstSample,firstChannel);
        end
        fallTime=firstFallTime(x,p);
        if isnan(firstTime); timing="no_saturation";
        elseif isnan(fallTime); timing="saturation_without_classified_fall";
        elseif firstTime<fallTime-1e-9; timing="before_fall_threshold";
        elseif abs(firstTime-fallTime)<=1e-9; timing="at_fall_threshold";
        else; timing="after_fall_threshold"; end
        rowIndex=rowIndex+1;
        rows{rowIndex,1}=table(limits(limitIndex),d.SignedForce_N, ...
            metric.Outcome,metric.Fell,metric.Recovered, ...
            logical(any(saturationMask,"all")),firstTime,firstJoint, ...
            firstTorque,fallTime,string(timing),peakTorque, ...
            jointNames(peakChannel),x.torqueTime(peakSample), ...
            'VariableNames',{'ResidualTorqueLimit_Nm','SignedForce_N', ...
            'Outcome','Fell','Recovered','FinalActuatorSaturation', ...
            'FirstSaturationTime_s','FirstSaturatedJoint', ...
            'FirstSaturatedTorque_Nm','FirstFallThresholdTime_s', ...
            'SaturationRelativeToFall','PeakAppliedTorque_Nm', ...
            'PeakAppliedTorqueJoint','PeakAppliedTorqueTime_s'});
    end
end
events=vertcat(rows{:});
end

function time=firstFallTime(x,p)
q=x.q; initial=q(1,:);
verticalDrop=initial(3)-q(:,3);
horizontalTravel=vecnorm(q(:,1:2)-initial(1:2),2,2);
rotation=max(abs(rad2deg(q(:,4:6)-initial(4:6))),[],2);
index=find(verticalDrop>p.fallVerticalDrop_m | ...
    horizontalTravel>p.fallHorizontalTravel_m | ...
    rotation>p.fallRotation_deg,1,"first");
if isempty(index); time=NaN; else; time=x.qTime(index); end
end

function [stateFigure,torqueFigure]=makeRepresentativePlots( ...
        runs,limits,cases,showFigures)
visibility="off"; if showFigures; visibility="on"; end
colors=lines(3);
stateFigure=figure("Color","w","Visible",visibility, ...
    "Position",[60 60 1120 780]);
stateLayout=tiledlayout(3,3,"TileSpacing","compact", ...
    "Padding","compact");
title(stateLayout,"Frozen RL75: representative authority-limited responses");
torqueFigure=figure("Color","w","Visible",visibility, ...
    "Position",[90 90 1120 780]);
torqueLayout=tiledlayout(3,3,"TileSpacing","compact", ...
    "Padding","compact");
title(torqueLayout,"Normalized actions and final actuator torques");
for index=1:height(cases)
    runIndex=find(limits==cases.ResidualTorqueLimit_Nm(index),1);
    run=runs{runIndex};
    [x,metric]=caseTrajectory(run,"lateral",cases.SignedForce_N(index));
    [nominal,~]=caseTrajectory(run,"nominal",0);
    qNominal=interp1(nominal.qTime,nominal.q,x.qTime,"linear","extrap");
    displacement=x.q(:,1)-qNominal(:,1);
    tilt=rad2deg(x.q(:,5)-qNominal(:,5));
    label=sprintf("+/-%g Nm, %+g N: %s",limits(runIndex), ...
        cases.SignedForce_N(index),metric.Outcome);

    figure(stateFigure);
    nexttile((index-1)*3+1); plot(x.qTime,displacement, ...
        "LineWidth",1.4,"Color",colors(index,:)); grid on
    ylabel("x displacement (m)"); title(label); xline(2,":"); xline(2.1,":");
    nexttile((index-1)*3+2); plot(x.qTime,tilt, ...
        "LineWidth",1.4,"Color",colors(index,:)); grid on
    ylabel("Lateral tilt (deg)"); xline(2,":"); xline(2.1,":");
    nexttile((index-1)*3+3); plot(x.residualTime,x.residualTorque, ...
        "LineWidth",1.2); grid on
    yline(limits(runIndex),":"); yline(-limits(runIndex),":");
    ylabel("Residual torque (Nm)");
    legend("Right hip","Left hip","Location","best");

    figure(torqueFigure);
    nexttile((index-1)*3+1); plot(x.actionTime,x.action, ...
        "LineWidth",1.2); grid on; ylim([-1.05 1.05]);
    ylabel("Normalized action"); title(label);
    legend("Right hip","Left hip","Location","best");
    nexttile((index-1)*3+2); plot(x.torqueTime,x.totalTorque, ...
        "LineWidth",0.8); grid on; yline(100,":"); yline(-100,":");
    ylabel("All applied torques (Nm)");
    nexttile((index-1)*3+3); plot(x.torqueTime, ...
        max(abs(x.totalTorque),[],2),"k","LineWidth",1.4); grid on
    yline(100,":r"); ylabel("Max |applied torque| (Nm)");
end
xlabel(stateLayout,"Time (s)"); xlabel(torqueLayout,"Time (s)");
end

function [trajectory,metric]=caseTrajectory(run,axisName,signedForce)
index=find(run.definitions.Axis==string(axisName) & ...
    run.definitions.SignedForce_N==signedForce,1);
assert(~isempty(index),"AuthorityAnalysis:CaseMissing", ...
    "Saved case %s, force %+.6g N was not found.",axisName,signedForce);
trajectory=run.trajectories{index};
metricIndex=run.primaryMetrics.CaseNumber==run.definitions.CaseNumber(index) & ...
    run.primaryMetrics.Horizon_s==12 & ...
    run.primaryMetrics.ThresholdVariant=="central";
metric=run.primaryMetrics(metricIndex,:);
assert(height(metric)==1,"AuthorityAnalysis:MetricMissing", ...
    "Expected one central 12 s metric row for the selected case.");
end

function value=rmsAll(values)
value=sqrt(mean(values.^2,"all","omitnan"));
end
