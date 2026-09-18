function analysis = analyze_frozen_rl75_learned_behavior( ...
        sourceDirectory,outputRoot,showFigures)
%ANALYZE_FROZEN_RL75_LEARNED_BEHAVIOR Interpret saved RL75 trajectories.
%   This is read-only post-processing: no simulation or model modification.
%   Actions and states are compared with RL75's matched nominal trajectory,
%   which separates push-induced behaviour from its nonzero nominal action.
%
%   The saved evidence does not include the two foot-load observations.
%   Correlations are therefore descriptive closed-loop associations, not a
%   full surrogate model or evidence of causality.

arguments
    sourceDirectory (1,1) string = fullfile( ...
        project_codes_root(),"Results", ...
        "FrozenRL75AuthorityAblation","20260812_110700")
    outputRoot (1,1) string = fullfile( ...
        project_codes_root(),"Results", ...
        "ResidualRLPolicyInterpretation","20260817_final")
    showFigures (1,1) logical = true
end

sourceFile=fullfile(sourceDirectory,"processed_results.mat");
assert(isfile(sourceFile),"RL75Interpretation:EvidenceMissing", ...
    "Saved RL75 authority evidence was not found: %s",sourceFile);
saved=load(sourceFile,"runs","torqueLimits_Nm");
limitIndex=find(saved.torqueLimits_Nm==15,1);
assert(~isempty(limitIndex),"RL75Interpretation:LimitMissing", ...
    "The saved evidence does not contain the frozen +/-15 Nm policy.");
run=saved.runs{limitIndex};
assert(height(run.definitions)==7 && numel(run.trajectories)==7, ...
    "RL75Interpretation:UnexpectedCaseCount", ...
    "Expected nominal and +/-[100 150 185] N trajectories.");
if ~isfolder(outputRoot); mkdir(outputRoot); end

nominalIndex=find(run.definitions.Axis=="nominal" & ...
    run.definitions.SignedForce_N==0,1);
assert(~isempty(nominalIndex),"RL75Interpretation:NominalMissing", ...
    "Matched nominal RL75 trajectory is missing.");
nominal=run.trajectories{nominalIndex};

caseRows=cell(height(run.definitions),1);
correlationRows=cell(0,1); correlationIndex=0;
derived=cell(height(run.definitions),1);
for index=1:height(run.definitions)
    definition=run.definitions(index,:);
    trajectory=run.trajectories{index};
    derived{index}=deriveSignals(trajectory,nominal);
    metric=centralMetric(run,definition.CaseNumber);
    caseRows{index}=caseMetrics(definition,metric,derived{index},15);
    if definition.SignedForce_N~=0
        rows=closedLoopCorrelations(definition,derived{index});
        for rowIndex=1:numel(rows)
            correlationIndex=correlationIndex+1;
            correlationRows{correlationIndex,1}=rows{rowIndex};
        end
    end
end
caseActionMetrics=vertcat(caseRows{:});
stateActionCorrelations=vertcat(correlationRows{:});
stateActionCorrelations.AbsolutePearsonCorrelation=abs( ...
    stateActionCorrelations.PearsonCorrelation);
correlationSummary=groupsummary(stateActionCorrelations, ...
    ["State","ActionMode"],["mean","max"], ...
    "AbsolutePearsonCorrelation");
directionalSymmetry=makeDirectionalSymmetry(run,derived);

sourceManifest=table(sourceFile,fileHash(sourceFile),15, ...
    'VariableNames',{'EvidenceFile','SHA256','ResidualTorqueLimit_Nm'});
writetable(caseActionMetrics,fullfile(outputRoot,"case_action_metrics.csv"));
writetable(stateActionCorrelations,fullfile(outputRoot, ...
    "closed_loop_state_action_correlations.csv"));
writetable(correlationSummary,fullfile(outputRoot, ...
    "closed_loop_correlation_summary.csv"));
writetable(directionalSymmetry,fullfile(outputRoot, ...
    "directional_action_symmetry.csv"));
writetable(sourceManifest,fullfile(outputRoot,"source_manifest.csv"));

[responseFigure,symmetryFigure]=makeFigures( ...
    run,derived,showFigures);
saveFigure(responseFigure,outputRoot,"representative_policy_response");
saveFigure(symmetryFigure,outputRoot,"directional_action_symmetry");

limitations=[ ...
    "This analysis uses saved deterministic closed-loop trajectories; no simulations were rerun."; ...
    "Push-induced actions and states are defined relative to RL75's matched nominal trajectory."; ...
    "The policy used 12 observations, but saved evidence does not contain the two foot-load observations."; ...
    "State-action correlations are descriptive and must not be interpreted as causal policy derivatives."; ...
    "The analysis concerns the selected seed-0 episode-75 policy with a +/-15 Nm residual bound."];
writelines(limitations,fullfile(outputRoot,"interpretation_limitations.txt"));
disturbed=caseActionMetrics.SignedForce_N~=0;
differentialEnergyFraction=1-caseActionMetrics.CommonModeEnergyFraction(disturbed);
differentialSummary=correlationSummary( ...
    correlationSummary.ActionMode=="differential_action",:);
[~,tiltRateRow]=max( ...
    differentialSummary.mean_AbsolutePearsonCorrelation);
interpretation=[ ...
    compose("Nominal normalized action RMS was %.4f (%.3f Nm at the 15 Nm scale).", ...
        caseActionMetrics.ActualActionRMS(1), ...
        15*caseActionMetrics.ActualActionRMS(1)); ...
    compose("Disturbed-case peak action occurred %.3f-%.3f s after push onset.", ...
        min(caseActionMetrics.PeakActionDelayFromPush_s(disturbed)), ...
        max(caseActionMetrics.PeakActionDelayFromPush_s(disturbed))); ...
    compose("Near-bound action duration increased across the tested cases from %.3f to %.3f s.", ...
        min(caseActionMetrics.NearBoundDuration_s(disturbed)), ...
        max(caseActionMetrics.NearBoundDuration_s(disturbed))); ...
    compose("Differential bilateral action represented %.1f-%.1f%% of induced action-mode energy.", ...
        100*min(differentialEnergyFraction), ...
        100*max(differentialEnergyFraction)); ...
    compose("The strongest mean closed-loop differential-action association was with %s (mean |r| = %.3f).", ...
        differentialSummary.State(tiltRateRow), ...
        differentialSummary.mean_AbsolutePearsonCorrelation(tiltRateRow)); ...
    compose("Two-channel direction-reversal NRMSE ranged from %.3f to %.3f (zero would be perfect sign antisymmetry).", ...
        min(directionalSymmetry.TwoChannelAntiSymmetryNRMSE), ...
        max(directionalSymmetry.TwoChannelAntiSymmetryNRMSE)); ...
    "Interpretation: RL75 provides a rapid, mostly differential and rate-associated correction, but is neither inactive nominally nor perfectly symmetric between push directions."];
writelines(interpretation,fullfile(outputRoot,"scientific_interpretation.txt"));
save(fullfile(outputRoot,"processed_policy_interpretation.mat"), ...
    "caseActionMetrics","stateActionCorrelations","correlationSummary", ...
    "directionalSymmetry","sourceManifest","sourceDirectory");

analysis=struct("sourceDirectory",sourceDirectory, ...
    "outputDirectory",outputRoot, ...
    "caseActionMetrics",caseActionMetrics, ...
    "stateActionCorrelations",stateActionCorrelations, ...
    "correlationSummary",correlationSummary, ...
    "directionalSymmetry",directionalSymmetry, ...
    "sourceManifest",sourceManifest);

fprintf("\nFrozen RL75 learned-behaviour analysis\n");
fprintf("No simulations were run. Source: %s\n",sourceDirectory);
fprintf("\nAction timing and coordination:\n");
disp(caseActionMetrics(:,["SignedForce_N","Outcome", ...
    "ActualActionRMS","PushInducedActionRMS", ...
    "PeakPushInducedAction","PeakActionDelayFromPush_s", ...
    "NearBoundDuration_s","CommonModeEnergyFraction"]));
fprintf("\nDirectional symmetry:\n");
disp(directionalSymmetry);
fprintf("Results written to %s\n",outputRoot);
end

function signals=deriveSignals(trajectory,nominal)
t=trajectory.actionTime(:);
action=sampleRows(trajectory.action,numel(t));
nominalAction=interp1(nominal.actionTime(:), ...
    sampleRows(nominal.action,numel(nominal.actionTime)),t, ...
    "linear","extrap");
q=interp1(trajectory.qTime(:),sampleRows(trajectory.q, ...
    numel(trajectory.qTime)),t,"linear","extrap");
qNominal=interp1(nominal.qTime(:),sampleRows(nominal.q, ...
    numel(nominal.qTime)),t,"linear","extrap");
dq=q-qNominal;

signals=struct();
signals.time=t;
signals.action=action;
signals.nominalAction=nominalAction;
signals.inducedAction=action-nominalAction;
signals.residualTorque=interp1(trajectory.residualTime(:), ...
    sampleRows(trajectory.residualTorque, ...
    numel(trajectory.residualTime)),t,"linear","extrap");
signals.lateralDisplacement=dq(:,1);
signals.lateralVelocity=gradient(dq(:,1),t);
signals.lateralTilt=rad2deg(dq(:,5));
signals.lateralTiltRate=rad2deg(gradient(dq(:,5),t));
signals.rightHipAngle=rad2deg(dq(:,13));
signals.rightHipRate=rad2deg(gradient(dq(:,13),t));
signals.leftHipAngle=rad2deg(dq(:,19));
signals.leftHipRate=rad2deg(gradient(dq(:,19),t));
signals.commonAction=mean(signals.inducedAction,2);
signals.differentialAction=0.5*( ...
    signals.inducedAction(:,1)-signals.inducedAction(:,2));
end

function row=caseMetrics(definition,metric,s,torqueLimit)
post=s.time>=2;
nearBound=any(abs(s.action)>=0.95,2) & post;
sampleTime=median(diff(s.time));
[peakInduced,linearIndex]=max(abs(s.inducedAction(post,:)),[],"all");
postIndices=find(post);
[peakSample,peakChannel]=ind2sub( ...
    size(s.inducedAction(post,:)),linearIndex);
peakTime=s.time(postIndices(peakSample));
commonRMS=rmsAll(s.commonAction(post));
differentialRMS=rmsAll(s.differentialAction(post));
modeEnergy=commonRMS^2+differentialRMS^2;
if modeEnergy>0
    commonEnergyFraction=commonRMS^2/modeEnergy;
else
    commonEnergyFraction=NaN;
end
closureError=max(abs(s.residualTorque-torqueLimit*s.action),[],"all");

row=table(definition.CaseNumber,definition.Axis, ...
    definition.SignedForce_N,metric.Outcome, ...
    rmsAll(s.action),rmsAll(s.inducedAction(post,:)), ...
    max(abs(s.action),[],"all"),peakInduced,peakChannel, ...
    peakTime-2,100*mean(nearBound(post)),sum(nearBound)*sampleTime, ...
    commonRMS,differentialRMS,commonEnergyFraction,closureError, ...
    rmsAll(s.lateralDisplacement(post)), ...
    rmsAll(s.lateralTilt(post)),torqueLimit, ...
    'VariableNames',{'CaseNumber','Axis','SignedForce_N','Outcome', ...
    'ActualActionRMS','PushInducedActionRMS', ...
    'PeakAbsoluteAction','PeakPushInducedAction', ...
    'PeakPushInducedActionChannel','PeakActionDelayFromPush_s', ...
    'NearBoundSamplePercent','NearBoundDuration_s', ...
    'CommonModeActionRMS','DifferentialModeActionRMS', ...
    'CommonModeEnergyFraction','ResidualActionClosureMaxError_Nm', ...
    'PostPushLateralDisplacementRMS_m', ...
    'PostPushLateralTiltRMS_deg','ResidualTorqueLimit_Nm'});
end

function rows=closedLoopCorrelations(definition,s)
post=s.time>=2;
stateNames=["lateral_displacement","lateral_velocity", ...
    "lateral_tilt","lateral_tilt_rate","right_hip_angle", ...
    "right_hip_rate","left_hip_angle","left_hip_rate"];
states=[s.lateralDisplacement,s.lateralVelocity,s.lateralTilt, ...
    s.lateralTiltRate,s.rightHipAngle,s.rightHipRate, ...
    s.leftHipAngle,s.leftHipRate];
actionNames=["right_action","left_action","common_action", ...
    "differential_action"];
actions=[s.inducedAction,s.commonAction,s.differentialAction];
rows=cell(numel(stateNames)*numel(actionNames),1); rowIndex=0;
for stateIndex=1:numel(stateNames)
    for actionIndex=1:numel(actionNames)
        rowIndex=rowIndex+1;
        coefficient=safeCorrelation( ...
            states(post,stateIndex),actions(post,actionIndex));
        rows{rowIndex}=table(definition.SignedForce_N, ...
            stateNames(stateIndex),actionNames(actionIndex),coefficient, ...
            'VariableNames',{'SignedForce_N','State','ActionMode', ...
            'PearsonCorrelation'});
    end
end
end

function symmetry=makeDirectionalSymmetry(run,derived)
levels=[100;150;185]; antiSymmetryNRMSE=zeros(3,1);
commonAntiSymmetryNRMSE=zeros(3,1);
differentialAntiSymmetryNRMSE=zeros(3,1);
for index=1:numel(levels)
    positive=find(run.definitions.SignedForce_N==levels(index),1);
    negative=find(run.definitions.SignedForce_N==-levels(index),1);
    p=derived{positive}; n=derived{negative};
    nAction=interp1(n.time,n.inducedAction,p.time,"linear","extrap");
    nCommon=interp1(n.time,n.commonAction,p.time,"linear","extrap");
    nDifferential=interp1(n.time,n.differentialAction, ...
        p.time,"linear","extrap");
    selected=p.time>=2;
    antiSymmetryNRMSE(index)=normalizedMirrorError( ...
        p.inducedAction(selected,:),nAction(selected,:));
    commonAntiSymmetryNRMSE(index)=normalizedMirrorError( ...
        p.commonAction(selected),nCommon(selected));
    differentialAntiSymmetryNRMSE(index)=normalizedMirrorError( ...
        p.differentialAction(selected),nDifferential(selected));
end
symmetry=table(levels,antiSymmetryNRMSE,commonAntiSymmetryNRMSE, ...
    differentialAntiSymmetryNRMSE,'VariableNames', ...
    {'ForceMagnitude_N','TwoChannelAntiSymmetryNRMSE', ...
    'CommonModeAntiSymmetryNRMSE','DifferentialModeAntiSymmetryNRMSE'});
end

function [responseFigure,symmetryFigure]=makeFigures(run,derived,showFigures)
visibility="off"; if showFigures; visibility="on"; end
selectedForces=[150 -150 185];
responseFigure=figure(Color="white",Visible=visibility, ...
    Position=[50 50 1150 800]);
layout=tiledlayout(4,3,TileSpacing="compact",Padding="compact");
title(layout,"Frozen RL75: state-dependent residual response");
colors=lines(2);
for column=1:numel(selectedForces)
    index=find(run.definitions.SignedForce_N==selectedForces(column),1);
    s=derived{index};
    metric=centralMetric(run,run.definitions.CaseNumber(index));
    label=sprintf("%+g N: %s",selectedForces(column),metric.Outcome);
    nexttile(column); plot(s.time,s.lateralDisplacement,"LineWidth",1.4);
    title(label); ylabel("Lateral displacement (m)"); grid on; markPush();
    nexttile(3+column); plot(s.time,s.lateralTilt,"LineWidth",1.4);
    ylabel("Lateral tilt (deg)"); grid on; markPush();
    nexttile(6+column); hold on
    plot(s.time,s.inducedAction(:,1),"LineWidth",1.2,Color=colors(1,:));
    plot(s.time,s.inducedAction(:,2),"LineWidth",1.2,Color=colors(2,:));
    ylabel("Push-induced action"); grid on; markPush();
    if column==3
        legend("Right hip","Left hip",Location="best");
    end
    nexttile(9+column); hold on
    plot(s.time,s.commonAction,"LineWidth",1.2);
    plot(s.time,s.differentialAction,"LineWidth",1.2);
    yline(1,":"); yline(-1,":");
    xlabel("Time (s)"); ylabel("Action modes"); grid on; markPush();
    if column==3
        legend("Common","Differential",Location="best");
    end
end
disableToolbars(responseFigure);

symmetryFigure=figure(Color="white",Visible=visibility, ...
    Position=[80 80 1100 650]);
symmetryLayout=tiledlayout(2,3,TileSpacing="compact",Padding="compact");
title(symmetryLayout,"Direction reversal: positive response versus mirrored negative response");
levels=[100 150 185];
for column=1:numel(levels)
    positive=find(run.definitions.SignedForce_N==levels(column),1);
    negative=find(run.definitions.SignedForce_N==-levels(column),1);
    p=derived{positive}; n=derived{negative};
    nexttile(column); hold on
    plot(p.time,p.commonAction,"LineWidth",1.3);
    plot(n.time,-n.commonAction,"LineWidth",1.3);
    title(sprintf("%g N common mode",levels(column)));
    ylabel("Induced action"); grid on; markPush();
    if column==3; legend("+F","mirrored -F",Location="best"); end
    nexttile(3+column); hold on
    plot(p.time,p.differentialAction,"LineWidth",1.3);
    plot(n.time,-n.differentialAction,"LineWidth",1.3);
    title(sprintf("%g N differential mode",levels(column)));
    xlabel("Time (s)"); ylabel("Induced action"); grid on; markPush();
end
disableToolbars(symmetryFigure);
end

function markPush()
xline(2,":k"); xline(2.1,":k");
end

function disableToolbars(figureHandle)
axesHandles=findall(figureHandle,Type="axes");
for index=1:numel(axesHandles)
    axesHandles(index).Toolbar.Visible="off";
end
end

function saveFigure(handle,outputRoot,name)
exportgraphics(handle,fullfile(outputRoot,name+".png"),Resolution=250);
savefig(handle,fullfile(outputRoot,name+".fig"));
end

function metric=centralMetric(run,caseNumber)
selected=run.primaryMetrics.CaseNumber==caseNumber & ...
    run.primaryMetrics.Horizon_s==12 & ...
    run.primaryMetrics.ThresholdVariant=="central";
metric=run.primaryMetrics(selected,:);
assert(height(metric)==1,"RL75Interpretation:MetricMissing", ...
    "Expected one central 12 s metric row for case %d.",caseNumber);
end

function values=sampleRows(values,count)
values=squeeze(values);
if isvector(values)
    values=values(:);
elseif size(values,1)~=count && size(values,2)==count
    values=values.';
end
assert(size(values,1)==count,"RL75Interpretation:SampleCount", ...
    "Logged signal has %d samples; expected %d.",size(values,1),count);
end

function coefficient=safeCorrelation(x,y)
if std(x,"omitnan")<eps || std(y,"omitnan")<eps
    coefficient=NaN;
else
    matrix=corrcoef(x,y,"Rows","complete");
    coefficient=matrix(1,2);
end
end

function error=normalizedMirrorError(positive,negative)
numerator=rmsAll(positive+negative);
denominator=sqrt(0.5*(rmsAll(positive)^2+rmsAll(negative)^2));
if denominator>eps; error=numerator/denominator; else; error=NaN; end
end

function value=rmsAll(values)
value=sqrt(mean(values.^2,"all","omitnan"));
end

function hash=fileHash(path)
hash=release_file_sha256(path);
end
