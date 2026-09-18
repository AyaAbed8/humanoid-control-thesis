function results = run_frozen_rl75_authority_ablation( ...
        torqueLimits_Nm, outputRoot)
%RUN_FROZEN_RL75_AUTHORITY_ABLATION Clip one frozen policy at new bounds.
%   This is a deployment-time, same-policy ablation. It does not train an
%   agent and does not edit either Simulink model. The exact frozen seed-0
%   RL75 checkpoint is evaluated with physical residual-torque limits of
%   +/-5, +/-10 and +/-15 Nm by default. All other controller, disturbance,
%   model, solver, logging and recovery-classification settings are held
%   fixed under the final 12 s protocol.
%
%   The force cases are nominal standing and lateral +/-100, +/-150 and
%   +/-185 N. They respectively probe Controller A's established boundary,
%   the original RL comparison level and RL75's demonstrated boundary.

arguments
    torqueLimits_Nm (1,:) double ...
        {mustBeFinite,mustBePositive} = [5 10 15]
    outputRoot (1,1) string = fullfile( ...
        project_codes_root(),"Results", ...
        "FrozenRL75AuthorityAblation")
end

assert(numel(unique(torqueLimits_Nm))==numel(torqueLimits_Nm), ...
    "AuthorityAblation:DuplicateLimit", ...
    "Every requested residual-torque limit must be unique.");

stamp=string(datetime("now","Format","yyyyMMdd_HHmmss"));
resultDirectory=fullfile(outputRoot,stamp);
mkdir(resultDirectory);

fprintf("\nFrozen RL75 same-policy residual-authority ablation\n");
fprintf("Deployment limits: +/- %s Nm\n",mat2str(torqueLimits_Nm));
fprintf("Policy weights: unchanged frozen seed-0 RL75 checkpoint\n");
fprintf("Cases per limit: nominal, +/-100, +/-150 and +/-185 N lateral\n");
fprintf("Output: %s\n\n",resultDirectory);

runs=cell(numel(torqueLimits_Nm),1);
combined=table();
for index=1:numel(torqueLimits_Nm)
    limit=torqueLimits_Nm(index);
    fprintf("[%d/%d] Evaluating frozen RL75 at +/-%g Nm\n", ...
        index,numel(torqueLimits_Nm),limit);
    runRoot=fullfile(resultDirectory,sprintf("limit_%gNm",limit));
    runs{index}=analyze_benchmark_protocol_sensitivity( ...
        runRoot,"rl75_authority",limit,index==1);
    metrics=runs{index}.primaryMetrics;
    metrics.ResidualTorqueLimit_Nm=repmat(limit,height(metrics),1);
    metrics=movevars(metrics,"ResidualTorqueLimit_Nm","Before","Controller");
    combined=[combined;metrics]; %#ok<AGROW>
    save(fullfile(resultDirectory,"simulation_checkpoint.mat"), ...
        "runs","combined","torqueLimits_Nm","-v7.3");
end

summary=summarizeAuthority(combined,torqueLimits_Nm);
writetable(combined,fullfile(resultDirectory,"authority_metrics.csv"));
writetable(summary,fullfile(resultDirectory,"authority_summary.csv"));
save(fullfile(resultDirectory,"processed_results.mat"), ...
    "runs","combined","summary","torqueLimits_Nm","-v7.3");
makeAuthorityPlot(summary,resultDirectory);

results=struct("resultDirectory",string(resultDirectory), ...
    "metrics",combined,"summary",summary,"runs",{runs}, ...
    "torqueLimits_Nm",torqueLimits_Nm, ...
    "interpretation", ...
    "Same frozen policy; differences isolate deployed residual authority, not retraining.");

fprintf("\nAuthority summary:\n");
disp(summary);
fprintf("Results written to %s\n",resultDirectory);
end

function summary=summarizeAuthority(metrics,limits)
n=numel(limits);
recovered=zeros(n,1); upright=zeros(n,1); fell=zeros(n,1);
maximumBidirectional=nan(n,1); peakResidual=zeros(n,1);
peakNormalized=zeros(n,1); saturation=false(n,1);
for index=1:n
    selected=metrics.ResidualTorqueLimit_Nm==limits(index) & ...
        metrics.SignedForce_N~=0;
    recovered(index)=nnz(metrics.Outcome(selected)=="recovered");
    upright(index)=nnz(metrics.Outcome(selected)=="upright_not_recovered");
    fell(index)=nnz(metrics.Outcome(selected)=="fell");
    levels=unique(abs(metrics.SignedForce_N(selected)));
    bidirectional=false(size(levels));
    for levelIndex=1:numel(levels)
        atLevel=selected & abs(metrics.SignedForce_N)==levels(levelIndex);
        bidirectional(levelIndex)=nnz(atLevel)==2 && ...
            all(metrics.Recovered(atLevel));
    end
    if any(bidirectional)
        maximumBidirectional(index)=max(levels(bidirectional));
    end
    peakResidual(index)=max(metrics.PeakResidualTorque_Nm(selected));
    peakNormalized(index)=max(metrics.PeakNormalizedAction(selected));
    saturation(index)=any(metrics.ActuatorSaturation(selected));
end
summary=table(limits(:),recovered,upright,fell,maximumBidirectional, ...
    peakResidual,peakNormalized,saturation,'VariableNames', ...
    {'ResidualTorqueLimit_Nm','RecoveredCaseCount', ...
    'UprightNotRecoveredCaseCount','FallCaseCount', ...
    'MaximumTestedBidirectionalRecovery_N', ...
    'PeakResidualTorque_Nm','PeakNormalizedAction', ...
    'AnyFinalActuatorSaturation'});
end

function makeAuthorityPlot(summary,resultDirectory)
figureHandle=figure("Color","w","Position",[100 100 760 470]);
bar(summary.ResidualTorqueLimit_Nm, ...
    summary.MaximumTestedBidirectionalRecovery_N,0.65, ...
    "FaceColor",[0.16 0.44 0.67]);
grid on
xlabel("Residual torque limit (Nm)");
ylabel("Maximum tested bidirectional recovery (N)");
title("Frozen RL75: same-policy action-authority ablation");
xticks(summary.ResidualTorqueLimit_Nm);
ylim([0,max([200;summary.MaximumTestedBidirectionalRecovery_N])+10]);
exportgraphics(figureHandle,fullfile(resultDirectory, ...
    "authority_vs_recovery.png"),"Resolution",200);
savefig(figureHandle,fullfile(resultDirectory, ...
    "authority_vs_recovery.fig"));
end
