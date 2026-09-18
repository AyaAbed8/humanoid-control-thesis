function outputs = generate_chapter4_residual_figures(outputDirectory)
%GENERATE_CHAPTER4_RESIDUAL_FIGURES Rebuild current Chapter 4 figures.
% Reads frozen residual-RL evidence only; no simulations or models are run.

arguments
    outputDirectory (1,1) string = fullfile(project_root(), "Report Content","Figures", "Chapter 4")
end

codesDirectory = string(project_codes_root());
if ~isfolder(outputDirectory), mkdir(outputDirectory); end

authorityDirectory = fullfile(codesDirectory,"..","Report Content","Data", ...
    "ResidualRLAuthorityAblationConsolidated","20260814_final");
evaluationFile = fullfile(codesDirectory,"..","Report Content","Data","ResidualRLEvaluation", ...
    "formal_20260728_162840","processed_results.mat");
interpretationFigure = fullfile(codesDirectory,"..","Report Content","Data", ...
    "ResidualRLPolicyInterpretation","20260817_final", ...
    "representative_policy_response.fig");

assert(isfile(fullfile(authorityDirectory,"signed_force_outcomes.csv")), ...
    "Chapter4Figures:MissingAuthorityEvidence", ...
    "The consolidated action-authority evidence is missing.");
assert(isfile(fullfile(authorityDirectory,"authority_summary.csv")), ...
    "Chapter4Figures:MissingAuthoritySummary", ...
    "The consolidated action-authority summary is missing.");
assert(isfile(evaluationFile),"Chapter4Figures:MissingEvaluationEvidence", ...
    "The frozen RL75 formal-evaluation evidence is missing.");
assert(isfile(interpretationFigure), ...
    "Chapter4Figures:MissingInterpretationEvidence", ...
    "The saved learned-behaviour figure is missing.");

oldDefaults = setReportDefaults();
defaultsCleanup = onCleanup(@() restoreDefaults(oldDefaults));
palette = struct("A",[0.15 0.38 0.62],"RL",[0.85 0.40 0.10], ...
    "recovered",[0.20 0.55 0.38],"upright",[0.72 0.74 0.77], ...
    "fell",[0.73 0.24 0.20]);

outputs = strings(5,2);
outcomes = readtable(fullfile(authorityDirectory,"signed_force_outcomes.csv"), ...
    "TextType","string");
authoritySummary = readtable(fullfile(authorityDirectory, ...
    "authority_summary.csv"),"TextType","string");
outputs(1,:) = makeOutcomeMap(outcomes,palette,outputDirectory);

evaluation = load(evaluationFile,"definitions","metrics","trajectories");
outputs(2,:) = makeLateralResponse(evaluation,palette,outputDirectory);
outputs(3,:) = makeAuthorityComparison(authoritySummary,outcomes,palette, ...
    outputDirectory);
outputs(4,:) = makeAuthoritySaturationChronology(codesDirectory,palette, ...
    outputDirectory);
outputs(5,:) = makeInterpretationFigure(interpretationFigure,outputDirectory);

fprintf("Chapter 4 residual-RL figures written to:\n%s\n",outputDirectory);
disp(array2table(outputs,"VariableNames",["PNG","PDF"]));
end

function files=makeAuthorityComparison(summary,outcomes,palette,outputDirectory)
labels=["Controller A","RL75 clipped 10", ...
    "RL10-25","RL75 15"];
recovery=summary.MaximumTestedBidirectionalRecovery_N;
forces=[-185 -150 -100 100 150 185];
columns=["Minus185N","Minus150N","Minus100N", ...
    "Plus100N","Plus150N","Plus185N"];
codes=zeros(height(outcomes),numel(columns));
for row=1:height(outcomes)
    for column=1:numel(columns)
        value=outcomes.(columns(column))(row);
        if value=="upright_not_recovered", codes(row,column)=1;
        elseif value=="fell", codes(row,column)=2;
        end
    end
end

fig=figure("Color","white","Position",[60 80 1320 570]);
cleanup=onCleanup(@() close(fig));
layout=tiledlayout(fig,1,2,"TileSpacing","compact","Padding","compact");

ax=nexttile(layout,1);
bars=bar(ax,recovery,0.65,"FaceColor","flat","EdgeColor","none");
bars.CData=[palette.A;palette.RL;palette.RL;palette.RL];
xticks(ax,1:4); xticklabels(ax,labels);
xtickangle(ax,18);
ylabel(ax,"Maximum tested bidirectional recovery (N)");
ylim(ax,[0 205]); yticks(ax,0:25:200); grid(ax,"on"); box(ax,"off");
title(ax,"Recovery on the common tested grid");
for index=1:4
    text(ax,index,recovery(index)+6,sprintf("%g N",recovery(index)), ...
        "HorizontalAlignment","center","FontWeight","bold");
end

ax=nexttile(layout,2);
imagesc(ax,codes); axis(ax,"tight");
colormap(ax,[palette.recovered;palette.upright;palette.fell]);
clim(ax,[-0.5 2.5]);
xticks(ax,1:6); xticklabels(ax,compose("%+g",forces));
yticks(ax,1:4); yticklabels(ax,["Controller A", ...
    "RL75 clipped (10 N m)","RL10-25 (10 N m)","RL75 (15 N m)"]);
xlabel(ax,"Signed lateral force (N)");
title(ax,"Outcome of each signed disturbance");
for row=1:4
    for column=1:6
        marker="R"; color="white";
        if codes(row,column)==1, marker="U"; color="black";
        elseif codes(row,column)==2, marker="F";
        end
        text(ax,column,row,marker,"HorizontalAlignment","center", ...
            "FontWeight","bold","Color",color);
    end
end
title(layout,"Residual control authority and lateral recovery", ...
    "FontWeight","bold");
files=exportPair(fig,outputDirectory,"residual_authority_comparison");
end

function files=makeAuthoritySaturationChronology(codesDirectory,palette,outputDirectory)
file=fullfile(codesDirectory,"..","Report Content","Data","FrozenRL75AuthorityAblation", ...
    "20260812_110700","postprocessing","saturation_chronology.csv");
assert(isfile(file),"Chapter4Figures:MissingSaturationChronology", ...
    "The authority saturation chronology is missing.");
data=readtable(file,"TextType","string");
failed=data(data.Fell==1,:);
assert(height(failed)==3,"Chapter4Figures:FailedAuthorityCases", ...
    "Expected the three recorded same-policy failed cases.");
labels=compose("%g N m, %+g N",failed.ResidualTorqueLimit_Nm, ...
    failed.SignedForce_N);

fig=figure("Color","white","Position",[100 100 920 500]);
cleanup=onCleanup(@() close(fig));
ax=axes(fig); hold(ax,"on");
for index=1:height(failed)
    plot(ax,[failed.FirstSaturationTime_s(index), ...
        failed.FirstFallThresholdTime_s(index)],[index index], ...
        "Color",[0.45 0.45 0.45],"LineWidth",2.2, ...
        "HandleVisibility","off");
end
saturation=scatter(ax,failed.FirstSaturationTime_s,1:height(failed),85, ...
    palette.RL,"filled","MarkerEdgeColor","white");
fall=scatter(ax,failed.FirstFallThresholdTime_s,1:height(failed),85, ...
    palette.fell,"filled","MarkerEdgeColor","white");
yticks(ax,1:height(failed)); yticklabels(ax,labels); ax.YDir="reverse";
xlabel(ax,"Time (s)");
ylabel(ax,"Residual bound and signed push");
title(ax,"Final actuator saturation preceded the fall threshold");
grid(ax,"on"); box(ax,"off"); xlim(ax,[2.5 5.7]); ylim(ax,[0.65 3.35]);
legend(ax,[saturation fall],["First actuator saturation","Fall threshold"], ...
    "Location","southeast");
files=exportPair(fig,outputDirectory,"residual_authority_saturation_timeline");
end

function files = makeOutcomeMap(outcomes,palette,outputDirectory)
forces = [-185 -150 -100 100 150 185];
rows = ["Controller A","RL75 at +/-15 Nm"];
rowLabels = ["Controller A","RL75"];
columns = ["Minus185N","Minus150N","Minus100N", ...
    "Plus100N","Plus150N","Plus185N"];
imageData = zeros(2,6); labels = strings(2,6);
for row=1:2
    source = outcomes(outcomes.ControllerVariant==rows(row),:);
    assert(height(source)==1,"Chapter4Figures:OutcomeRow", ...
        "Expected one consolidated outcome row for %s.",rows(row));
    for column=1:6
        outcome=source.(columns(column));
        if outcome=="recovered", imageData(row,column)=1; labels(row,column)="Recovered";
        elseif outcome=="upright_not_recovered", imageData(row,column)=2; labels(row,column)="Upright";
        else, imageData(row,column)=3; labels(row,column)="Fell";
        end
    end
end

fig=figure("Color","white","Position",[80 100 1200 430]);
cleanup=onCleanup(@() close(fig));
ax=axes(fig); imagesc(ax,imageData);
colormap(ax,[palette.recovered;palette.upright;palette.fell]); clim(ax,[0.5 3.5]);
xticks(ax,1:6); xticklabels(ax,compose("%+g",forces));
yticks(ax,1:2); yticklabels(ax,rowLabels); ax.YDir="reverse";
xlabel(ax,"Signed lateral force (N)");
title(ax,"Common-grid lateral disturbance outcomes under ideal torque delivery");
for row=1:2
    for column=1:6
        color="white"; if imageData(row,column)==2, color="black"; end
        text(ax,column,row,labels(row,column),"HorizontalAlignment","center", ...
            "Color",color,"FontWeight","bold","FontSize",9);
    end
end
files=exportPair(fig,outputDirectory,"residual_recovery_outcome_map");
end

function files = makeLateralResponse(evaluation,palette,outputDirectory)
fig=figure("Color","white","Position",[60 40 1280 900]);
cleanup=onCleanup(@() close(fig));
layout=tiledlayout(4,2,"TileSpacing","compact","Padding","compact");
for column=1:2
    force=150; if column==2, force=-150; end
    trajectories=selectTrajectories(evaluation,force);
    t=trajectories.A.qTime;
    nominalA=interp1(trajectories.A0.qTime,trajectories.A0.q, ...
        t,"linear","extrap");
    nominalRL=interp1(trajectories.RL0.qTime,trajectories.RL0.q, ...
        trajectories.RL.qTime,"linear","extrap");
    displacementA=trajectories.A.q(:,1)-nominalA(:,1);
    displacementRL=trajectories.RL.q(:,1)-nominalRL(:,1);
    tiltA=rad2deg(trajectories.A.q(:,4)-nominalA(:,4));
    tiltRL=rad2deg(trajectories.RL.q(:,4)-nominalRL(:,4));
    velocityA=gradient(displacementA,t);
    velocityRL=gradient(displacementRL,trajectories.RL.qTime);

    ax=nexttile(column); lines=plotControllerPair(ax,t,1e3*displacementA, ...
        trajectories.RL.qTime,1e3*displacementRL,palette);
    ylabel(ax,"Lateral displacement (mm)"); title(ax,sprintf("%+g N lateral push",force));
    markPulse(ax);
    ax=nexttile(column+2); plotControllerPair(ax,t,tiltA, ...
        trajectories.RL.qTime,tiltRL,palette);
    ylabel(ax,"Lateral tilt (deg)"); markPulse(ax);
    ax=nexttile(column+4); plotControllerPair(ax,t,velocityA, ...
        trajectories.RL.qTime,velocityRL,palette);
    ylabel(ax,"Lateral velocity (m/s)"); markPulse(ax);
    ax=nexttile(column+6); hold(ax,"on");
    plot(ax,trajectories.RL.residualTime,trajectories.RL.residualTorque(:,1), ...
        "Color",palette.A,"LineWidth",1.4);
    plot(ax,trajectories.RL.residualTime,trajectories.RL.residualTorque(:,2), ...
        "Color",palette.RL,"LineWidth",1.4);
    yline(ax,15,":","Color",[0.4 0.4 0.4]); yline(ax,-15,":","Color",[0.4 0.4 0.4]);
    ylabel(ax,"RL75 residual torque (N m)"); xlabel(ax,"Time (s)");
    grid(ax,"on"); box(ax,"off"); xlim(ax,[0 max(t)]); markPulse(ax);
    if column==1
        legend(nexttile(1),lines,["Controller A","RL75"],"Location","northwest");
        legend(nexttile(7),[findobj(nexttile(7),"Type","line","-not","LineStyle",":")], ...
            ["Left hip","Right hip"],"Location","northwest");
    end
end
title(layout,"Controller A and RL75 at the +/-150 N comparison level", ...
    "FontWeight","bold");
files=exportPair(fig,outputDirectory,"controller_A_RL75_lateral_response");
end

function selected=selectTrajectories(evaluation,force)
definitions=evaluation.definitions;
selected.A0=evaluation.trajectories{find(definitions.Controller=="A" & definitions.SignedForce_N==0,1)};
selected.RL0=evaluation.trajectories{find(definitions.Controller=="RL75" & definitions.SignedForce_N==0,1)};
selected.A=evaluation.trajectories{find(definitions.Controller=="A" & definitions.SignedForce_N==force,1)};
selected.RL=evaluation.trajectories{find(definitions.Controller=="RL75" & definitions.SignedForce_N==force,1)};
end

function lines=plotControllerPair(ax,timeA,valueA,timeRL,valueRL,palette)
hold(ax,"on");
lines(1)=plot(ax,timeA,valueA,"Color",palette.A,"LineWidth",1.5);
lines(2)=plot(ax,timeRL,valueRL,"Color",palette.RL,"LineWidth",1.5);
grid(ax,"on"); box(ax,"off"); xlim(ax,[0 max([timeA(:);timeRL(:)])]);
end

function markPulse(ax)
xline(ax,2,":","Color",[0.4 0.4 0.4],"HandleVisibility","off");
xline(ax,2.1,":","Color",[0.4 0.4 0.4],"HandleVisibility","off");
end

function files = makeInterpretationFigure(sourceFigure,outputDirectory)
fig=openfig(sourceFigure,"invisible");
cleanup=onCleanup(@() close(fig));
set(fig,"Color","white","Position",[40 30 1450 1000]);
axesHandles=findall(fig,"Type","axes");
for ax=reshape(axesHandles,1,[])
    label=string(ax.YLabel.String);
    if contains(label,"Push-induced action") || contains(label,"Action modes")
        lines=findall(ax,"Type","line");
        for line=reshape(lines,1,[])
            if ~isempty(line.YData), line.YData=15*line.YData; end
        end
        if contains(label,"Push-induced action")
            ylabel(ax,"Push-induced residual torque (N m)");
        else
            ylabel(ax,"Residual-torque modes (N m)");
        end
        ylim(ax,"auto");
    end
end
files=exportPair(fig,outputDirectory,"RL75_action_interpretation");
end

function files=exportPair(fig,outputDirectory,baseName)
for ax=reshape(findall(fig,"Type","axes"),1,[])
    try
        axtoolbar(ax,"Visible","off");
    catch
    end
end
pngFile=fullfile(outputDirectory,baseName+".png");
pdfFile=fullfile(outputDirectory,baseName+".pdf");
exportgraphics(fig,pngFile,"Resolution",350);
exportgraphics(fig,pdfFile,"ContentType","vector");
files=[pngFile pdfFile];
end

function old=setReportDefaults()
properties=["defaultAxesFontName","defaultTextFontName", ...
    "defaultAxesFontSize","defaultAxesLineWidth"];
oldValues=cell(size(properties));
for k=1:numel(properties), oldValues{k}=get(groot,properties(k)); end
set(groot,"defaultAxesFontName","Arial","defaultTextFontName","Arial", ...
    "defaultAxesFontSize",10,"defaultAxesLineWidth",0.8);
old={properties,oldValues};
end

function restoreDefaults(old)
for k=1:numel(old{1}), set(groot,old{1}(k),old{2}{k}); end
end
