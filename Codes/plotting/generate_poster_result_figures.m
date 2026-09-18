function outputs = generate_poster_result_figures(outputDirectory)
%GENERATE_POSTER_RESULT_FIGURES Create three clean poster result figures.
% Reads frozen evidence only. No simulation or controller is executed.

arguments
    outputDirectory (1,1) string = fullfile(project_root(), "Report Content","Figures", "Poster", "Results")
end

codesDirectory = string(project_codes_root());
if ~isfolder(outputDirectory), mkdir(outputDirectory); end

modelMetricsFile = fullfile(codesDirectory, "..","Report Content","Data", ...
    "FormalFrozenControllerRobustness", "20260804_153609", ...
    "robustness_metrics.csv");
rlEvaluationFile = fullfile(codesDirectory, "..","Report Content","Data", ...
    "ResidualRLEvaluation", "formal_20260728_162840", ...
    "processed_results.mat");
authorityFile = fullfile(codesDirectory, "..","Report Content","Data", ...
    "FrozenRL75AuthorityAblation", "20260812_110700", ...
    "authority_summary.csv");
nonidealMetricsFile = fullfile(codesDirectory, "..","Report Content","Data", ...
    "AllMotorDriveDisturbanceBenchmark", "20260822_141700", ...
    "metrics.csv");

requiredFiles = [modelMetricsFile, rlEvaluationFile, ...
    authorityFile, nonidealMetricsFile];
assert(all(isfile(requiredFiles)), "PosterFigures:MissingEvidence", ...
    "One or more frozen evidence files are missing.");

modelMetrics = readtable(modelMetricsFile, "TextType", "string");
rlEvaluation = load(rlEvaluationFile, ...
    "definitions", "metrics", "trajectories");
authority = readtable(authorityFile, "TextType", "string");
nonidealMetrics = readtable(nonidealMetricsFile, "TextType", "string");

palette = struct( ...
    "A", [0.12 0.36 0.62], ...
    "D", [0.18 0.52 0.34], ...
    "RL", [0.91 0.39 0.08], ...
    "recovered", [0.16 0.55 0.34], ...
    "upright", [0.72 0.75 0.79], ...
    "fell", [0.72 0.16 0.14]);

oldDefaults = setPosterDefaults();
defaultsCleanup = onCleanup(@() restoreDefaults(oldDefaults));

outputs = strings(3,3);
outputs(1,:) = makeModelBasedFigure(modelMetrics, palette, outputDirectory);
outputs(2,:) = makeResidualRLFigure(rlEvaluation, authority, palette, outputDirectory);
outputs(3,:) = makeNonidealFigure(nonidealMetrics, palette, outputDirectory);

fprintf("Poster result figures written to:\n%s\n", outputDirectory);
disp(array2table(outputs, "VariableNames", ["PNG", "PDF", "FIG"]));
end

function files = makeModelBasedFigure(metrics, palette, outputDirectory)
groups = ["A","lateral"; "D","lateral"; "A","sagittal"; "D","sagittal"];
percentages = zeros(4,3);
counts = zeros(4,3);
totals = zeros(4,1);
for row = 1:4
    selected = metrics.Controller == groups(row,1) & ...
        metrics.Axis == groups(row,2);
    outcomes = metrics.Outcome(selected);
    totals(row) = numel(outcomes);
    counts(row,:) = [nnz(outcomes=="recovered"), ...
        nnz(outcomes=="upright_not_recovered"), nnz(outcomes=="fell")];
    percentages(row,:) = 100*counts(row,:)/totals(row);
end

fig = figure("Color","white","Units","pixels", ...
    "Position",[80 80 1500 900],"Name","Poster result 1");
cleanup = onCleanup(@() close(fig));
ax = axes(fig); hold(ax,"on");
bars = bar(ax, percentages, 0.68, "stacked", "EdgeColor", "none");
bars(1).FaceColor = palette.recovered;
bars(2).FaceColor = palette.upright;
bars(3).FaceColor = palette.fell;

labels = {"A / lateral", "D / lateral", "A / sagittal", "D / sagittal"};
xticks(ax,1:4); xticklabels(ax,labels);
ylabel(ax,"Signed disturbance cases (%)");
ylim(ax,[0 108]); yticks(ax,0:25:100);
grid(ax,"on"); ax.XGrid="off"; box(ax,"off");
title(ax,"Model-based disturbance outcomes", ...
    "FontSize",24,"FontWeight","bold");
subtitle(ax,"Lateral: +/-25, 50, 75, 100 N     Sagittal: +/-50, 75, 100 N", ...
    "FontSize",16);

for row = 1:4
    bottom = 0;
    for category = 1:3
        height = percentages(row,category);
        if height > 0
            text(ax,row,bottom+height/2, ...
                sprintf("%d/%d",counts(row,category),totals(row)), ...
                "HorizontalAlignment","center","FontSize",18, ...
                "FontWeight","bold", ...
                "Color",chooseTextColor(category));
        end
        bottom = bottom+height;
    end
end
legend(ax,bars,["Recovered","Upright, outside envelope","Fell"], ...
    "Location","southoutside","Orientation","horizontal", ...
    "FontSize",16,"Box","off");
files = exportPosterFigure(fig,outputDirectory,"poster_result_1_model_based");
end

function files = makeResidualRLFigure(evaluation, authority, palette, outputDirectory)
positive = matchedEvaluationResponse(evaluation,150);
negative = matchedEvaluationResponse(evaluation,-150);

[~, order] = sort(authority.ResidualTorqueLimit_Nm);
recovery = authority.MaximumTestedBidirectionalRecovery_N(order);

fig = figure("Color","white","Units","pixels", ...
    "Position",[80 80 1650 900],"Name","Poster result 2");
cleanup = onCleanup(@() close(fig));
layout = tiledlayout(fig,2,10,"TileSpacing","compact","Padding","compact");

% The recovery-boundary comparison is the primary quantitative result and
% occupies the larger right-hand region of the poster figure.
ax = nexttile(layout,4,[2 7]); hold(ax,"on");
ax.FontSize = 21;
recoveryValues = [100; recovery(:)];
bars = bar(ax,recoveryValues,0.64,"FaceColor","flat","EdgeColor","none");
bars.CData = [palette.A; 1.00 0.68 0.32; 0.96 0.49 0.13; palette.RL];
xticks(ax,1:4);
xticklabels(ax,{"A","RL75 (5)","RL75 (10)","RL75 (15)"});
ylabel(ax,"Bidirectional recovery (N)");
xlabel(ax,"Controller / residual limit (N m)");
ylim(ax,[0 205]); yticks(ax,0:25:200);
title(ax,"Tested recovery boundary","FontSize",24,"FontWeight","normal");
grid(ax,"on"); ax.XGrid="off"; box(ax,"off");
for index = 1:4
    text(ax,index,recoveryValues(index)+6,sprintf("%g N",recoveryValues(index)), ...
        "HorizontalAlignment","center","FontSize",20,"FontWeight","normal");
end

% Two compact responses expose the directional asymmetry and support the
% bidirectional boundary reported by the larger panel.
ax = nexttile(layout,1,[1 3]);
plotPosterResponse(ax,positive,palette,"+150 N response",[1.8 4.2],[-5 105],true);
xlabel(ax,"");

ax = nexttile(layout,11,[1 3]);
plotPosterResponse(ax,negative,palette,"-150 N response",[1.8 6],[-16 5],false);
files = exportPosterFigure(fig,outputDirectory,"poster_result_2_residual_rl");
end

function response = matchedEvaluationResponse(evaluation,force)
trajectories = selectEvaluationTrajectories(evaluation,force);
tA = trajectories.A.qTime;
tRL = trajectories.RL.qTime;
nominalA = interp1(trajectories.A0.qTime,trajectories.A0.q,tA, ...
    "linear","extrap");
nominalRL = interp1(trajectories.RL0.qTime,trajectories.RL0.q,tRL, ...
    "linear","extrap");
response = struct( ...
    "tA",tA,"tRL",tRL, ...
    "displacementA_cm",100*(trajectories.A.q(:,1)-nominalA(:,1)), ...
    "displacementRL_cm",100*(trajectories.RL.q(:,1)-nominalRL(:,1)));
end

function plotPosterResponse(ax,response,palette,titleText,xLimits,yLimits,showThreshold)
hold(ax,"on"); ax.FontSize=19;
plot(ax,response.tA,response.displacementA_cm, ...
    "Color",palette.A,"LineWidth",2.8);
plot(ax,response.tRL,response.displacementRL_cm, ...
    "Color",palette.RL,"LineWidth",2.8);
xline(ax,2.0,":","Color",[0.35 0.35 0.35],"LineWidth",1.5, ...
    "HandleVisibility","off");
xline(ax,2.1,":","Color",[0.35 0.35 0.35],"LineWidth",1.5, ...
    "HandleVisibility","off");
if showThreshold
    yline(ax,100,"--","1 m threshold","Color",palette.fell, ...
        "LineWidth",1.6,"FontSize",16,"FontWeight","normal", ...
        "LabelHorizontalAlignment","right","HandleVisibility","off");
end
yline(ax,0,"Color",[0.35 0.35 0.35],"LineWidth",0.8, ...
    "HandleVisibility","off");
xlim(ax,xLimits); ylim(ax,yLimits);
xlabel(ax,"Time (s)"); ylabel(ax,"Displacement (cm)");
title(ax,titleText,"FontSize",22,"FontWeight","normal");
legend(ax,["Controller A","RL75"],"Location","best", ...
    "FontSize",16,"Box","off");
grid(ax,"on"); box(ax,"off");
end

function files = makeNonidealFigure(metrics, palette, outputDirectory)
controllers = ["A","D","RL75"];
forces = [-150 -100 100 150];
codes = zeros(3,4);
labels = strings(3,4);
for row = 1:3
    for column = 1:4
        selected = metrics.Controller==controllers(row) & ...
            metrics.SignedForce_N==forces(column);
        assert(nnz(selected)==1,"PosterFigures:NonidealLookup", ...
            "Expected one nonideal result for %s, %+g N.", ...
            controllers(row),forces(column));
        outcome = metrics.Outcome(selected);
        if outcome=="recovered"
            codes(row,column)=0; labels(row,column)="Recovered";
        elseif outcome=="upright_not_recovered"
            codes(row,column)=1;
            labels(row,column)="Upright,"+newline+"outside"+newline+"envelope";
        else
            codes(row,column)=2; labels(row,column)="Fell";
        end
    end
end

fig = figure("Color","white","Units","pixels", ...
    "Position",[80 80 1050 760],"Name","Poster result 3");
cleanup = onCleanup(@() close(fig));
ax = axes(fig);
imagesc(ax,codes); axis(ax,"image");
colormap(ax,[palette.recovered;palette.upright;palette.fell]);
clim(ax,[-0.5 2.5]);
xticks(ax,1:4); xticklabels(ax,compose("%+d N",forces));
yticks(ax,1:3); yticklabels(ax,["Controller A","Controller D","RL75"]);
xlabel(ax,"Signed lateral disturbance");
title(ax,"Nonideal actuation: 0/12 cases recovered", ...
    "FontSize",27,"FontWeight","normal");
ax.FontSize = 21;
ax.TickLength=[0 0];

for row = 1:3
    for column = 1:4
        color = "white";
        if codes(row,column)==1, color="black"; end
        text(ax,column,row,labels(row,column), ...
            "HorizontalAlignment","center","VerticalAlignment","middle", ...
            "FontSize",21,"FontWeight","normal","Color",color);
    end
end
hold(ax,"on");
for boundary=0.5:1:4.5
    xline(ax,boundary,"Color","white","LineWidth",2, ...
        "HandleVisibility","off");
end
for boundary=0.5:1:3.5
    yline(ax,boundary,"Color","white","LineWidth",2, ...
        "HandleVisibility","off");
end
files = exportPosterFigure(fig,outputDirectory,"poster_result_3_nonideal_actuation");
end

function selected = selectEvaluationTrajectories(evaluation,force)
definitions = evaluation.definitions;
selected.A0 = evaluation.trajectories{find( ...
    definitions.Controller=="A" & definitions.SignedForce_N==0,1)};
selected.RL0 = evaluation.trajectories{find( ...
    definitions.Controller=="RL75" & definitions.SignedForce_N==0,1)};
selected.A = evaluation.trajectories{find( ...
    definitions.Controller=="A" & definitions.SignedForce_N==force,1)};
selected.RL = evaluation.trajectories{find( ...
    definitions.Controller=="RL75" & definitions.SignedForce_N==force,1)};
end

function color = chooseTextColor(category)
if category==2, color="black"; else, color="white"; end
end

function files = exportPosterFigure(fig,outputDirectory,baseName)
pngFile = fullfile(outputDirectory,baseName+".png");
pdfFile = fullfile(outputDirectory,baseName+".pdf");
figFile = fullfile(outputDirectory,baseName+".fig");
exportgraphics(fig,pngFile,"Resolution",350);
exportgraphics(fig,pdfFile,"ContentType","vector");
savefig(fig,figFile);
files = [pngFile pdfFile figFile];
end

function old = setPosterDefaults()
properties = ["defaultAxesFontName","defaultTextFontName", ...
    "defaultAxesFontSize","defaultAxesLineWidth"];
values = cell(size(properties));
for k=1:numel(properties), values{k}=get(groot,properties(k)); end
set(groot,"defaultAxesFontName","Arial", ...
    "defaultTextFontName","Arial", ...
    "defaultAxesFontSize",18,"defaultAxesLineWidth",1.2);
old = {properties,values};
end

function restoreDefaults(old)
for k=1:numel(old{1}), set(groot,old{1}(k),old{2}{k}); end
end
