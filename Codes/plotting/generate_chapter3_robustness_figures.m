function outputs = generate_chapter3_robustness_figures(outputDirectory)
%GENERATE_CHAPTER3_ROBUSTNESS_FIGURES Rebuild the Chapter 3 robustness figures.
% Reads the frozen 12 s benchmark evidence; it does not run simulations.

arguments
    outputDirectory (1,1) string = fullfile(project_root(), "Report Content","Figures", "Chapter 3")
end

codesDirectory = string(project_codes_root());
evidenceFile = fullfile(codesDirectory, "..","Report Content","Data", ...
    "FormalFrozenControllerRobustness", "20260804_153609", ...
    "processed_results.mat");
assert(isfile(evidenceFile), "Chapter3Robustness:MissingEvidence", ...
    "Frozen benchmark evidence not found: %s", evidenceFile);
if ~isfolder(outputDirectory), mkdir(outputDirectory); end

data = load(evidenceFile, "evidence", "metrics");
palette = struct("A", [0.15 0.38 0.62], "D", [0.20 0.55 0.38], ...
    "recovered", [0.20 0.55 0.38], "upright", [0.72 0.74 0.77], ...
    "fell", [0.73 0.24 0.20]);

oldDefaults = setReportDefaults();
defaultsCleanup = onCleanup(@() restoreDefaults(oldDefaults));
outputs = strings(3, 2);
outputs(1,:) = makeOutcomeMap(data.metrics, palette, outputDirectory);
outputs(2,:) = makeLateralResponse(data.evidence{3}, palette, outputDirectory);
outputs(3,:) = makeSagittalResponse(data.evidence{6}, palette, outputDirectory);

fprintf("Chapter 3 robustness figures written to:\n%s\n", outputDirectory);
disp(array2table(outputs, "VariableNames", ["PNG", "PDF"]));
end

function files = makeOutcomeMap(metrics, palette, outputDirectory)
fig = figure("Color", "white", "Position", [80 80 1280 560]);
cleanup = onCleanup(@() close(fig));
layout = tiledlayout(1,2,"TileSpacing","compact","Padding","compact");
drawOutcomePanel(metrics, "lateral", [-100 -75 -50 -25 25 50 75 100], palette);
drawOutcomePanel(metrics, "sagittal", [-100 -75 -50 50 75 100], palette);
title(layout, "Frozen-controller force-disturbance outcomes", "FontWeight", "bold");
annotation(fig,"textbox",[0.18 0.005 0.64 0.045],"String", ...
    "Green: recovered     Grey: upright, outside recovery envelope     Red: fell", ...
    "HorizontalAlignment","center","EdgeColor","none","FontSize",9);
files = exportPair(fig, outputDirectory, "model_based_force_outcome_map");
end

function drawOutcomePanel(metrics, axisName, forces, palette)
ax = nexttile; hold(ax,"on");
controllers = ["A","D"];
imageData = zeros(2,numel(forces)); labels = strings(2,numel(forces));
for row = 1:2
    for column = 1:numel(forces)
        match = metrics.Controller == controllers(row) & ...
            metrics.Axis == axisName & metrics.SignedForce_N == forces(column);
        assert(nnz(match)==1,"Chapter3Robustness:OutcomeLookup", ...
            "Expected one %s/%s/%+g N result.",controllers(row),axisName,forces(column));
        outcome = string(metrics.Outcome(match));
        if outcome == "recovered", imageData(row,column)=1; labels(row,column)="Recovered";
        elseif outcome == "upright_not_recovered", imageData(row,column)=2; labels(row,column)="Upright";
        else, imageData(row,column)=3; labels(row,column)="Fell";
        end
    end
end
imagesc(ax,imageData); colormap(ax,[palette.recovered;palette.upright;palette.fell]);
clim(ax,[0.5 3.5]); xticks(ax,1:numel(forces)); xticklabels(ax,compose("%+g",forces));
yticks(ax,1:2); yticklabels(ax,["Controller A","Controller D"]);
xlabel(ax,"Signed force (N)"); title(ax,upperFirst(axisName)+" pushes");
for row=1:2
    for column=1:numel(forces)
        color="white"; if imageData(row,column)==2, color="black"; end
        text(ax,column,row,labels(row,column),"HorizontalAlignment","center", ...
            "Color",color,"FontWeight","bold","FontSize",8);
    end
end
ax.YDir="reverse"; box(ax,"on");
end

function files = makeLateralResponse(batch, palette, outputDirectory)
fig = figure("Color","white","Position",[80 50 1280 820]);
cleanup = onCleanup(@() close(fig));
layout=tiledlayout(3,2,"TileSpacing","compact","Padding","compact");
for column=1:2
    if column==1, indices=[2 5]; force=75; else, indices=[3 6]; force=-75; end
    nominalIndices=[1 4];
    lines = plotMatchedPair(batch,indices,nominalIndices,"torsoXDeviation",1e3,palette,column);
    ylabel("Lateral displacement (mm)"); title(sprintf("%+g N lateral push",force));
    addLateWindowAndLimits(25);
    plotMatchedPair(batch,indices,nominalIndices,"torsoLateralTiltDeviation",180/pi,palette,column+2);
    ylabel("Lateral tilt (deg)"); addLateWindowAndLimits(2);
    plotTorquePair(batch,indices,palette,column+4); ylabel("Peak |torque| (N m)");
    yline(100,":","Actuator limit","Color",[0.35 0.35 0.35]); xlabel("Time (s)");
    if column==1
        legend(lines,["Controller A","Controller D"],"Location","northwest");
    end
end
title(layout,"Representative lateral boundary responses at +/-75 N", ...
    "FontWeight","bold");
files=exportPair(fig,outputDirectory,"model_based_lateral_boundary_response");
end

function files = makeSagittalResponse(batch, palette, outputDirectory)
fig=figure("Color","white","Position",[80 50 1280 820]);
cleanup=onCleanup(@() close(fig));
layout=tiledlayout(3,2,"TileSpacing","compact","Padding","compact");
for column=1:2
    if column==1, indices=[2 5]; force=75; else, indices=[3 6]; force=-75; end
    nominalIndices=[1 4];
    lines=plotMatchedPair(batch,indices,nominalIndices,"torsoYDeviation",1e3,palette,column);
    ylabel("Sagittal displacement (mm)"); title(sprintf("%+g N sagittal push",force));
    addLateWindowAndLimits(25);
    plotMatchedPair(batch,indices,nominalIndices,"torsoSagittalTiltDeviation",180/pi,palette,column+2);
    ylabel("Sagittal tilt (deg)"); addLateWindowAndLimits(2);
    plotMatchedPair(batch,indices,nominalIndices,"torsoYVelocity",1,palette,column+4);
    ylabel("Sagittal velocity (m/s)"); addLateWindowAndLimits(0.02); xlabel("Time (s)");
    if column==1
        legend(lines,["Controller A","Controller D"],"Location","northwest");
    end
end
title(layout,"Representative sagittal responses at +/-75 N", ...
    "FontWeight","bold");
files=exportPair(fig,outputDirectory,"model_based_sagittal_response");
end

function lines=plotMatchedPair(batch,indices,nominalIndices,fieldName,scale,palette,tile)
ax=nexttile(tile); hold(ax,"on"); colors={palette.A,palette.D}; lines=gobjects(1,2);
for k=1:2
    disturbed=batch.trajectories{indices(k)}; nominal=batch.trajectories{nominalIndices(k)};
    residual=disturbed.(fieldName)-interp1(nominal.qTime,nominal.(fieldName), ...
        disturbed.qTime,"linear","extrap");
    lines(k)=plot(ax,disturbed.qTime,scale*residual,"Color",colors{k},"LineWidth",1.5);
end
grid(ax,"on"); box(ax,"off"); xlim(ax,[0 12]); xline(ax,2,":","Color",[0.4 0.4 0.4]);
xline(ax,2.1,":","Color",[0.4 0.4 0.4]);
end

function plotTorquePair(batch,indices,palette,tile)
ax=nexttile(tile); hold(ax,"on"); colors={palette.A,palette.D};
for k=1:2
    trajectory=batch.trajectories{indices(k)};
    plot(ax,trajectory.torqueTime,max(abs(trajectory.totalTorque),[],2), ...
        "Color",colors{k},"LineWidth",1.5);
end
grid(ax,"on"); box(ax,"off"); xlim(ax,[0 12]);
end

function addLateWindowAndLimits(limit)
ax=gca; yl=ylim(ax); patch(ax,[10 12 12 10],[yl(1) yl(1) yl(2) yl(2)], ...
    [0.85 0.85 0.85],"FaceAlpha",0.22,"EdgeColor","none","HandleVisibility","off");
uistack(findobj(ax,"Type","line"),"top");
yline(ax,limit,":","Color",[0.45 0.45 0.45],"HandleVisibility","off");
yline(ax,-limit,":","Color",[0.45 0.45 0.45],"HandleVisibility","off");
end

function text=upperFirst(text)
text=char(text); text=[upper(text(1)) text(2:end)]; text=string(text);
end

function files=exportPair(fig,outputDirectory,baseName)
pngFile=fullfile(outputDirectory,baseName+".png");
pdfFile=fullfile(outputDirectory,baseName+".pdf");
exportgraphics(fig,pngFile,"Resolution",350);
exportgraphics(fig,pdfFile,"ContentType","vector");
files=[pngFile pdfFile];
end

function old=setReportDefaults()
root=groot; properties=["defaultAxesFontName","defaultTextFontName", ...
    "defaultAxesFontSize","defaultAxesLineWidth"];
old=cell(size(properties));
for k=1:numel(properties), old{k}=get(root,properties(k)); end
set(root,"defaultAxesFontName","Arial","defaultTextFontName","Arial", ...
    "defaultAxesFontSize",10,"defaultAxesLineWidth",0.8);
old={properties,old};
end

function restoreDefaults(old)
for k=1:numel(old{1}), set(groot,old{1}(k),old{2}{k}); end
end
