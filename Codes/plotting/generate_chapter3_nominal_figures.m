function outputs = generate_chapter3_nominal_figures(outputDirectory)
%GENERATE_CHAPTER3_NOMINAL_FIGURES Rebuild Chapter 3 report figures.
%   This function reads the frozen nominal-study evidence only. It does not
%   simulate or modify either Simulink model.

arguments
    outputDirectory (1,1) string = fullfile(project_root(), "Report Content","Figures", "Chapter 3")
end

thisDirectory = string(project_codes_root());
outputDirectory = string(outputDirectory);
if ~isfolder(outputDirectory)
    mkdir(outputDirectory);
end

fourCaseFile = fullfile(thisDirectory, "..","Report Content","Data", ...
    "GravityNominalFourCaseAblation", "20260726_212925", ...
    "processed_results.mat");
sweepFile = fullfile(thisDirectory, "..","Report Content","Data", ...
    "ControllerCSequence", "20260726_191350", ...
    "controller_c_processed.mat");
gainFile = fullfile(thisDirectory, "..","Report Content","Data", ...
    "MatchedFeedbackGainAuthority", "20260726_214137", ...
    "processed_results.mat");

assert(isfile(fourCaseFile), "Chapter3Figures:MissingEvidence", ...
    "Four-case evidence not found: %s", fourCaseFile);
assert(isfile(sweepFile), "Chapter3Figures:MissingEvidence", ...
    "Nominal-sweep evidence not found: %s", sweepFile);
assert(isfile(gainFile), "Chapter3Figures:MissingEvidence", ...
    "Gain-authority evidence not found: %s", gainFile);

fourCase = load(fourCaseFile, "metrics", "definitions", "trajectories");
sweep = load(sweepFile, "metrics");
gain = load(gainFile, "metrics", "limits");

palette = struct( ...
    "A", [0.15 0.38 0.62], ...
    "B", [0.73 0.24 0.20], ...
    "C", [0.90 0.55 0.18], ...
    "D", [0.20 0.55 0.38], ...
    "grey", [0.35 0.38 0.41]);

oldDefaults = setReportDefaults();
defaultCleanup = onCleanup(@() restoreDefaults(oldDefaults));

outputs = strings(4, 2);
outputs(1, :) = makeFourCaseFigure(fourCase.metrics, palette, ...
    outputDirectory);
outputs(2, :) = makeSweepFigure(sweep.metrics, palette, ...
    outputDirectory);
outputs(3, :) = makeTrajectoryFigure(fourCase, palette, ...
    outputDirectory);
outputs(4, :) = makeGainFigure(gain.metrics, gain.limits, palette, ...
    outputDirectory);

fprintf("Chapter 3 nominal-performance figures written to:\n%s\n", ...
    outputDirectory);
disp(array2table(outputs, "VariableNames", ["PNG", "PDF"]));
end

function files = makeFourCaseFigure(metrics, palette, outputDirectory)
caseOrder = ["A", "B", "C", "D"];
metrics = sortrows(metrics, "Case");
assert(isequal(metrics.Case.', caseOrder), ...
    "Chapter3Figures:CaseOrder", "Unexpected four-case ordering.");
colors = [palette.A; palette.B; palette.C; palette.D];

figureHandle = figure("Color", "white", "Position", [80 80 1320 720]);
cleanup = onCleanup(@() close(figureHandle));
layout = tiledlayout(2, 3, "TileSpacing", "compact", "Padding", "compact");

plotCaseBars(metrics.PostureRMSError_deg, "Posture RMS", ...
    "Error (deg)", colors, "%.2f");
plotCaseBars(metrics.MaxJointDeviation_deg, "Maximum joint deviation", ...
    "Deviation (deg)", colors, "%.2f");
plotCaseBars(1e3 * metrics.TorsoYRMSError_m, ...
    "Sagittal torso-position RMS", "Error (mm)", colors, "%.2f");
plotCaseBars(metrics.PurePDTorqueRMS_Nm, ...
    "Pure torso-feedback torque", "RMS torque (N m)", colors, "%.2f");
plotCaseBars(metrics.TotalTorqueRMS_Nm, ...
    "Final applied torque", "RMS torque (N m)", colors, "%.2f");
plotCaseBars(metrics.PeakTotalTorque_Nm, ...
    "Peak final applied torque", "Peak torque (N m)", colors, "%.2f");

title(layout, "Gravity and nominal-command four-case ablation", ...
    "FontWeight", "bold");
xlabel(layout, "Controller configuration");
files = exportPair(figureHandle, outputDirectory, ...
    "gravity_nominal_four_case_ablation");
end

function plotCaseBars(values, plotTitle, yLabel, colors, format)
nexttile;
b = bar(1:4, values, 0.68, "FaceColor", "flat", ...
    "EdgeColor", [0.18 0.18 0.18], "LineWidth", 0.6);
b.CData = colors;
xticks(1:4); xticklabels(["A", "B", "C", "D"]);
ylabel(yLabel); title(plotTitle); grid on; box off;
ylim([0, max(values) * 1.23 + eps]);
for index = 1:4
    text(index, values(index) + 0.035 * max(values), ...
        sprintf(format, values(index)), "HorizontalAlignment", "center", ...
        "FontSize", 8.5, "FontWeight", "bold");
end
end

function files = makeSweepFigure(metrics, palette, outputDirectory)
sweepRows = metrics.Stage == "nominal_sweep";
data = sortrows(metrics(sweepRows, :), "NominalFraction");
selected = 1.15;

figureHandle = figure("Color", "white", "Position", [100 100 1320 430]);
cleanup = onCleanup(@() close(figureHandle));
layout = tiledlayout(1, 3, "TileSpacing", "compact", "Padding", "compact");

plotSweep(data.NominalFraction, data.PostureRMSError_deg, selected, ...
    "Posture RMS", "Error (deg)", palette.D);
plotSweep(data.NominalFraction, 1e3 * data.TorsoYRMSError_m, selected, ...
    "Sagittal torso-position RMS", "Error (mm)", palette.D);
plotSweep(data.NominalFraction, data.TotalTorqueRMS_Nm, selected, ...
    "Final applied torque", "RMS torque (N m)", palette.D);

title(layout, "Selection of the nominal-command correction fraction", ...
    "FontWeight", "bold");
xlabel(layout, "Nominal-command correction fraction, f");
files = exportPair(figureHandle, outputDirectory, ...
    "nominal_correction_fraction_sweep");
end

function plotSweep(x, y, selected, plotTitle, yLabel, color)
nexttile;
plot(x, y, "-o", "Color", color, "MarkerFaceColor", "white", ...
    "LineWidth", 1.8, "MarkerSize", 5.5); hold on;
selectedIndex = find(abs(x - selected) < 1e-12, 1);
plot(x(selectedIndex), y(selectedIndex), "o", "Color", color, ...
    "MarkerFaceColor", color, "MarkerSize", 8, "LineWidth", 1.3);
xline(selected, ":", "Selected f = 1.15", ...
    "Color", [0.25 0.25 0.25], "LabelVerticalAlignment", "bottom", ...
    "LabelHorizontalAlignment", "left");
grid on; box off; title(plotTitle); ylabel(yLabel);
xticks(x); xlim([min(x) - 0.015, max(x) + 0.015]);
end

function files = makeTrajectoryFigure(fourCase, palette, outputDirectory)
caseNames = string(fourCase.definitions.Case);
indexA = find(caseNames == "A", 1);
indexD = find(caseNames == "D", 1);
assert(~isempty(indexA) && ~isempty(indexD), ...
    "Chapter3Figures:TrajectoryCases", "A and D trajectories are required.");
trajectoryA = fourCase.trajectories{indexA};
trajectoryD = fourCase.trajectories{indexD};

figureHandle = figure("Color", "white", "Position", [100 100 1280 720]);
cleanup = onCleanup(@() close(figureHandle));
layout = tiledlayout(3, 1, "TileSpacing", "compact", "Padding", "compact");

axisOne = nexttile;
legendLines = plotPair(trajectoryA.time, postureTrace(trajectoryA), ...
    trajectoryD.time, postureTrace(trajectoryD), palette);
ylabel("Joint RMS (deg)"); title("Instantaneous joint-posture deviation");

nexttile;
plotPair(trajectoryA.time, 1e3 * (trajectoryA.torsoY - trajectoryA.torsoY(1)), ...
    trajectoryD.time, 1e3 * (trajectoryD.torsoY - trajectoryD.torsoY(1)), palette);
ylabel("Deviation (mm)"); title("Sagittal torso-position deviation");

nexttile;
plotPair(trajectoryA.time, rowRMS(trajectoryA.totalTorque), ...
    trajectoryD.time, rowRMS(trajectoryD.totalTorque), palette);
ylabel("RMS torque (N m)"); title("Applied torque across leg channels");
xlabel("Time (s)");

legend(axisOne, legendLines, ["Controller A", "Controller D"], ...
    "Location", "northoutside", "Orientation", "horizontal");
title(layout, "Nominal trajectories of Controllers A and D", ...
    "FontWeight", "bold");
files = exportPair(figureHandle, outputDirectory, ...
    "controller_A_D_nominal_trajectories");
end

function trace = postureTrace(trajectory)
deviation = rad2deg(trajectory.jointAngles - trajectory.jointAngles(1, :));
trace = rowRMS(deviation);
end

function values = rowRMS(matrix)
values = sqrt(mean(matrix.^2, 2));
end

function lines = plotPair(timeA, valuesA, timeD, valuesD, palette)
lineA = plot(timeA, valuesA, "Color", palette.A, "LineWidth", 1.5, ...
    "DisplayName", "Controller A", "Tag", "legendLine"); hold on;
lineD = plot(timeD, valuesD, "Color", palette.D, "LineWidth", 1.5, ...
    "DisplayName", "Controller D", "Tag", "legendLine");
grid on; box off; xlim([0 max([timeA(:); timeD(:)])]);
lines = [lineA, lineD];
end

function files = makeGainFigure(metrics, limits, palette, outputDirectory)
dataA = sortrows(metrics(metrics.Controller == "A", :), "GainScale");
dataD = sortrows(metrics(metrics.Controller == "D", :), "GainScale");

figureHandle = figure("Color", "white", "Position", [90 90 1200 720]);
cleanup = onCleanup(@() close(figureHandle));
layout = tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact");

axisOne = nexttile;
legendLines = plotGainMetric(dataA, dataD, "PostureRMSError_deg", ...
    limits.PostureRMSError_deg, "Posture RMS", "Error (deg)", palette);
nexttile;
plotGainMetric(dataA, dataD, "MaxJointDeviation_deg", ...
    limits.MaxJointDeviation_deg, "Maximum joint deviation", ...
    "Deviation (deg)", palette);
nexttile;
plotGainMetric(dataA, dataD, "TorsoYRMSError_m", ...
    limits.TorsoYRMSError_m, "Sagittal torso-position RMS", ...
    "Error (m)", palette);
nexttile;
plotGainMetric(dataA, dataD, "TorsoYMaxDeviation_m", ...
    limits.TorsoYMaxDeviation_m, "Maximum sagittal torso deviation", ...
    "Deviation (m)", palette);

title(layout, "Matched outer-feedback gain scaling", "FontWeight", "bold");
xlabel(layout, "Uniform proportional and derivative gain scale, kappa");
legend(axisOne, legendLines, ...
    ["Controller A", "Controller D", "Acceptance limit"], ...
    "Location", "southoutside", "Orientation", "horizontal");
files = exportPair(figureHandle, outputDirectory, ...
    "matched_feedback_gain_scaling");
end

function lines = plotGainMetric(dataA, dataD, variable, limit, plotTitle, yLabel, palette)
lineA = plot(dataA.GainScale, dataA.(variable), "-o", "Color", palette.A, ...
    "MarkerFaceColor", palette.A, "LineWidth", 1.6, "MarkerSize", 5, ...
    "DisplayName", "Controller A"); hold on;
lineD = plot(dataD.GainScale, dataD.(variable), "-s", "Color", palette.D, ...
    "MarkerFaceColor", palette.D, "LineWidth", 1.6, "MarkerSize", 5, ...
    "DisplayName", "Controller D");
limitLine = yline(limit, ":", "Color", palette.grey, "LineWidth", 1.5, ...
    "DisplayName", "Acceptance limit");
grid on; box off; title(plotTitle); ylabel(yLabel);
xticks(sort(unique(dataA.GainScale))); xlim([0.48 1.02]);
lines = [lineA, lineD, limitLine];
end

function files = exportPair(figureHandle, outputDirectory, baseName)
pngFile = fullfile(outputDirectory, baseName + ".png");
pdfFile = fullfile(outputDirectory, baseName + ".pdf");
exportgraphics(figureHandle, pngFile, "Resolution", 350, ...
    "BackgroundColor", "white");
exportgraphics(figureHandle, pdfFile, "ContentType", "vector", ...
    "BackgroundColor", "white");
files = [pngFile, pdfFile];
end

function old = setReportDefaults()
properties = [ ...
    "defaultAxesFontName", "defaultAxesFontSize", ...
    "defaultTextFontName", "defaultTextFontSize", ...
    "defaultAxesLineWidth", "defaultLineLineWidth", ...
    "defaultAxesToolbarVisible"];
old = struct();
for index = 1:numel(properties)
    old.(properties(index)) = get(groot, properties(index));
end
set(groot, "defaultAxesFontName", "Arial", ...
    "defaultAxesFontSize", 10, ...
    "defaultTextFontName", "Arial", ...
    "defaultTextFontSize", 10, ...
    "defaultAxesLineWidth", 0.8, ...
    "defaultLineLineWidth", 1.4, ...
    "defaultAxesToolbarVisible", "off");
end

function restoreDefaults(old)
names = fieldnames(old);
for index = 1:numel(names)
    set(groot, names{index}, old.(names{index}));
end
end
