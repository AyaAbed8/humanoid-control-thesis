function files = generate_chapter5_actuator_figures(outputDirectory)
%GENERATE_CHAPTER5_ACTUATOR_FIGURES Rebuild thesis figures from frozen data.
% No simulation is run. All plots use the final saved actuator-study evidence.

arguments
    outputDirectory (1,1) string = fullfile(project_root(),"Report Content","Figures","Chapter 5")
end

codeDirectory = string(project_codes_root());
nominalDirectory = fullfile(codeDirectory,"..","Report Content","Data", ...
    "AllMotorDriveNominalControllerGate","20260822_132046");
actionDirectory = fullfile(codeDirectory,"..","Report Content","Data", ...
    "RL75AllMotorDriveNominalActionAudit","20260822_135035");
disturbanceDirectory = fullfile(codeDirectory,"..","Report Content","Data", ...
    "AllMotorDriveDisturbanceBenchmark","20260822_141700");
assert(isfolder(nominalDirectory) && isfolder(actionDirectory) && ...
    isfolder(disturbanceDirectory),"Chapter5Figures:EvidenceMissing", ...
    "One or more frozen actuator-study result directories are missing.");
if ~isfolder(outputDirectory); mkdir(outputDirectory); end

nominal = load(fullfile(nominalDirectory,"raw_trajectories.mat"), ...
    "definitions","trajectories");
action = load(fullfile(actionDirectory,"processed_results.mat"), ...
    "qTime","q","residualTime","residualTorque","torqueLimit_Nm");
disturbance = load(fullfile(disturbanceDirectory,"raw_trajectories.mat"), ...
    "definitions","trajectories","protocol");
disturbanceMetrics = readtable(fullfile(disturbanceDirectory,"metrics.csv"), ...
    "TextType","string");

files = strings(0,1);
files = [files; nominalBehaviourFigure(nominal,action,outputDirectory)];
files = [files; disturbanceOutcomeFigure(disturbanceMetrics,outputDirectory)];
files = [files; matchedResponseFigure(disturbance,outputDirectory)];

fprintf("\nChapter 5 figures generated from frozen evidence:\n");
fprintf("  %s\n",files);
end

function files = nominalBehaviourFigure(nominal,action,outputDirectory)
controllers = ["A","D","RL75"];
colors = [0.0000 0.4470 0.7410; 0.8500 0.3250 0.0980; ...
    0.4660 0.6740 0.1880];
fig = figure("Color","w","Units","centimeters", ...
    "Position",[2 2 18.2 17.2],"Name","Chapter 5 nominal behaviour");
layout = tiledlayout(fig,2,2,"TileSpacing","compact","Padding","compact");

ax = nexttile(layout,1); hold(ax,"on");
for index = 1:3
    x = trajectory(nominal,controllers(index));
    plot(ax,x.time,1000*(x.q(:,3)-x.q(1,3)),"LineWidth",1.35, ...
        "Color",colors(index,:));
end
formatAxes(ax,"Torso vertical displacement (mm)","(a) Vertical torso motion");
legend(ax,"Controller A","Controller D","RL75", ...
    "Location","northoutside","Orientation","horizontal");

ax = nexttile(layout,2); hold(ax,"on");
for index = 1:3
    x = trajectory(nominal,controllers(index));
    rotationMagnitude = vecnorm(rad2deg(x.q(:,4:6)-x.q(1,4:6)),2,2);
    plot(ax,x.time,rotationMagnitude,"LineWidth",1.35, ...
        "Color",colors(index,:));
end
formatAxes(ax,"Torso rotation deviation (deg)", ...
    "(b) Absolute torso-orientation deviation");

ax = nexttile(layout,3); hold(ax,"on");
plot(ax,action.qTime,rad2deg(action.q(:,14)-action.q(1,14)), ...
    "LineWidth",1.25,"Color",[0.0000 0.4470 0.7410]);
plot(ax,action.qTime,rad2deg(action.q(:,20)-action.q(1,20)), ...
    "LineWidth",1.25,"Color",[0.8500 0.3250 0.0980]);
formatAxes(ax,"Knee-angle change (deg)","(c) RL75 bilateral knee motion");
legend(ax,"Right knee","Left knee","Location","best");

ax = nexttile(layout,4); hold(ax,"on");
plot(ax,action.residualTime,action.residualTorque(:,1),"LineWidth",1.1, ...
    "Color",[0.0000 0.4470 0.7410]);
plot(ax,action.residualTime,action.residualTorque(:,2),"LineWidth",1.1, ...
    "Color",[0.8500 0.3250 0.0980]);
yline(ax, action.torqueLimit_Nm,":","Color",[0.25 0.25 0.25]);
yline(ax,-action.torqueLimit_Nm,":","Color",[0.25 0.25 0.25]);
formatAxes(ax,"Residual torque (N m)","(d) Frozen RL75 residual action");
legend(ax,"Right hip","Left hip","Location","best");

xlabel(layout,"Time (s)");
files = exportPair(fig,outputDirectory,"nonideal_nominal_behaviour");
close(fig);
end

function files = disturbanceOutcomeFigure(metrics,outputDirectory)
controllers = ["A","D","RL75"];
forces = [-150 -100 100 150];
outcomes = strings(3,4);
for row = 1:3
    for column = 1:4
        selected = metrics.Controller==controllers(row) & ...
            metrics.SignedForce_N==forces(column);
        assert(nnz(selected)==1,"Chapter5Figures:OutcomeLookup", ...
            "Expected one row for %s, %+g N.",controllers(row),forces(column));
        outcomes(row,column) = metrics.Outcome(selected);
    end
end
codes = zeros(3,4);
codes(outcomes=="upright_not_recovered") = 1;
codes(outcomes=="fell") = 2;

fig = figure("Color","w","Units","centimeters", ...
    "Position",[2 2 16.8 8.4],"Name","Chapter 5 disturbance outcomes");
ax = axes(fig);
imagesc(ax,codes); axis(ax,"tight");
colormap(ax,[0.2471 0.5882 0.8039; 0.72 0.75 0.79; 0.70 0.18 0.16]);
clim(ax,[-0.5 2.5]);
xticks(ax,1:4); xticklabels(ax,compose("%+d",forces));
yticks(ax,1:3); yticklabels(ax,["Controller A","Controller D","RL75"]);
xlabel(ax,"Signed lateral force (N)");
title(ax,"Outcomes with all 14 nonideal actuator paths enabled");
ax.TickLength=[0 0]; ax.FontName="Arial"; ax.FontSize=10.5;
for row = 1:3
    for column = 1:4
        if outcomes(row,column)=="upright_not_recovered"
            label=sprintf("Upright,\nnot recovered"); textColor=[0.08 0.08 0.08];
        elseif outcomes(row,column)=="fell"
            label="Fell"; textColor=[1 1 1];
        else
            label="Recovered"; textColor=[1 1 1];
        end
        text(ax,column,row,label,"HorizontalAlignment","center", ...
            "FontWeight","bold","FontSize",9.5,"Color",textColor);
    end
end
hold(ax,"on");
legendHandles = gobjects(3,1);
legendColors = [0.2471 0.5882 0.8039; 0.72 0.75 0.79; 0.70 0.18 0.16];
for index=1:3
    legendHandles(index)=plot(ax,nan,nan,"s","MarkerSize",10, ...
        "MarkerFaceColor",legendColors(index,:), ...
        "MarkerEdgeColor",[0.2 0.2 0.2],"LineStyle","none");
end
legend(ax,legendHandles,["Recovered","Upright, not recovered","Fell"], ...
    "Location","southoutside","Orientation","horizontal");
files = exportPair(fig,outputDirectory,"nonideal_disturbance_outcome_map");
close(fig);
end

function files = matchedResponseFigure(data,outputDirectory)
force = -100;
controllers = ["A","RL75"];
colors = [0.0000 0.4470 0.7410; 0.8500 0.3250 0.0980];
response = cell(2,1);
for index = 1:2
    disturbed = trajectory(data,controllers(index),force);
    nominal = trajectory(data,controllers(index),0);
    nominalQ = interp1(nominal.time,nominal.q,disturbed.time, ...
        "linear","extrap");
    induced = disturbed.q-nominalQ;
    response{index} = struct("time",disturbed.time, ...
        "displacement",induced(:,1),"tilt",rad2deg(induced(:,5)), ...
        "velocity",gradient(induced(:,1),disturbed.time));
end

fig = figure("Color","w","Units","centimeters", ...
    "Position",[2 2 17.4 14.0],"Name","Chapter 5 matched response");
layout = tiledlayout(fig,3,1,"TileSpacing","compact","Padding","compact");
labels = ["Lateral displacement (m)", ...
    "Lateral tilt (deg)","Lateral velocity (m/s)"];
titles = ["(a) Lateral torso displacement", ...
    "(b) Lateral torso tilt","(c) Lateral torso velocity"];
fields = ["displacement","tilt","velocity"];
for panel = 1:3
    ax = nexttile(layout,panel); hold(ax,"on");
    for index = 1:2
        plot(ax,response{index}.time,response{index}.(fields(panel)), ...
            "LineWidth",1.35,"Color",colors(index,:));
    end
    xline(ax,2.0,":","Color",[0.25 0.25 0.25]);
    xline(ax,2.1,":","Color",[0.25 0.25 0.25]);
    yline(ax,0,":","Color",[0.35 0.35 0.35]);
    formatAxes(ax,labels(panel),titles(panel));
    if panel==1
        legend(ax,"Controller A","RL75","Location","best");
    end
end
xlabel(layout,"Time (s)");
files = exportPair(fig,outputDirectory,"nonideal_A_RL75_response");
close(fig);
end

function x = trajectory(data,controller,force)
if nargin<3
    selected = data.definitions.Controller==controller;
else
    selected = data.definitions.Controller==controller & ...
        data.definitions.SignedForce_N==force;
end
index = find(selected,1);
assert(nnz(selected)==1,"Chapter5Figures:TrajectoryLookup", ...
    "Could not identify a unique trajectory for %s.",controller);
x = data.trajectories{index};
end

function formatAxes(ax,yLabelText,titleText)
grid(ax,"on"); box(ax,"on");
ax.FontName="Arial"; ax.FontSize=9.5; ax.LineWidth=0.8;
ax.Toolbar.Visible="off";
xlim(ax,[0 20]);
if contains(titleText,"Force-induced") || contains(titleText,"Lateral torso")
    xlim(ax,[0 12]);
end
ylabel(ax,yLabelText);
title(ax,titleText,"FontWeight","bold");
end

function files = exportPair(fig,outputDirectory,baseName)
pngFile = fullfile(outputDirectory,baseName+".png");
pdfFile = fullfile(outputDirectory,baseName+".pdf");
exportgraphics(fig,pngFile,"Resolution",300);
exportgraphics(fig,pdfFile,"ContentType","vector");
files = [string(pngFile);string(pdfFile)];
end
