function files = generate_poster_nonideal_nominal_trajectories(outputDirectory)
%GENERATE_POSTER_NONIDEAL_NOMINAL_TRAJECTORIES Poster-ready nominal response.
% Rebuilds one clean figure from the frozen 20 s all-Motor-&-Drive evidence.
% No simulation or model modification is performed.

arguments
    outputDirectory (1,1) string = fullfile(project_root(),"Report Content","Figures","Poster","Results")
end

codeDirectory = string(project_codes_root());
evidenceDirectory = fullfile(codeDirectory,"..","Report Content","Data", ...
    "AllMotorDriveNominalControllerGate","20260822_132046");
evidenceFile = fullfile(evidenceDirectory,"raw_trajectories.mat");
assert(isfile(evidenceFile),"PosterNonidealNominal:EvidenceMissing", ...
    "Frozen trajectory evidence was not found: %s",evidenceFile);
if ~isfolder(outputDirectory); mkdir(outputDirectory); end

data = load(evidenceFile,"definitions","trajectories");
controllers = ["A","D","RL75"];
legendLabels = ["Controller A","Controller D","RL75"];
colors = [0.0000 0.4470 0.7410; ...
          0.8500 0.3250 0.0980; ...
          0.4660 0.6740 0.1880];

fig = figure("Color","w","Units","centimeters", ...
    "Position",[2 2 18.0 9.2], ...
    "Name","Poster nonideal nominal trajectories");
ax = axes(fig); hold(ax,"on");

for index = 1:numel(controllers)
    selected = data.definitions.Controller==controllers(index) & ...
        data.definitions.Actuation=="all_motor_drives";
    assert(nnz(selected)==1,"PosterNonidealNominal:TrajectoryLookup", ...
        "Expected one all-Motor-&-Drive trajectory for %s.",controllers(index));
    trajectory = data.trajectories{find(selected,1)};
    torsoHeightChange_mm = 1000*(trajectory.q(:,3)-trajectory.q(1,3));
    plot(ax,trajectory.time,torsoHeightChange_mm,"LineWidth",2.25, ...
        "Color",colors(index,:));
end

yline(ax,0,"Color",[0.25 0.25 0.25],"LineWidth",1.0);
xlim(ax,[0 20]);
xticks(ax,0:5:20);
xlabel(ax,"Time (s)","FontWeight","normal");
ylabel(ax,"Torso height change (mm)","FontWeight","normal");
legend(ax,legendLabels,"Location","northoutside", ...
    "Orientation","horizontal","Box","off");
grid(ax,"on"); box(ax,"on");
ax.FontName = "Arial";
ax.FontSize = 14;
ax.LineWidth = 1.0;
ax.GridAlpha = 0.18;
ax.Toolbar.Visible = "off";

pngFile = fullfile(outputDirectory,"poster_nonideal_nominal_trajectories.png");
pdfFile = fullfile(outputDirectory,"poster_nonideal_nominal_trajectories.pdf");
figFile = fullfile(outputDirectory,"poster_nonideal_nominal_trajectories.fig");
exportgraphics(fig,pngFile,"Resolution",300);
exportgraphics(fig,pdfFile,"ContentType","vector");
savefig(fig,figFile);
files = [string(pngFile);string(pdfFile);string(figFile)];

fprintf("\nPoster nominal nonideal-actuation figure generated from:\n  %s\n", ...
    evidenceFile);
fprintf("Outputs:\n  %s\n",files);
end
