function results = consolidate_residual_authority_ablation(outputDirectory)
%CONSOLIDATE_RESIDUAL_AUTHORITY_ABLATION Combine final saved evidence.
%   Reads existing result files only. No simulation or model modification
%   is performed. The comparison separates training and deployment bounds.

arguments
    outputDirectory (1,1) string = fullfile( ...
        project_codes_root(),"Results", ...
        "ResidualRLAuthorityAblationConsolidated","20260814_final")
end

codeDirectory=string(project_codes_root());
authorityDirectory=fullfile(codeDirectory,"Results", ...
    "FrozenRL75AuthorityAblation","20260812_110700");
formal10Directory=fullfile(codeDirectory,"Results", ...
    "ResidualRL10Evaluation","authority_formal_20260814_093714");

authorityFile=fullfile(authorityDirectory,"authority_metrics.csv");
nominalFile=fullfile(authorityDirectory,"postprocessing", ...
    "nominal_standing_metrics.csv");
formal10File=fullfile(formal10Directory,"evaluation_metrics.csv");
assert(isfile(authorityFile) && isfile(nominalFile) && ...
    isfile(formal10File),"AuthorityConsolidation:EvidenceMissing", ...
    "One or more required saved result files are missing.");

if ~isfolder(outputDirectory); mkdir(outputDirectory); end
authority=readtable(authorityFile,"TextType","string");
nominal=readtable(nominalFile,"TextType","string");
formal10=readtable(formal10File,"TextType","string");

variants=["Controller A";"RL75 clipped to +/-10 Nm"; ...
    "RL10-25 trained at +/-10 Nm";"RL75 at +/-15 Nm"];
trainingBound=[NaN;15;10;15]; deploymentBound=[0;10;10;15];
nominalResidual=[0;nominal.ResidualTorqueRMS_Nm( ...
    nominal.ResidualTorqueLimit_Nm==10); ...
    formal10.ResidualTorqueRMS_Nm( ...
    formal10.Controller=="RL25" & formal10.SignedForce_N==0); ...
    nominal.ResidualTorqueRMS_Nm( ...
    nominal.ResidualTorqueLimit_Nm==15)];

forceGrid=[100 -100 150 -150 185 -185];
outcomes=strings(numel(variants),numel(forceGrid));
saturationCount=zeros(numel(variants),1);
for forceIndex=1:numel(forceGrid)
    force=forceGrid(forceIndex);
    outcomes(1,forceIndex)=formalOutcome(formal10,"A",force);
    outcomes(2,forceIndex)=authorityOutcome(authority,10,force);
    outcomes(3,forceIndex)=formalOutcome(formal10,"RL25",force);
    outcomes(4,forceIndex)=authorityOutcome(authority,15,force);
end
saturationCount(1)=nnz(formal10.ActuatorSaturation( ...
    formal10.Controller=="A" & formal10.SignedForce_N~=0));
saturationCount(2)=nnz(authority.ActuatorSaturation( ...
    authority.ResidualTorqueLimit_Nm==10 & authority.SignedForce_N~=0));
saturationCount(3)=nnz(formal10.ActuatorSaturation( ...
    formal10.Controller=="RL25" & formal10.SignedForce_N~=0));
saturationCount(4)=nnz(authority.ActuatorSaturation( ...
    authority.ResidualTorqueLimit_Nm==15 & authority.SignedForce_N~=0));

maximumBidirectional=zeros(numel(variants),1);
for index=1:numel(variants)
    for magnitude=[100 150 185]
        columns=abs(forceGrid)==magnitude;
        if all(outcomes(index,columns)=="recovered")
            maximumBidirectional(index)=magnitude;
        end
    end
end

summary=table(variants,trainingBound,deploymentBound, ...
    nominalResidual,maximumBidirectional,saturationCount, ...
    'VariableNames',{'ControllerVariant','TrainingBound_Nm', ...
    'DeploymentBound_Nm','NominalResidualTorqueRMS_Nm', ...
    'MaximumTestedBidirectionalRecovery_N', ...
    'FinalActuatorSaturationCaseCount'});
caseOutcomes=array2table(outcomes,'VariableNames', ...
    {'Plus100N','Minus100N','Plus150N','Minus150N', ...
    'Plus185N','Minus185N'});
caseOutcomes=addvars(caseOutcomes,variants,'Before',1, ...
    'NewVariableNames','ControllerVariant');

writetable(summary,fullfile(outputDirectory,"authority_summary.csv"));
writetable(caseOutcomes,fullfile(outputDirectory, ...
    "signed_force_outcomes.csv"));
save(fullfile(outputDirectory,"consolidated_results.mat"), ...
    "summary","caseOutcomes","forceGrid","authorityDirectory", ...
    "formal10Directory");
makeFigure(summary,outcomes,forceGrid,outputDirectory);

results=struct("outputDirectory",string(outputDirectory), ...
    "summary",summary,"caseOutcomes",caseOutcomes, ...
    "sourceAuthorityDirectory",string(authorityDirectory), ...
    "sourceFormal10Directory",string(formal10Directory));
fprintf("\nConsolidated residual-authority comparison:\n");
disp(summary);
disp(caseOutcomes);
fprintf("Results written to %s\n",outputDirectory);
end

function outcome=authorityOutcome(tableData,bound,force)
selected=tableData.ResidualTorqueLimit_Nm==bound & ...
    tableData.SignedForce_N==force;
assert(nnz(selected)==1,"AuthorityConsolidation:AuthorityRowMissing", ...
    "Expected one authority row for +/-%g Nm, force %g N.",bound,force);
outcome=tableData.Outcome(selected);
end

function outcome=formalOutcome(tableData,controller,force)
selected=tableData.Controller==controller & ...
    tableData.SignedForce_N==force;
assert(nnz(selected)==1,"AuthorityConsolidation:FormalRowMissing", ...
    "Expected one formal row for %s, force %g N.",controller,force);
outcome=tableData.Outcome(selected);
end

function makeFigure(summary,outcomes,forceGrid,outputDirectory)
figureHandle=figure("Color","w","Position",[100 100 1120 500]);
layout=tiledlayout(1,2,"TileSpacing","compact","Padding","compact");
title(layout,"Residual action authority: consolidated evidence");

nexttile
bar(categorical(["A","RL75 clipped 10","RL10-25","RL75 15"], ...
    ["A","RL75 clipped 10","RL10-25","RL75 15"]), ...
    summary.MaximumTestedBidirectionalRecovery_N,0.68, ...
    "FaceColor",[0.16 0.44 0.67]);
ylabel("Maximum tested bidirectional recovery (N)");
ylim([0 205]); grid on; title("Recovery level");

nexttile
code=zeros(size(outcomes));
code(outcomes=="upright_not_recovered")=1;
code(outcomes=="fell")=2;
imagesc(code); axis tight
colormap(gca,[0.18 0.55 0.34;0.72 0.74 0.76;0.70 0.18 0.16]);
colorbar("Ticks",[0 1 2],"TickLabels", ...
    ["Recovered","Upright, not recovered","Fell"]);
xticks(1:numel(forceGrid));
xticklabels(compose("%+g N",forceGrid));
yticks(1:4); yticklabels(["A","RL75 clipped 10", ...
    "RL10-25","RL75 15"]);
xlabel("Signed lateral force"); title("Finite-horizon outcomes");
for row=1:size(code,1)
    for column=1:size(code,2)
        marker="R";
        if code(row,column)==1; marker="U";
        elseif code(row,column)==2; marker="F"; end
        text(column,row,marker,"HorizontalAlignment","center", ...
            "FontWeight","bold","Color", ...
            conditionalColor(code(row,column)));
    end
end
exportgraphics(figureHandle,fullfile(outputDirectory, ...
    "residual_authority_comparison.png"),"Resolution",220);
savefig(figureHandle,fullfile(outputDirectory, ...
    "residual_authority_comparison.fig"));
end

function color=conditionalColor(code)
if code==1; color=[0.1 0.1 0.1]; else; color=[1 1 1]; end
end
