function results = train_residual_rl_sac( ...
    maxEpisodes, randomSeed, outputRoot, showPlots, residualTorqueLimit_Nm)
%TRAIN_RESIDUAL_RL_SAC Train the initial bounded hip-residual SAC agent.
%   This is a first-stage training pipeline, not a final hyperparameter
%   claim. Training uses frozen Controller A and lateral push randomization.

arguments
    maxEpisodes (1,1) double {mustBeInteger,mustBePositive} = 100
    randomSeed (1,1) double {mustBeInteger,mustBeNonnegative} = 0
    outputRoot (1,1) string = fullfile( ...
        project_codes_root(), "Results", "ResidualRLTraining")
    showPlots (1,1) logical = true
    residualTorqueLimit_Nm (1,1) double ...
        {mustBeFinite,mustBePositive} = 15
end

rng(randomSeed, "twister");
protocol = residual_rl_training_protocol("training");
protocol.residualTorqueLimit_Nm = residualTorqueLimit_Nm;
timestamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
resultDirectory = fullfile(outputRoot, timestamp);
checkpointDirectory = fullfile(resultDirectory, "checkpoints");
mkdir(checkpointDirectory);

[environment, agentObj, observationInfo, actionInfo] = ...
    create_residual_rl_environment(protocol);

agentOptions = agentObj.AgentOptions;
agentOptions.DiscountFactor = 0.995;
agentOptions.ExperienceBufferLength = 1e5;
agentOptions.MiniBatchSize = 64;
agentOptions.NumWarmStartSteps = 256;
agentOptions.LearningFrequency = 4;
agentObj.AgentOptions = agentOptions;

maxSteps = ceil(protocol.episodeDuration_s / protocol.sampleTime_s);
checkpointFrequency = max(1, min(25, ceil(maxEpisodes / 4)));
if showPlots
    plotSetting = "training-progress";
else
    plotSetting = "none";
end
trainingOptions = rlTrainingOptions( ...
    "MaxEpisodes", maxEpisodes, ...
    "MaxStepsPerEpisode", maxSteps, ...
    "StopTrainingCriteria", "EpisodeCount", ...
    "StopTrainingValue", maxEpisodes, ...
    "SaveAgentCriteria", "EpisodeFrequency", ...
    "SaveAgentValue", checkpointFrequency, ...
    "SaveAgentDirectory", checkpointDirectory, ...
    "Plots", plotSetting, ...
    "Verbose", true, ...
    "UseParallel", false);

save(fullfile(resultDirectory, "initial_agent.mat"), ...
    "agentObj", "protocol", "trainingOptions", ...
    "observationInfo", "actionInfo", "randomSeed");
trainingResult = train(agentObj, environment, trainingOptions);
% In this MATLAB release, train updates the handle agent in place and its
% return value contains training statistics rather than an Agent property.
trainedAgent = agentObj;
save(fullfile(resultDirectory, "trained_agent_and_results.mat"), ...
    "trainedAgent", "trainingResult", "protocol", "trainingOptions", ...
    "observationInfo", "actionInfo", "randomSeed");

results.resultDirectory = resultDirectory;
results.checkpointDirectory = checkpointDirectory;
results.protocol = protocol;
results.trainingOptions = trainingOptions;
results.trainingResult = trainingResult;
results.trainedAgent = trainedAgent;
results.residualTorqueLimit_Nm = residualTorqueLimit_Nm;
fprintf("Residual RL training results written to %s\n", resultDirectory);
end
