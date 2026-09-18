function trainingCase = sample_residual_rl_training_case(protocol)
%SAMPLE_RESIDUAL_RL_TRAINING_CASE Sample one reproducible reset condition.

arguments
    protocol struct = residual_rl_training_protocol("training")
end

draw = rand();
if draw < protocol.nominalProbability
    category = "nominal";
    magnitude_N = 0;
elseif draw < protocol.nominalProbability + protocol.easyProbability
    category = "easy";
    magnitude_N = sampleUniform(protocol.easyMagnitudeRange_N);
else
    category = "boundary";
    magnitude_N = sampleUniform(protocol.boundaryMagnitudeRange_N);
end

if magnitude_N == 0
    signValue = 0;
else
    signValue = 2 * (rand() >= 0.5) - 1;
end
onset_s = sampleUniform(protocol.pushOnsetRange_s);

trainingCase.category = category;
trainingCase.signedMagnitude_N = signValue * magnitude_N;
trainingCase.forceVector_N = [trainingCase.signedMagnitude_N; 0; 0];
trainingCase.onset_s = onset_s;
trainingCase.duration_s = protocol.pushDuration_s;
trainingCase.stopTime_s = protocol.episodeDuration_s;
end

function value = sampleUniform(bounds)
value = bounds(1) + diff(bounds) * rand();
end
