function protocol = residual_rl_training_protocol(mode)
%RESIDUAL_RL_TRAINING_PROTOCOL Define the first-stage push curriculum.
%   Training focuses on the measured weakness of frozen Controller A:
%   lateral pushes near its recovery boundary. Formal evaluation remains a
%   separate 11 s benchmark and must not reuse training episodes as evidence.

arguments
    mode (1,1) string {mustBeMember(mode,["validation","training"])} = ...
        "training"
end

protocol.mode = mode;
protocol.sampleTime_s = 0.025;
protocol.pushDuration_s = 0.1;

if mode == "validation"
    protocol.episodeDuration_s = 11;
    protocol.nominalProbability = 1;
    protocol.easyProbability = 0;
    protocol.easyMagnitudeRange_N = [0 0];
    protocol.boundaryMagnitudeRange_N = [0 0];
    protocol.pushOnsetRange_s = [2 2];
else
    protocol.episodeDuration_s = 6;
    protocol.nominalProbability = 0.20;
    protocol.easyProbability = 0.20;
    protocol.easyMagnitudeRange_N = [50 80];
    protocol.boundaryMagnitudeRange_N = [80 125];
    protocol.pushOnsetRange_s = [1.5 2.5];
end

protocol.boundaryProbability = ...
    1 - protocol.nominalProbability - protocol.easyProbability;
protocol.axis = "lateral";
protocol.forceVectorOrder = "[Fx; Fy; Fz]";
protocol.forcePulseDescription = ...
    "100 ms lateral force pulse; random sign for nonzero episodes";
end
