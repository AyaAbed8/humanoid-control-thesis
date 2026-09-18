function results = calibrate_frozen_seed1_rl25_lateral_boundary( ...
    forceLevels_N, outputRoot)
%CALIBRATE_FROZEN_SEED1_RL25_LATERAL_BOUNDARY Extend seed-1 RL25.
%   Tests +/-175 N first, then +/-185 N and +/-190 N only while both
%   signed directions recover. Uses the frozen deterministic seed-1
%   episode-25 policy and the established recovery protocol.

arguments
    forceLevels_N (1,:) double ...
        {mustBeFinite,mustBePositive} = [175 185 190]
    outputRoot (1,1) string = fullfile( ...
        project_codes_root(), "Results", ...
        "FrozenSeed1RL25RecoveryBoundary")
end

results = calibrate_frozen_rl75_lateral_boundary( ...
    forceLevels_N, outputRoot, "RL25_seed1");
end
