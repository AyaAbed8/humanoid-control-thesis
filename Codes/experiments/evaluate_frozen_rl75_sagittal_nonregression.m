function results = evaluate_frozen_rl75_sagittal_nonregression( ...
    forceMagnitudes_N, outputRoot)
%EVALUATE_FROZEN_RL75_SAGITTAL_NONREGRESSION Compare A and RL75 sagittally.
%   RL75 was trained for lateral pushes. This test therefore asks whether
%   the added lateral hip residual preserves rather than improves the
%   frozen Controller A response to +/-75 N and +/-100 N sagittal pulses.

arguments
    forceMagnitudes_N (1,:) double ...
        {mustBeFinite,mustBePositive} = [75 100]
    outputRoot (1,1) string = fullfile( ...
        project_codes_root(), "Results", ...
        "ResidualRLSagittalNonRegression")
end

results = evaluate_frozen_rl75_timing_generalization( ...
    2, forceMagnitudes_N, outputRoot, 1, "sagittal");
end
