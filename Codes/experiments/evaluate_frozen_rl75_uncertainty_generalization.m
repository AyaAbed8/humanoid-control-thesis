function results = evaluate_frozen_rl75_uncertainty_generalization( ...
    trunkScale, forceMagnitude_N, outputRoot)
%EVALUATE_FROZEN_RL75_UNCERTAINTY_GENERALIZATION Compare A and RL75.
%   Applies a Simscape-only Trunk density scale while keeping the
%   controller rigidBodyTree nominal. By default, this is the established
%   +20 percent Trunk mass/inertia mismatch with +/-150 N lateral pushes.
%   The 100 ms pulse begins at 2 s and the simulation lasts 11 s.

arguments
    trunkScale (1,1) double {mustBeFinite,mustBePositive} = 1.2
    forceMagnitude_N (1,1) double ...
        {mustBeFinite,mustBePositive} = 150
    outputRoot (1,1) string = fullfile( ...
        project_codes_root(), "Results", ...
        "ResidualRLModelUncertaintyGeneralization")
end

results = evaluate_frozen_rl75_timing_generalization( ...
    2, forceMagnitude_N, outputRoot, trunkScale);
end
