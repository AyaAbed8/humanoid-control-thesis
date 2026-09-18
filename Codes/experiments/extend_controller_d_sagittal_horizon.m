function results = extend_controller_d_sagittal_horizon(outputRoot)
%EXTEND_CONTROLLER_D_SAGITTAL_HORIZON Resolve delayed D sagittal behaviour.
%   Runs only Controller D's matched nominal reference and sagittal
%   +/-100 N cases to 16 s. The same trajectories are classified at
%   12, 14 and 16 s using the already declared recovery protocol.
%   Neither Simulink model is edited or saved.

arguments
    outputRoot (1,1) string = fullfile( ...
        project_codes_root(), "Results", ...
        "ControllerDSagittalHorizonExtension")
end

results = analyze_benchmark_protocol_sensitivity( ...
    outputRoot, "d_sagittal_extension");
end
