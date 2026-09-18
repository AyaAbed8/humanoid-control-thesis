function [agentObj, frozen] = load_frozen_residual_rl_agent()
%LOAD_FROZEN_RESIDUAL_RL_AGENT Verify and load deterministic RL75.

frozen = frozen_residual_rl_controller();
loaded = load(frozen.provenance.absoluteFiles.agent, "saved_agent");
assert(isfield(loaded, "saved_agent") && ...
    isa(loaded.saved_agent, "rl.agent.rlSACAgent"), ...
    "FrozenResidualRL:InvalidAgentFile", ...
    "Frozen agent file does not contain an rlSACAgent named saved_agent.");
agentObj = loaded.saved_agent;
agentObj.UseExplorationPolicy = false;
end
