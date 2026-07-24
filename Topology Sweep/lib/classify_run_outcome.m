function [is_consensus, is_blowup, normalized_final_disagreement] = ...
    classify_run_outcome(output_disagreement, consensus_tol)
%CLASSIFY_RUN_OUTCOME Classify one run from its disagreement history.

is_blowup = ~all(isfinite(output_disagreement));
normalized_final_disagreement = ...
    output_disagreement(end) / output_disagreement(1);
is_consensus = ~is_blowup && ...
    normalized_final_disagreement <= consensus_tol;
end
