function tau = measure_convergence_time(output_disagreement, time, tol)
%MEASURE_CONVERGENCE_TIME First time the normalized disagreement settles.
%
% Returns the earliest time at which output_disagreement, normalized by its
% initial value, drops below tol AND stays below it until the end of the
% horizon. Returns NaN if the run blows up or never settles, so that a run
% that fails to converge is not silently counted as a fast one.

if ~all(isfinite(output_disagreement))     % numerical blow-up
    tau = NaN;
    return;
end

d0 = output_disagreement(1);
if d0 == 0                                  % already at consensus
    tau = time(1);
    return;
end

normalized = output_disagreement / d0;
below = normalized <= tol;

% last time it was still ABOVE the tolerance; consensus is reached on the
% following sample and must hold to the end
last_above = find(~below, 1, 'last');
if isempty(last_above)                      % below from the very first step
    tau = time(1);
elseif last_above == numel(normalized)      % still above at the final step
    tau = NaN;
else
    tau = time(last_above + 1);
end
end
