function mode_info = check_consensus_modes(agent, graph, K)
% check stability of disagreement modes

lambda = sort(real(eig(graph.L)));
disagreement_lambda = lambda(2:end);
num_modes = length(disagreement_lambda);

mode_eigenvalues = zeros(agent.n, num_modes);
is_stable = false(num_modes, 1);

for i = 1:num_modes
    A_mode = agent.A - disagreement_lambda(i) * agent.B * K;
    mode_eigenvalues(:, i) = eig(A_mode);
    is_stable(i) = all(real(mode_eigenvalues(:, i)) < 0);
end

mode_info.lambda = disagreement_lambda;
mode_info.mode_eigenvalues = mode_eigenvalues;
mode_info.is_stable = is_stable;
end
