function x0 = make_random_initial_condition(seed, N, n)
% generate random initial conditions used by the consensus examples

rng(seed);
x0 = zeros(N * n, 1);

for agent_id = 1:N
    idx = (agent_id - 1) * n + 1 : agent_id * n;
    x0(idx) = [2 * randn(); 0.5 * randn()];
end
end
