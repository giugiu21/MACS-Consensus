function u = compute_continuous_control(x, graph, K, n, m_input)
% compute distributed continuous consensus control

N = graph.N;

u = zeros(N * m_input, 1);

for i = 1:N
    local_disagreement = compute_local_disagreement(x, graph, n, i);

    idx_u = (i-1)*m_input + 1 : i*m_input;
    u(idx_u) = -K * local_disagreement;
end
end
