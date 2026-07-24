function u = compute_event_triggered_control(x_hat, graph, K, n, m_input)
% compute distributed event-triggered consensus control

N = graph.N;

u = zeros(N * m_input, 1);

for i = 1:N
    local_disagreement = compute_local_disagreement(x_hat, graph, n, i);

    idx_u = (i-1)*m_input + 1 : i*m_input;
    u(idx_u) = -K * local_disagreement;
end
end
