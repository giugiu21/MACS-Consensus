function K = compute_lqr_consensus_gain(agent)
% compute LQR state-feedback gain

Q = eye(agent.n);
R = eye(agent.m_input);

K = lqr(agent.A, agent.B, Q, R);

if ~isequal(size(K), [agent.input_dimension, agent.n])
    error('compute_lqr_consensus_gain:BadGainSize', ...
        'K must have size agent.input_dimension-by-agent.n.');
end
end
