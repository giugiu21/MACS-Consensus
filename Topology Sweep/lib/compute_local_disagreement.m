function local_disagreement = compute_local_disagreement(state, graph, n, agent_id)
% compute local graph disagreement for one agent from stacked states

local_disagreement = zeros(n, 1);
idx_i = (agent_id - 1) * n + 1 : agent_id * n;

for neighbor_id = 1:graph.N
    if graph.Adj(agent_id, neighbor_id) ~= 0
        idx_j = (neighbor_id - 1) * n + 1 : neighbor_id * n;
        local_disagreement = local_disagreement + ...
            graph.Adj(agent_id, neighbor_id) * ...
            (state(idx_i) - state(idx_j));
    end
end
end
    