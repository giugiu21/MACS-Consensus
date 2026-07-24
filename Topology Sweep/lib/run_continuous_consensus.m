function result = run_continuous_consensus(x0, agent, graph, K, time, dt)
% run continuous-communication consensus and collect standard histories

N = graph.N;
num_steps = numel(time);
C_global = kron(eye(N), agent.C);

x = x0;
x_history = zeros(N * agent.n, num_steps);
y_history = zeros(N, num_steps);
output_disagreement = zeros(1, num_steps);

for step_id = 1:num_steps
    x_history(:, step_id) = x;

    y = C_global * x;
    y_history(:, step_id) = y;
    output_disagreement(step_id) = norm(y - mean(y) * ones(N, 1));

    if step_id < num_steps
        dx = continuous_linear_rhs(x, agent, graph, K);
        x = x + dt * dx;
    end
end

result = struct();
result.time = time;
result.x_history = x_history;
result.y_history = y_history;
result.output_history = y_history;
result.output_disagreement = output_disagreement;
result.final_state = x;
result.final_output_disagreement = output_disagreement(end);
end
