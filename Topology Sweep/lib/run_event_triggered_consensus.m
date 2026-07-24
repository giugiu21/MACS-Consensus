function result = run_event_triggered_consensus( ...
    x0, agent, graph, K, time, dt, sigma, epsilon_trigger, ...
    trigger_type, trigger_params)
% run event-triggered consensus and collect standard histories

if nargin < 9 || isempty(trigger_type)
    trigger_type = 'relative';
end

if nargin < 10 || isempty(trigger_params)
    trigger_params = struct();
end

N = graph.N;
num_steps = numel(time);
C_global = kron(eye(N), agent.C);
trigger_type_key = lower(char(trigger_type));

x = x0;
x_hat = x0;
x_history = zeros(N * agent.n, num_steps);
y_history = zeros(N, num_steps);
output_disagreement = zeros(1, num_steps);
event_history = zeros(N, num_steps);
trigger_value_history = zeros(N, num_steps);
threshold_history = zeros(N, num_steps);

for step_id = 1:num_steps
    x_history(:, step_id) = x;

    y = C_global * x;
    y_history(:, step_id) = y;
    output_disagreement(step_id) = norm(y - mean(y) * ones(N, 1));

    step_trigger_params = trigger_params;
    if strcmp(trigger_type_key, 'exponential') || ...
            strcmp(trigger_type_key, 'garcia_exponential')
        step_trigger_params.time = time(step_id);
    end

    [x_hat, events, trigger_values, thresholds] = update_event_triggers( ...
        x, x_hat, graph, sigma, epsilon_trigger, agent.n, ...
        trigger_type, step_trigger_params);

    event_history(:, step_id) = events;
    trigger_value_history(:, step_id) = trigger_values;
    threshold_history(:, step_id) = thresholds;

    if step_id < num_steps
        dx = event_triggered_linear_rhs(x, x_hat, agent, graph, K);
        x = x + dt * dx;
    end
end

events_per_agent = sum(event_history, 2);

result = struct();
result.time = time;
result.x_history = x_history;
result.y_history = y_history;
result.output_history = y_history;
result.output_disagreement = output_disagreement;
result.event_history = event_history;
result.trigger_value_history = trigger_value_history;
result.threshold_history = threshold_history;
result.events_per_agent = events_per_agent;
result.cumulative_events = cumsum(event_history, 2);
result.total_events = sum(events_per_agent);
result.final_state = x;
result.final_sampled_state = x_hat;
result.final_output_disagreement = output_disagreement(end);
end
