function [x_hat, events, trigger_values, thresholds] = update_event_triggers( ...
    x, x_hat, graph, sigma, epsilon_trigger, n, trigger_type, trigger_params)
% update sampled states according to selected local event-triggering rule

N = graph.N;
events = zeros(N, 1);
trigger_values = zeros(N, 1);
thresholds = zeros(N, 1);

if nargin < 7 || isempty(trigger_type)
    trigger_type = 'relative';
end

if nargin < 8 || isempty(trigger_params)
    trigger_params = struct();
end

for agent_id = 1:N
    idx_i = (agent_id - 1) * n + 1 : agent_id * n;

    state_vector = x(idx_i);
    error_vector = x_hat(idx_i) - x(idx_i);
    disagreement_vector = compute_local_disagreement( ...
        x_hat, graph, n, agent_id);

    [should_trigger, trigger_value, threshold] = ...
        evaluate_trigger_condition( ...
        error_vector, disagreement_vector, sigma, epsilon_trigger, n, ...
        trigger_type, trigger_params, state_vector);

    trigger_values(agent_id) = trigger_value;
    thresholds(agent_id) = threshold;

    if should_trigger
        x_hat(idx_i) = x(idx_i);
        events(agent_id) = 1;
    end
end
end
