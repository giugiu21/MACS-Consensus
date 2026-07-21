function [x_hat, events, trigger_values, thresholds, last_event_time] = update_event_triggers( ...
    x, x_previous, x_hat, graph, sigma, epsilon_trigger, n, trigger_type, trigger_params, current_time, last_event_time)
% update sampled states according to selected local event-triggering rule

N = graph.N;
events = zeros(N, 1);
trigger_values = zeros(N, 1);
thresholds = zeros(N, 1);

x_matrix = reshape(x, n, N);
x_previous_matrix = reshape(x_previous, n, N);
state_rate_matrix = ...
    (x_matrix - x_previous_matrix) / trigger_params.dt;
local_motion_rate = zeros(n, N);

if nargin < 8 || isempty(trigger_type)
    trigger_type = 'relative';
end

if nargin < 8 || isempty(trigger_params)
    trigger_params = struct();
end

if nargin < 10 || isempty(current_time)
    current_time = 0;
end

if nargin < 11 || isempty(last_event_time)
    last_event_time = -inf(N, 1);
end

last_event_time = last_event_time(:);

if numel(last_event_time) ~= N
    error( ...
        'update_event_triggers:BadLastEventTime', ...
        'last_event_time must contain one value per agent.');
end

minimum_inter_event_time = get_optional_scalar_param( ...
    trigger_params, ...
    'minimum_inter_event_time', ...
    0);

%Calcola, per ogni agente, quanto il suo moto locale differisce dal moto medio dei suoi vicini.
for agent_id = 1:N

    degree_i = sum(graph.Adj(agent_id, :));

    if degree_i > 0

        neighbor_mean_rate = zeros(n, 1);

        for neighbor_id = 1:N

            a_ij = graph.Adj(agent_id, neighbor_id);

            if a_ij == 0
                continue;
            end

            neighbor_mean_rate = ...
                neighbor_mean_rate ...
                + a_ij * state_rate_matrix(:, neighbor_id);
        end

        neighbor_mean_rate = ...
            neighbor_mean_rate / degree_i;

        %Calcola la differenza tra: il moto dell’agente corrente e il moto medio dei suoi vicini.
        local_motion_rate(:, agent_id) = ...
            state_rate_matrix(:, agent_id) - neighbor_mean_rate;

    else
        %se non ci sono vicini si usa il moto assoluto dell'agente
        local_motion_rate(:, agent_id) = ...
            state_rate_matrix(:, agent_id);
    end
end



for agent_id = 1:N
    idx_i = (agent_id - 1) * n + 1 : agent_id * n;

    local_motion_rate_vector = local_motion_rate(:, agent_id);

    state_vector = x(idx_i);
    previous_state_vector = x_previous(idx_i);
    %errore tra l'ultimo stato trasmesso e quello attuale
    error_vector = x_hat(idx_i) - x(idx_i); 
    %disagreement con i vicini in base allo stato attuale dell'agente 
    disagreement_vector = compute_local_disagreement( ...
        x, graph, n, agent_id); 

    [threshold_condition, trigger_value, threshold] = ...
        evaluate_trigger_condition( ...
        error_vector, disagreement_vector, sigma, epsilon_trigger, n, ...
        trigger_type, trigger_params, state_vector, previous_state_vector, local_motion_rate_vector);

    trigger_values(agent_id) = trigger_value;
    thresholds(agent_id) = threshold;

    time_since_last_event = current_time - last_event_time(agent_id);

    time_condition = time_since_last_event >= minimum_inter_event_time;

    should_trigger = threshold_condition && time_condition;

    if should_trigger

        x_hat(idx_i) = x(idx_i);
        events(agent_id) = 1;

        last_event_time(agent_id) = current_time;
    end
end
end

function value = get_optional_scalar_param( ...
    parameter_struct, field_name, default_value)
% Read an optional finite nonnegative scalar parameter.

    if isfield(parameter_struct, field_name)
        value = parameter_struct.(field_name);
    else
        value = default_value;
    end

    if ~isscalar(value) || ...
            ~isfinite(value) || ...
            value < 0

        error( ...
            'update_event_triggers:BadScalarParameter', ...
            ['trigger_params.%s must be a finite ' ...
             'nonnegative scalar.'], ...
            field_name);
    end
end