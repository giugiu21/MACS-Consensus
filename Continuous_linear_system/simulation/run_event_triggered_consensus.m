function result = run_event_triggered_consensus( ...
    x0, agent, graph, K, time, dt, opts)
%RUN_EVENT_TRIGGERED_CONSENSUS
% Event-triggered consensus simulation.
%
% Supported control architectures:
%
%   opts.control_case = "leaderless"
%   opts.control_case = "leader_follower"
%
% Supported systems:
%
%   opts.system = "no-equilibrium"
%
%       An assigned reference x_desired(t) is available.
%
%       leaderless:
%           every agent tracks x_desired directly while the distributed
%           consensus term uses sampled states x_hat.
%
%       leader_follower:
%           only the leader tracks x_desired directly; the followers
%           receive the reference indirectly through the network.
%
%   opts.system = "equilibrium"
%
%       leaderless:
%           standard event-triggered free consensus.
%
%       leader_follower:
%           the leader is autonomous:
%
%               x_dot_L = A*x_L
%               u_L = 0
%
%           and the followers synchronize with its natural trajectory
%           using the last transmitted states.
%
% The sampled state x_hat is updated only when an event is generated.
%
% The actual control law is implemented in
% event_triggered_linear_rhs.

    %% Dimensions

    N = graph.N;
    n = agent.n;
    num_steps = numel(time);

    C_global = kron(eye(N), agent.C);
    A_global = kron(eye(N), agent.A);

    %% Validate simulation inputs

    if num_steps < 1
        error( ...
            'run_event_triggered_consensus:EmptyTimeVector', ...
            'time must contain at least one element.');
    end

    if ~isscalar(dt) || ~isfinite(dt) || dt <= 0
        error( ...
            'run_event_triggered_consensus:InvalidTimeStep', ...
            'dt must be a finite positive scalar.');
    end

    %% Default options

    if nargin < 7 || isempty(opts)
        opts = struct();
    end

    if ~isfield(opts, 'control_case') || isempty(opts.control_case)
        opts.control_case = "leaderless";
    end

    if ~isfield(opts, 'system') || isempty(opts.system)
        opts.system = "equilibrium";
    end

    if ~isfield(opts, 'sigma') || isempty(opts.sigma)
        opts.sigma = 0.05;
    end

    if ~isfield(opts, 'epsilon_trigger') || ...
            isempty(opts.epsilon_trigger)

        opts.epsilon_trigger = 1e-5;
    end

    if ~isfield(opts, 'trigger_type') || ...
            isempty(opts.trigger_type)

        opts.trigger_type = "relative";
    end

    if ~isfield(opts, 'trigger_params') || ...
            isempty(opts.trigger_params)

        opts.trigger_params = struct();
    end

    if ~isfield(opts, 'hold_mode') || isempty(opts.hold_mode)
        opts.hold_mode = "zoh";
    end

    control_case = lower(string(opts.control_case));
    system_type = lower(string(opts.system));
    trigger_type = string(opts.trigger_type);
    hold_mode = lower(string(opts.hold_mode));

    sigma = opts.sigma;
    epsilon_trigger = opts.epsilon_trigger;
    trigger_params = opts.trigger_params;

    %% Validate control case

    valid_control_cases = [
        "leaderless", ...
        "leader_follower"
    ];

    if ~ismember(control_case, valid_control_cases)
        error( ...
            'run_event_triggered_consensus:InvalidControlCase', ...
            ['opts.control_case must be "leaderless" ', ...
             'or "leader_follower".']);
    end

    %% Validate system type

    valid_system_types = [
        "equilibrium", ...
        "no-equilibrium"
    ];

    if ~ismember(system_type, valid_system_types)
        error( ...
            'run_event_triggered_consensus:InvalidSystem', ...
            ['opts.system must be "equilibrium" ', ...
             'or "no-equilibrium".']);
    end

    %% Validate hold mode

    valid_hold_modes = [
        "zoh", ...
        "model_based"
    ];

    if ~ismember(hold_mode, valid_hold_modes)
        error( ...
            'run_event_triggered_consensus:InvalidHoldMode', ...
            'opts.hold_mode must be "zoh" or "model_based".');
    end

    %% Validate scalar trigger parameters

    if ~isscalar(sigma) || ~isfinite(sigma) || sigma < 0
        error( ...
            'run_event_triggered_consensus:InvalidSigma', ...
            'opts.sigma must be a finite nonnegative scalar.');
    end

    if ~isscalar(epsilon_trigger) || ...
            ~isfinite(epsilon_trigger) || ...
            epsilon_trigger < 0

        error( ...
            'run_event_triggered_consensus:InvalidEpsilon', ...
            ['opts.epsilon_trigger must be a finite ', ...
             'nonnegative scalar.']);
    end

    %% Validate architecture-dependent options

    leader_id = [];
    x_desired_fun = [];

    % leader_id is required only in leader-follower mode.
    if control_case == "leader_follower"

        if ~isfield(opts, 'leader_id') || isempty(opts.leader_id)
            error( ...
                'run_event_triggered_consensus:MissingLeader', ...
                ['Missing opts.leader_id for ', ...
                 'leader-follower control.']);
        end

        leader_id = opts.leader_id;

        if ~isscalar(leader_id) || ...
                leader_id ~= round(leader_id) || ...
                leader_id < 1 || ...
                leader_id > N

            error( ...
                'run_event_triggered_consensus:InvalidLeader', ...
                ['opts.leader_id must be an integer ', ...
                 'between 1 and N.']);
        end
    end

    % Every no-equilibrium case requires an assigned reference.
    if system_type == "no-equilibrium"

        required_fields = {
            'K_reference', ...
            'x_desired_fun'
        };

        for field_id = 1:numel(required_fields)

            field_name = required_fields{field_id};

            if ~isfield(opts, field_name) || ...
                    isempty(opts.(field_name))

                error( ...
                    'run_event_triggered_consensus:MissingOption', ...
                    ['Missing opts.%s for no-equilibrium ', ...
                     '%s control.'], ...
                    field_name, ...
                    char(control_case));
            end
        end

        x_desired_fun = opts.x_desired_fun;

        if ~isa(x_desired_fun, 'function_handle')
            error( ...
                'run_event_triggered_consensus:' + ...
                "InvalidReferenceFunction", ...
                'opts.x_desired_fun must be a function handle.');
        end
    end

    %% Initial conditions

    x = x0(:);
    x_hat = x0(:);
    % At the first instant, the previous state equals the initial state.
    x_previous = x;

    % Initializing it to -Inf allows every agent to trigger immediately
    % if the threshold condition is satisfied at the first step.
    last_event_time = -inf(N, 1);

    if numel(x) ~= N * n
        error( ...
            'run_event_triggered_consensus:BadInitialCondition', ...
            'x0 must contain N * agent.n elements.');
    end

    if any(~isfinite(x))
        error( ...
            'run_event_triggered_consensus:' + ...
            "NonFiniteInitialCondition", ...
            'x0 must contain only finite values.');
    end

    %% Preallocate general histories

    x_history = zeros(N * n, num_steps);

    sampled_state_history = ...
        zeros(N * n, num_steps);

    y_history = zeros(N, num_steps);

    output_disagreement = zeros(1, num_steps);

    event_history = false(N, num_steps);

    trigger_value_history = ...
        zeros(N, num_steps);

    threshold_history = ...
        zeros(N, num_steps);

    %% Assigned-reference tracking histories

    tracking_error_history = nan(1, num_steps);

    leader_tracking_error_history = ...
        nan(1, num_steps);

    reference_history = nan(n, num_steps);

    desired_output_history = ...
        nan(1, num_steps);

    reference_state_error_history = ...
        nan(N, num_steps);

    reference_output_error_history = ...
        nan(N, num_steps);

    %% Autonomous-leader synchronization histories

    leader_state_history = nan(n, num_steps);

    leader_output_history = ...
        nan(1, num_steps);

    leader_state_error_history = ...
        nan(N, num_steps);

    leader_output_error_history = ...
        nan(N, num_steps);

    sampled_leader_state_error_history = ...
        nan(N, num_steps);

    network_leader_state_error = ...
        nan(1, num_steps);

    network_leader_output_error = ...
        nan(1, num_steps);

    %% Simulation loop

    for step_id = 1:num_steps

        current_time = time(step_id);

        %% Store current states

        x_history(:, step_id) = x;

        sampled_state_history(:, step_id) = ...
            x_hat;

        %% Current outputs

        y = C_global * x;

        y_history(:, step_id) = y;

        %% Standard output disagreement

        output_disagreement(step_id) = ...
            norm(y - mean(y) * ones(N, 1));

        %% Performance metrics

        if system_type == "no-equilibrium"

            %% Assigned-reference tracking

            x_desired = x_desired_fun(current_time);
            x_desired = x_desired(:);

            if numel(x_desired) ~= n
                error( ...
                    'run_event_triggered_consensus:BadReference', ...
                    ['opts.x_desired_fun must return ', ...
                     'agent.n elements.']);
            end

            if any(~isfinite(x_desired))
                error( ...
                    'run_event_triggered_consensus:' + ...
                    "NonFiniteReference", ...
                    ['opts.x_desired_fun returned ', ...
                     'non-finite values.']);
            end

            y_desired = agent.C * x_desired;

            reference_history(:, step_id) = ...
                x_desired;

            desired_output_history(step_id) = ...
                y_desired;

            tracking_error_history(step_id) = ...
                norm(y - y_desired * ones(N, 1));

            for agent_id = 1:N

                idx_agent = ...
                    (agent_id - 1) * n + 1 : ...
                    agent_id * n;

                x_agent = x(idx_agent);
                y_agent = agent.C * x_agent;

                reference_state_error_history( ...
                    agent_id, step_id) = ...
                    norm(x_agent - x_desired);

                reference_output_error_history( ...
                    agent_id, step_id) = ...
                    norm(y_agent - y_desired);
            end

            if control_case == "leader_follower"

                idx_leader = ...
                    (leader_id - 1) * n + 1 : ...
                    leader_id * n;

                x_leader = x(idx_leader);

                leader_tracking_error_history(step_id) = ...
                    norm(x_leader - x_desired);
            end

        elseif system_type == "equilibrium" && ...
                control_case == "leader_follower"

            %% Autonomous-leader synchronization

            idx_leader = ...
                (leader_id - 1) * n + 1 : ...
                leader_id * n;

            x_leader = x(idx_leader);
            x_hat_leader = x_hat(idx_leader);

            y_leader = agent.C * x_leader;

            leader_state_history(:, step_id) = ...
                x_leader;

            leader_output_history(step_id) = ...
                y_leader;

            for agent_id = 1:N

                idx_agent = ...
                    (agent_id - 1) * n + 1 : ...
                    agent_id * n;

                x_agent = x(idx_agent);
                x_hat_agent = x_hat(idx_agent);

                y_agent = agent.C * x_agent;

                leader_state_error_history( ...
                    agent_id, step_id) = ...
                    norm(x_agent - x_leader);

                leader_output_error_history( ...
                    agent_id, step_id) = ...
                    norm(y_agent - y_leader);

                sampled_leader_state_error_history( ...
                    agent_id, step_id) = ...
                    norm(x_hat_agent - x_hat_leader);
            end

            network_leader_state_error(step_id) = ...
                norm(leader_state_error_history(:, step_id));

            network_leader_output_error(step_id) = ...
                norm(leader_output_error_history(:, step_id));
        end

        %% Time-dependent trigger parameters

        step_trigger_params = trigger_params;

        trigger_type_key = lower(char(trigger_type));

        if strcmp(trigger_type_key, 'exponential') || ...
                strcmp( ...
                    trigger_type_key, ...
                    'garcia_exponential')

            step_trigger_params.time = current_time;
        end

        %% Evaluate event-trigger conditions

        % Every agent evaluates its trigger.
        %
        % This includes the autonomous leader in equilibrium
        % leader-follower mode because the leader must keep updating
        % its transmitted state while oscillating.
        [x_hat, events, trigger_values, thresholds, last_event_time] = ...
            update_event_triggers( ...
                x, ...
                x_previous, ...
                x_hat, ...
                graph, ...
                sigma, ...
                epsilon_trigger, ...
                n, ...
                trigger_type, ...
                step_trigger_params, ...
                current_time, ...
                last_event_time);

        x_hat = x_hat(:);

        if numel(x_hat) ~= N * n
            error( ...
                'run_event_triggered_consensus:BadSampledState', ...
                ['update_event_triggers must return ', ...
                 'N * agent.n sampled-state elements.']);
        end

        event_history(:, step_id) = ...
            logical(events(:));

        trigger_value_history(:, step_id) = ...
            trigger_values(:);

        threshold_history(:, step_id) = ...
            thresholds(:);

        %% Propagate dynamics
        x_current = x;

        if step_id < num_steps

            dx = event_triggered_linear_rhs( ...
                x, ...
                x_hat, ...
                agent, ...
                graph, ...
                K, ...
                opts, ...
                current_time);

            dx = dx(:);

            if numel(dx) ~= N * n
                error( ...
                    'run_event_triggered_consensus:' + ...
                    "BadStateDerivative", ...
                    ['event_triggered_linear_rhs must return ', ...
                     'N * agent.n elements.']);
            end

            if any(~isfinite(dx))
                error( ...
                    'run_event_triggered_consensus:' + ...
                    "NonFiniteDerivative", ...
                    ['event_triggered_linear_rhs returned ', ...
                     'non-finite values.']);
            end

            % Forward Euler integration.
            x = x + dt * dx;

            if hold_mode == "model_based"

                x_hat = x_hat + dt * (A_global * x_hat);
            end
        end
        x_previous = x_current;
    end

    %% Communication statistics

    events_per_agent = ...
        sum(event_history, 2);

    cumulative_events = ...
        cumsum(event_history, 2);

    total_events = ...
        sum(events_per_agent);

    number_of_intervals = ...
        max(num_steps - 1, 1);

    event_rate_per_agent = ...
        events_per_agent / number_of_intervals;

    %% Result structure

    result = struct();

    result.time = time;

    result.x_history = x_history;

    result.sampled_state_history = ...
        sampled_state_history;

    result.y_history = y_history;
    result.output_history = y_history;

    result.output_disagreement = ...
        output_disagreement;

    %% Assigned-reference tracking fields

    result.tracking_error_history = ...
        tracking_error_history;

    result.leader_tracking_error_history = ...
        leader_tracking_error_history;

    result.reference_history = ...
        reference_history;

    result.desired_output_history = ...
        desired_output_history;

    result.reference_state_error_history = ...
        reference_state_error_history;

    result.reference_output_error_history = ...
        reference_output_error_history;

    %% Autonomous-leader synchronization fields

    result.leader_state_history = ...
        leader_state_history;

    result.leader_output_history = ...
        leader_output_history;

    result.leader_state_error_history = ...
        leader_state_error_history;

    result.leader_output_error_history = ...
        leader_output_error_history;

    result.sampled_leader_state_error_history = ...
        sampled_leader_state_error_history;

    result.network_leader_state_error = ...
        network_leader_state_error;

    result.network_leader_output_error = ...
        network_leader_output_error;

    %% Event-trigger fields

    result.event_history = event_history;

    result.communication_history = ...
        event_history;

    result.trigger_value_history = ...
        trigger_value_history;

    result.threshold_history = ...
        threshold_history;

    result.events_per_agent = ...
        events_per_agent;

    result.cumulative_events = ...
        cumulative_events;

    result.total_events = ...
        total_events;

    result.event_rate_per_agent = ...
        event_rate_per_agent;

    %% Final values

    result.final_state = x;
    result.final_sampled_state = x_hat;

    result.final_output_disagreement = ...
        output_disagreement(end);

    if system_type == "no-equilibrium"

        result.final_tracking_error = ...
            tracking_error_history(end);

        result.final_reference_state_errors = ...
            reference_state_error_history(:, end);

        result.final_reference_output_errors = ...
            reference_output_error_history(:, end);

        if control_case == "leader_follower"

            result.final_leader_tracking_error = ...
                leader_tracking_error_history(end);

        else

            result.final_leader_tracking_error = [];
        end

    else

        result.final_tracking_error = [];
        result.final_leader_tracking_error = [];

        result.final_reference_state_errors = [];
        result.final_reference_output_errors = [];
    end

    if control_case == "leader_follower" && ...
            system_type == "equilibrium"

        result.final_leader_state_errors = ...
            leader_state_error_history(:, end);

        result.final_leader_output_errors = ...
            leader_output_error_history(:, end);

    else

        result.final_leader_state_errors = [];
        result.final_leader_output_errors = [];
    end

    %% Configuration information

    result.communication_mode = ...
        "event_triggered";

    result.hold_mode = hold_mode;

    result.control_case = control_case;
    result.system = system_type;

    result.trigger_type = trigger_type;
    result.sigma = sigma;

    result.epsilon_trigger = ...
        epsilon_trigger;

    if control_case == "leader_follower"
        result.leader_id = leader_id;
    else
        result.leader_id = [];
    end
end