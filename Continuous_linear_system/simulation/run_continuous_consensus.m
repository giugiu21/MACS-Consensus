function result = run_continuous_consensus( ...
    x0, agent, graph, K, time, dt, opts)
%RUN_CONTINUOUS_CONSENSUS
% Continuous-communication consensus simulation.
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
%           every agent tracks x_desired directly while also applying
%           the distributed consensus controller.
%
%       leader_follower:
%           only the leader tracks x_desired directly; the followers
%           receive the reference indirectly through the network.
%
%   opts.system = "equilibrium"
%
%       leaderless:
%           standard free consensus, with no external reference.
%
%       leader_follower:
%           the leader is autonomous:
%
%               x_dot_L = A*x_L
%               u_L = 0
%
%           and the followers synchronize with its natural trajectory.
%
% The actual control law is implemented in continuous_linear_rhs.

    %% Dimensions

    N = graph.N;
    n = agent.n;
    num_steps = numel(time);

    C_global = kron(eye(N), agent.C);

    %% Validate simulation inputs

    if num_steps < 1
        error( ...
            'run_continuous_consensus:EmptyTimeVector', ...
            'time must contain at least one element.');
    end

    if ~isscalar(dt) || ~isfinite(dt) || dt <= 0
        error( ...
            'run_continuous_consensus:InvalidTimeStep', ...
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

    control_case = lower(string(opts.control_case));
    system_type = lower(string(opts.system));

    %% Validate control case

    valid_control_cases = [
        "leaderless", ...
        "leader_follower"
    ];

    if ~ismember(control_case, valid_control_cases)
        error( ...
            'run_continuous_consensus:InvalidControlCase', ...
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
            'run_continuous_consensus:InvalidSystem', ...
            ['opts.system must be "equilibrium" ', ...
             'or "no-equilibrium".']);
    end

    %% Validate architecture-dependent options

    leader_id = [];
    x_desired_fun = [];

    % leader_id is required only in leader-follower mode.
    if control_case == "leader_follower"

        if ~isfield(opts, 'leader_id') || isempty(opts.leader_id)
            error( ...
                'run_continuous_consensus:MissingLeader', ...
                ['Missing opts.leader_id in ', ...
                 'leader-follower mode.']);
        end

        leader_id = opts.leader_id;

        if ~isscalar(leader_id) || ...
                leader_id ~= round(leader_id) || ...
                leader_id < 1 || ...
                leader_id > N

            error( ...
                'run_continuous_consensus:InvalidLeader', ...
                ['opts.leader_id must be an integer ', ...
                 'between 1 and N.']);
        end
    end

    % Every no-equilibrium case requires an assigned reference.
    %
    % In leaderless mode, all agents use it.
    % In leader-follower mode, only the leader uses it directly.
    if system_type == "no-equilibrium"

        required_fields = {
            'x_desired_fun', ...
            'K_reference'
        };

        for field_id = 1:numel(required_fields)

            field_name = required_fields{field_id};

            if ~isfield(opts, field_name) || ...
                    isempty(opts.(field_name))

                error( ...
                    'run_continuous_consensus:MissingOption', ...
                    ['Missing opts.%s for no-equilibrium ', ...
                     '%s mode.'], ...
                    field_name, ...
                    char(control_case));
            end
        end

        x_desired_fun = opts.x_desired_fun;

        if ~isa(x_desired_fun, 'function_handle')
            error( ...
                'run_continuous_consensus:InvalidReferenceFunction', ...
                'opts.x_desired_fun must be a function handle.');
        end
    end

    %% Initial condition

    x = x0(:);

    if numel(x) ~= N * n
        error( ...
            'run_continuous_consensus:BadInitialCondition', ...
            'x0 must contain N * agent.n elements.');
    end

    if any(~isfinite(x))
        error( ...
            'run_continuous_consensus:NonFiniteInitialCondition', ...
            'x0 must contain only finite values.');
    end

    %% Preallocate general histories

    x_history = zeros(N * n, num_steps);
    y_history = zeros(N, num_steps);

    output_disagreement = zeros(1, num_steps);

    % In continuous communication, every agent makes its current state
    % available at every simulation instant.
    communication_history = true(N, num_steps);

    % Continuous mode does not generate event-triggered transmissions.
    event_history = false(N, num_steps);

    %% Assigned-reference tracking histories

    % Used in every no-equilibrium case.
    tracking_error_history = nan(1, num_steps);

    % Meaningful only in no-equilibrium leader-follower mode.
    leader_tracking_error_history = nan(1, num_steps);

    reference_history = nan(n, num_steps);
    desired_output_history = nan(1, num_steps);

    % Individual state and output tracking errors for all agents.
    reference_state_error_history = nan(N, num_steps);
    reference_output_error_history = nan(N, num_steps);

    %% Autonomous-leader synchronization histories

    % Used only in equilibrium leader-follower mode.
    leader_state_history = nan(n, num_steps);
    leader_output_history = nan(1, num_steps);

    leader_state_error_history = nan(N, num_steps);
    leader_output_error_history = nan(N, num_steps);

    network_leader_state_error = nan(1, num_steps);
    network_leader_output_error = nan(1, num_steps);

    %% Simulation loop

    for step_id = 1:num_steps

        current_time = time(step_id);

        %% Save current state

        x_history(:, step_id) = x;

        %% Compute current outputs

        y = C_global * x;

        y_history(:, step_id) = y;

        %% Standard network disagreement

        output_disagreement(step_id) = ...
            norm(y - mean(y) * ones(N, 1));

        %% Performance metrics

        if system_type == "no-equilibrium"

            %% Assigned-reference tracking

            x_desired = x_desired_fun(current_time);
            x_desired = x_desired(:);

            if numel(x_desired) ~= n
                error( ...
                    'run_continuous_consensus:BadReference', ...
                    ['opts.x_desired_fun must return ', ...
                     'agent.n elements.']);
            end

            if any(~isfinite(x_desired))
                error( ...
                    'run_continuous_consensus:NonFiniteReference', ...
                    ['opts.x_desired_fun returned ', ...
                     'non-finite values.']);
            end

            y_desired = agent.C * x_desired;

            reference_history(:, step_id) = x_desired;
            desired_output_history(step_id) = y_desired;

            % Global output tracking error.
            tracking_error_history(step_id) = ...
                norm(y - y_desired * ones(N, 1));

            % Individual state and output tracking errors.
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

            % In leader-follower mode, also store the direct tracking
            % error of the selected leader.
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

            %% Synchronization with autonomous leader

            idx_leader = ...
                (leader_id - 1) * n + 1 : ...
                leader_id * n;

            x_leader = x(idx_leader);
            y_leader = agent.C * x_leader;

            leader_state_history(:, step_id) = x_leader;
            leader_output_history(step_id) = y_leader;

            for agent_id = 1:N

                idx_agent = ...
                    (agent_id - 1) * n + 1 : ...
                    agent_id * n;

                x_agent = x(idx_agent);
                y_agent = agent.C * x_agent;

                leader_state_error_history( ...
                    agent_id, step_id) = ...
                    norm(x_agent - x_leader);

                leader_output_error_history( ...
                    agent_id, step_id) = ...
                    norm(y_agent - y_leader);
            end

            network_leader_state_error(step_id) = ...
                norm(leader_state_error_history(:, step_id));

            network_leader_output_error(step_id) = ...
                norm(leader_output_error_history(:, step_id));
        end

        %% State propagation

        if step_id < num_steps

            dx = continuous_linear_rhs( ...
                x, ...
                agent, ...
                graph, ...
                K, ...
                opts, ...
                current_time);

            dx = dx(:);

            if numel(dx) ~= N * n
                error( ...
                    'run_continuous_consensus:BadStateDerivative', ...
                    ['continuous_linear_rhs must return ', ...
                     'N * agent.n elements.']);
            end

            if any(~isfinite(dx))
                error( ...
                    'run_continuous_consensus:NonFiniteDerivative', ...
                    ['continuous_linear_rhs returned ', ...
                     'non-finite values.']);
            end

            % Forward Euler integration.
            x = x + dt * dx;
        end
    end

    %% Standardized result structure

    result = struct();

    result.time = time;

    result.x_history = x_history;
    result.y_history = y_history;
    result.output_history = y_history;

    result.output_disagreement = output_disagreement;

    %% Assigned-reference tracking fields

    result.tracking_error_history = ...
        tracking_error_history;

    result.leader_tracking_error_history = ...
        leader_tracking_error_history;

    result.reference_history = reference_history;

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

    result.network_leader_state_error = ...
        network_leader_state_error;

    result.network_leader_output_error = ...
        network_leader_output_error;

    %% Communication fields

    result.communication_history = ...
        communication_history;

    result.event_history = event_history;

    result.communications_per_agent = ...
        sum(communication_history, 2);

    result.total_communications = ...
        sum(communication_history, 'all');

    %% Final values

    result.final_state = x;

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

    result.communication_mode = "continuous";
    result.control_case = control_case;
    result.system = system_type;

    if control_case == "leader_follower"
        result.leader_id = leader_id;
    else
        result.leader_id = [];
    end
end