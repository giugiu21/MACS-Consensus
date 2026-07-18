function dx = event_triggered_linear_rhs( ...
    x, x_hat, agent, graph, K_consensus, opts, current_time)
%EVENT_TRIGGERED_LINEAR_RHS
% Compute event-triggered multi-agent dynamics.
%
% The physical dynamics use the real state x.
% The distributed consensus controller uses the transmitted state x_hat.
%
% Supported systems:
%
%   opts.system = "no-equilibrium"
%       The leader tracks an external desired state.
%
%   opts.system = "equilibrium"
%       The leader is autonomous and the followers synchronize with its
%       natural oscillatory trajectory.

    %% Dimensions

    N = graph.N;
    n = agent.n;
    m_input = agent.m_input;

    A_global = kron(eye(N), agent.A);
    B_global = kron(eye(N), agent.B);

    %% Read options

    if ~isfield(opts, 'control_case') || isempty(opts.control_case)
        control_case = "leaderless";
    else
        control_case = lower(string(opts.control_case));
    end

    if ~isfield(opts, 'system') || isempty(opts.system)
        system_type = "no-equilibrium";
    else
        system_type = lower(string(opts.system));
    end

    valid_control_cases = ["leaderless", "leader_follower"];
    valid_system_types = ["equilibrium", "no-equilibrium"];

    if ~ismember(control_case, valid_control_cases)
        error( ...
            'event_triggered_linear_rhs:InvalidControlCase', ...
            ['opts.control_case must be "leaderless" ', ...
             'or "leader_follower".']);
    end

    if ~ismember(system_type, valid_system_types)
        error( ...
            'event_triggered_linear_rhs:InvalidSystem', ...
            'opts.system must be "equilibrium" or "no-equilibrium".');
    end

    %% Control architecture

    switch control_case

        case "leaderless"

            %% Event-triggered diffusive consensus

            u = compute_event_triggered_control( ...
                x_hat, ...
                graph, ...
                K_consensus, ...
                n, ...
                m_input);

            %% Optional common-reference tracking
            %
            % The consensus term uses x_hat.
            % The reference feedback uses the real local state x.

            if system_type == "no-equilibrium"

                %% Validate reference gain

                if ~isfield(opts, 'K_reference') || ...
                        isempty(opts.K_reference)

                    error( ...
                        'event_triggered_linear_rhs:MissingReferenceGain', ...
                        ['opts.K_reference is required for ', ...
                        'no-equilibrium leaderless tracking.']);
                end

                K_reference = validate_reference_gain( ...
                    opts.K_reference, ...
                    m_input, ...
                    n);

                %% Validate desired trajectory

                if ~isfield(opts, 'x_desired_fun') || ...
                        isempty(opts.x_desired_fun)

                    error( ...
                        'event_triggered_linear_rhs:MissingReference', ...
                        ['opts.x_desired_fun is required for ', ...
                        'no-equilibrium leaderless tracking.']);
                end

                x_desired = opts.x_desired_fun(current_time);

                x_desired = validate_state_vector( ...
                    x_desired, ...
                    n, ...
                    'opts.x_desired_fun');

                %% Feedforward

                if isfield(opts, 'u_desired') && ...
                        ~isempty(opts.u_desired)

                    u_feedforward = validate_input_vector( ...
                        opts.u_desired, ...
                        m_input, ...
                        'opts.u_desired');

                else

                    u_feedforward = zeros(m_input, 1);
                end

                %% Global reference feedback

                x_desired_global = repmat(x_desired, N, 1);

                u_reference_global = ...
                    -kron(eye(N), K_reference) ...
                    * (x - x_desired_global);

                u = u ...
                    + u_reference_global ...
                    + repmat(u_feedforward, N, 1);
            end

        case "leader_follower"

            leader_id = validate_leader_id(opts, N);

            switch system_type

                case "no-equilibrium"

                    %% Damped leader-follower
                    %
                    % Preserve the original behavior.

                    u = compute_event_triggered_control( ...
                        x_hat, ...
                        graph, ...
                        K_consensus, ...
                        n, ...
                        m_input);

                    %% Validate reference gain

                    if ~isfield(opts, 'K_reference') || ...
                            isempty(opts.K_reference)

                        error( ...
                            'event_triggered_linear_rhs:' + ...
                            "MissingReferenceGain", ...
                            ['opts.K_reference is required for ', ...
                             'no-equilibrium leader-follower control.']);
                    end

                    K_reference = validate_reference_gain( ...
                        opts.K_reference, ...
                        m_input, ...
                        n);

                    %% Validate desired-state function

                    if ~isfield(opts, 'x_desired_fun') || ...
                            isempty(opts.x_desired_fun)

                        error( ...
                            'event_triggered_linear_rhs:' + ...
                            "MissingReference", ...
                            ['opts.x_desired_fun is required for ', ...
                             'no-equilibrium leader-follower control.']);
                    end

                    %% Desired state

                    x_desired = ...
                        opts.x_desired_fun(current_time);

                    x_desired = validate_state_vector( ...
                        x_desired, ...
                        n, ...
                        'opts.x_desired_fun');

                    %% Current real leader state

                    idx_leader = ...
                        (leader_id - 1) * n + 1 : ...
                        leader_id * n;

                    x_leader = x(idx_leader);

                    %% Reference feedback from the real leader state

                    u_reference = ...
                        -K_reference * ...
                        (x_leader - x_desired);

                    %% Equilibrium feedforward

                    if isfield(opts, 'u_desired') && ...
                            ~isempty(opts.u_desired)

                        u_feedforward = validate_input_vector( ...
                            opts.u_desired, ...
                            m_input, ...
                            'opts.u_desired');

                    else

                        u_feedforward = ...
                            zeros(m_input, 1);
                    end

                    % Preserve your current implementation:
                    % feedforward applied to every agent.
                    u = u + repmat(u_feedforward, N, 1);

                    %% Apply reference feedback only to the leader

                    idx_u_leader = ...
                        (leader_id - 1) * m_input + 1 : ...
                        leader_id * m_input;

                    u(idx_u_leader) = ...
                        u(idx_u_leader) + u_reference;

                case "equilibrium"

                    %% Undamped leader-follower
                    %
                    % The leader is autonomous but must continue to
                    % communicate its state through the trigger mechanism.
                    %
                    % Followers use the latest transmitted states.

                    L_control = graph.L;

                    % The leader does not react to followers.
                    L_control(leader_id, :) = 0;

                    % Event-triggered diffusive controller.
                    u = -kron(L_control, K_consensus) * x_hat;

                    %% Explicitly impose zero leader input

                    idx_u_leader = ...
                        (leader_id - 1) * m_input + 1 : ...
                        leader_id * m_input;

                    u(idx_u_leader) = zeros(m_input, 1);
            end
    end

    %% Physical dynamics

    % The uncontrolled and controlled physical dynamics always evolve
    % using the real state x. Only the control uses x_hat.
    dx = A_global * x + B_global * u;
end


function leader_id = validate_leader_id(opts, N)
%VALIDATE_LEADER_ID Validate and return the leader identifier.

    if ~isfield(opts, 'leader_id') || isempty(opts.leader_id)

        error( ...
            'event_triggered_linear_rhs:MissingLeader', ...
            'opts.leader_id is required in leader-follower mode.');
    end

    leader_id = opts.leader_id;

    if ~isscalar(leader_id) || ...
            leader_id ~= round(leader_id) || ...
            leader_id < 1 || ...
            leader_id > N

        error( ...
            'event_triggered_linear_rhs:BadLeader', ...
            'opts.leader_id must be an integer between 1 and graph.N.');
    end
end


function vector = validate_state_vector(vector, n, field_name)
%VALIDATE_STATE_VECTOR Validate an n-dimensional state vector.

    vector = vector(:);

    if numel(vector) ~= n

        error( ...
            'event_triggered_linear_rhs:BadStateVector', ...
            '%s must return agent.n elements.', ...
            field_name);
    end
end


function vector = validate_input_vector(vector, m_input, field_name)
%VALIDATE_INPUT_VECTOR Validate an input vector.

    vector = vector(:);

    if numel(vector) ~= m_input

        error( ...
            'event_triggered_linear_rhs:BadInputVector', ...
            '%s must contain agent.m_input elements.', ...
            field_name);
    end
end


function K_reference = validate_reference_gain( ...
    K_reference, m_input, n)
%VALIDATE_REFERENCE_GAIN Validate the leader reference feedback gain.

    if ~isequal(size(K_reference), [m_input, n])

        error( ...
            'event_triggered_linear_rhs:BadReferenceGain', ...
            ['opts.K_reference must have size ', ...
             'agent.m_input-by-agent.n.']);
    end
end