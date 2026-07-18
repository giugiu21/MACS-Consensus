function dx = continuous_linear_rhs( ...
    x, agent, graph, K_consensus, opts, current_time)
%CONTINUOUS_LINEAR_RHS
% Compute continuous-communication multi-agent dynamics.
%
% Supported control cases:
%
%   opts.control_case = "leaderless"
%   opts.control_case = "leader_follower"
%
% Supported systems:
%
%   opts.system = "no-equilibrium"
%       The leader tracks an external desired state.
%
%   opts.system = "equilibrium"
%       The leader is an autonomous oscillator:
%
%           x_dot_L = A*x_L
%           u_L = 0
%
%       Followers synchronize with the natural leader trajectory.

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
            'continuous_linear_rhs:InvalidControlCase', ...
            ['opts.control_case must be "leaderless" ', ...
             'or "leader_follower".']);
    end

    if ~ismember(system_type, valid_system_types)
        error( ...
            'continuous_linear_rhs:InvalidSystem', ...
            'opts.system must be "equilibrium" or "no-equilibrium".');
    end

    %% Control architecture

    switch control_case

        case "leaderless"

        %% Standard continuous diffusive consensus

        u = -kron(graph.L, K_consensus) * x;

        %% Optional common-reference tracking
        %
        % For opts.system = "no-equilibrium", every agent knows the
        % desired trajectory and applies the same tracking controller.
        %
        % u_i = u_i,consensus ...
        %       - K_reference * (x_i - x_desired) ...
        %       + u_desired

        if system_type == "no-equilibrium"

            %% Validate reference gain

            if ~isfield(opts, 'K_reference') || ...
                    isempty(opts.K_reference)

                error( ...
                    'continuous_linear_rhs:MissingReferenceGain', ...
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
                    'continuous_linear_rhs:MissingReference', ...
                    ['opts.x_desired_fun is required for ', ...
                    'no-equilibrium leaderless tracking.']);
            end

            x_desired = opts.x_desired_fun(current_time);

            x_desired = validate_state_vector( ...
                x_desired, ...
                n, ...
                'opts.x_desired_fun');

            %% Equilibrium/feedforward input

            if isfield(opts, 'u_desired') && ...
                    ~isempty(opts.u_desired)

                u_feedforward = validate_input_vector( ...
                    opts.u_desired, ...
                    m_input, ...
                    'opts.u_desired');

            else

                u_feedforward = zeros(m_input, 1);
            end

            %% Apply reference feedback to every agent

            for agent_id = 1:N

                idx_agent = ...
                    (agent_id - 1) * n + 1 : ...
                    agent_id * n;

                idx_u_agent = ...
                    (agent_id - 1) * m_input + 1 : ...
                    agent_id * m_input;

                x_agent = x(idx_agent);

                u_reference_i = ...
                    -K_reference * (x_agent - x_desired);

                u(idx_u_agent) = ...
                    u(idx_u_agent) ...
                    + u_reference_i ...
                    + u_feedforward;
            end
        end

        case "leader_follower"

            leader_id = validate_leader_id(opts, N);

            switch system_type

                case "no-equilibrium"

                    %% Damped leader-follower
                    %
                    % Preserve the original behavior:
                    %
                    % - all agents use diffusive consensus;
                    % - all agents may receive equilibrium feedforward;
                    % - only the leader receives reference feedback.

                    u = -kron(graph.L, K_consensus) * x;

                    %% Validate reference feedback gain

                    if ~isfield(opts, 'K_reference') || ...
                            isempty(opts.K_reference)

                        error( ...
                            'continuous_linear_rhs:MissingReferenceGain', ...
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
                            'continuous_linear_rhs:MissingReference', ...
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

                    %% Real leader state

                    idx_leader = ...
                        (leader_id - 1) * n + 1 : ...
                        leader_id * n;

                    x_leader = x(idx_leader);

                    %% Leader reference feedback

                    e_reference = x_leader - x_desired;

                    u_reference = ...
                        -K_reference * e_reference;

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

                    % Every damped physical agent receives the input
                    % required to maintain the desired equilibrium.
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
                    % The leader is autonomous:
                    %
                    %       u_L = 0
                    %       x_dot_L = A*x_L
                    %
                    % The followers use diffusive coupling to synchronize
                    % with the leader trajectory.

                    L_control = graph.L;

                    % The leader does not react to any neighbour.
                    L_control(leader_id, :) = 0;

                    % Followers use the current real states.
                    u = -kron(L_control, K_consensus) * x;

                    %% Explicitly impose zero leader input

                    idx_u_leader = ...
                        (leader_id - 1) * m_input + 1 : ...
                        leader_id * m_input;

                    u(idx_u_leader) = zeros(m_input, 1);
            end
    end

    %% Multi-agent physical dynamics

    dx = A_global * x + B_global * u;
end


function leader_id = validate_leader_id(opts, N)
%VALIDATE_LEADER_ID Validate and return the leader identifier.

    if ~isfield(opts, 'leader_id') || isempty(opts.leader_id)

        error( ...
            'continuous_linear_rhs:MissingLeader', ...
            'opts.leader_id is required in leader-follower mode.');
    end

    leader_id = opts.leader_id;

    if ~isscalar(leader_id) || ...
            leader_id ~= round(leader_id) || ...
            leader_id < 1 || ...
            leader_id > N

        error( ...
            'continuous_linear_rhs:BadLeader', ...
            'opts.leader_id must be an integer between 1 and graph.N.');
    end
end


function vector = validate_state_vector(vector, n, field_name)
%VALIDATE_STATE_VECTOR Validate an n-dimensional state vector.

    vector = vector(:);

    if numel(vector) ~= n

        error( ...
            'continuous_linear_rhs:BadStateVector', ...
            '%s must return agent.n elements.', ...
            field_name);
    end
end


function vector = validate_input_vector(vector, m_input, field_name)
%VALIDATE_INPUT_VECTOR Validate an input vector.

    vector = vector(:);

    if numel(vector) ~= m_input

        error( ...
            'continuous_linear_rhs:BadInputVector', ...
            '%s must contain agent.m_input elements.', ...
            field_name);
    end
end


function K_reference = validate_reference_gain( ...
    K_reference, m_input, n)
%VALIDATE_REFERENCE_GAIN Validate the leader reference feedback gain.

    if ~isequal(size(K_reference), [m_input, n])

        error( ...
            'continuous_linear_rhs:BadReferenceGain', ...
            ['opts.K_reference must have size ', ...
             'agent.m_input-by-agent.n.']);
    end
end