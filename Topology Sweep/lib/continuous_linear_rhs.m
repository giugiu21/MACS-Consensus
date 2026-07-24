function dx = continuous_linear_rhs(x, agent, graph, K)
% compute multi-agent linear dynamics using current states

N = graph.N;

A_global = kron(eye(N), agent.A);
B_global = kron(eye(N), agent.B);

u = compute_continuous_control( ...
    x, graph, K, agent.n, agent.m_input);

leader = get_leader_id(agent, N);

% equilibrium feedforward input applied ONLY to the leader: in a pure
% leader-follower scheme the followers know nothing about the reference
% and must reach it through the network.
if isfield(agent, 'u_desired') && ~isempty(agent.u_desired)
    u_desired = validate_input_vector(agent.u_desired, agent.m_input, ...
        'agent.u_desired');
    idx_u_leader = (leader - 1) * agent.m_input + 1 : ...
        leader * agent.m_input;
    u(idx_u_leader) = u(idx_u_leader) + u_desired;
end

if isfield(agent, 'K_reference') && isfield(agent, 'x_desired') && ...
        ~isempty(agent.K_reference) && ~isempty(agent.x_desired)
    K_reference = validate_reference_gain( ...
        agent.K_reference, agent.m_input, agent.n);
    x_desired = validate_state_vector(agent.x_desired, agent.n, ...
        'agent.x_desired');

    idx_leader = (leader - 1) * agent.n + 1 : leader * agent.n;
    x_leader = x(idx_leader);
    u_reference = -K_reference * (x_leader - x_desired);

    idx_u_leader = (leader - 1) * agent.m_input + 1 : ...
        leader * agent.m_input;
    u(idx_u_leader) = u(idx_u_leader) + u_reference;
end

% multi-agent dynamics
dx = A_global * x + B_global * u;
end


function leader = get_leader_id(agent, N)
% return the configured leader index, using agent 1 by default

if isfield(agent, 'leader')
    leader = agent.leader;
elseif isfield(agent, 'leader_id')
    leader = agent.leader_id;
else
    leader = 1;
end

if ~isscalar(leader) || leader ~= round(leader) || ...
        leader < 1 || leader > N
    error('continuous_linear_rhs:BadLeader', ...
        'leader must be an integer between 1 and graph.N.');
end
end


function vector = validate_state_vector(vector, n, field_name)
% validate an n-dimensional state vector

vector = vector(:);

if numel(vector) ~= n
    error('continuous_linear_rhs:BadStateVector', ...
        '%s must have agent.n elements.', field_name);
end
end


function vector = validate_input_vector(vector, m_input, field_name)
% validate an input vector

vector = vector(:);

if numel(vector) ~= m_input
    error('continuous_linear_rhs:BadInputVector', ...
        '%s must have agent.m_input elements.', field_name);
end
end


function K_reference = validate_reference_gain(K_reference, m_input, n)
% validate leader reference feedback gain

if ~isequal(size(K_reference), [m_input, n])
    error('continuous_linear_rhs:BadReferenceGain', ...
        'agent.K_reference must have size agent.m_input-by-agent.n.');
end
end
