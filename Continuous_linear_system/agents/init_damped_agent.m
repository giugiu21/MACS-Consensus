function agent = init_damped_agent()
% initialize a mass-spring-damper linear agent

% physical parameters
m = 1.0;
c = 0.4; %damping parameter
k = 2.0;

% state-space matrices
A = [0, 1;
    -k/m, -c/m];

B = [0;
     1/m];

% output matrix
C = [1, 0]; % we use just the position as output, not the whole state, as required by the assignment

% direct transmission matrix
D = 0;

% controllability matrix
Co = ctrb(A, B);

% observability matrix
Ob = obsv(A, C);

% store agent data
agent.m = m;
agent.c = c;
agent.k = k;
agent.mass = m;
agent.damping = c;
agent.k_spring = k;
agent.A = A;
agent.B = B;
agent.C = C;
agent.D = D;
agent.n = size(A, 1);
agent.m_input = size(B, 2);
agent.input_dimension = agent.m_input;
agent.p_output = size(C, 1);
agent.is_controllable = rank(Co) == agent.n;
agent.is_observable = rank(Ob) == agent.n;
end
