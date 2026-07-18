function agent = init_undamped_agent()
% Initialize an undamped mass-spring oscillator agent (damping c = 0).
%
% Same structure as init_mass_spring_damper_agent.m, kept as a separate
% experiment-specific initializer so that the original file stays untouched.
%
% With c = 0 the state matrix A has purely imaginary eigenvalues
% (+/- j*sqrt(k/m)): no agent settles by itself, so any output agreement
% is due to the network coupling alone. This makes the consensus
% objective structurally non-trivial (see docs/event_triggered_consensus_proof.pdf,
% Section 7). Note that the theory only requires the disagreement modes
% A - lambda_i*B*K to be Hurwitz, not A itself; this is verified at run
% time by check_consensus_modes.m.

% physical parameters
m = 1.0;
c = 0.0;   % no damping
k = 2.0;

% state-space matrices
A = [0, 1;
    -k/m, -c/m];

B = [0;
     1/m];

% output matrix (position only, as required by the assignment)
C = [1, 0];

% direct transmission matrix
D = 0;

% controllability and observability matrices
Co = ctrb(A, B);
Ob = obsv(A, C);

% store agent data (same fields as init_mass_spring_damper_agent)
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
