function info = analyze_consensus_convergence(agent, x0, N)
% infer the leaderless consensus target from the agent dynamics
%
% In leaderless consensus of identical LTI agents the control u = -(L kron K) x
% only kills the disagreement: the mean state evolves in open loop as
% d/dt xbar = A xbar, independent of K, topology and event-triggering (as long
% as the graph is undirected, 1'L = 0). Hence what the system converges TO is
% dictated by eig(A), not by eig(L):
%   Re(eig A) < 0  (damped)   -> xbar(t) -> 0, agents settle at the origin
%   Re(eig A) = 0  (undamped) -> xbar(t) oscillates at w0 = sqrt(k/m),
%                                agents synchronize on a common sinusoid whose
%                                amplitude/phase are set by xbar(0) = mean(x0)

n = agent.n;
A = agent.A;
eigA = eig(A);
tol = 1e-9;

% average initial condition: the invariant the consensus control never touches
x_bar0 = mean(reshape(x0, n, N), 2);

info.eigA = eigA;
info.x_bar0 = x_bar0;

if all(real(eigA) < -tol)
    info.regime = 'damped';
    info.target_value = zeros(n, 1);   % limit of e^{At} xbar0

elseif all(abs(real(eigA)) <= tol) && all(abs(imag(eigA)) > tol)
    info.regime = 'undamped';
    omega0 = abs(imag(eigA(1)));       % natural frequency sqrt(k/m)
    p0 = x_bar0(1);
    v0 = x_bar0(2);
    info.omega0 = omega0;
    info.amplitude = hypot(p0, v0 / omega0);
    info.phase = atan2(v0 / omega0, p0);

else
    info.regime = 'unstable';
end
end
