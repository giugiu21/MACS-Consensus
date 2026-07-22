function dxl = leader_dynamics_ck(xl, leader, t)

%   m ẍ⁰ + b⁰ ẋ⁰ + k⁰ x⁰ + 0.6 (x⁰)³ = u⁰(t)
%
% The leader is ACTIVE (it has an input u⁰) and NONLINEAR (cubic term)
% Its dynamics are unknown to the followers, they only see the sampled value x⁰(tₖ).


pos = xl(1);
vel = xl(2);
u0  = leader.u_fn(t);          % input

acc = (u0 - leader.b * vel - leader.k * pos - 0.6 * pos^3) / leader.m;

dxl = [vel; acc];
end
