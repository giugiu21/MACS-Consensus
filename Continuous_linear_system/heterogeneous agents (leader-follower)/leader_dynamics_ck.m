function dxl = leader_dynamics_ck(xl, leader, t)
% leader_dynamics_ck  –  Leader MSD non-lineare (eq. 13, Chung & Kia 2020)
%
%   m ẍ⁰ + b⁰ ẋ⁰ + k⁰ x⁰ + 0.6 (x⁰)³ = u⁰(t)
%
% Il leader è ATTIVO (ha un ingresso u⁰) e NON-LINEARE (termine cubico).
% La sua dinamica è sconosciuta ai follower; essi vedono solo il valore
% campionato x⁰(tₖ).
%
% INPUT
%   xl      – stato del leader [posizione; velocità]  (2×1)
%   leader  – struct con campi: .k, .b, .m, .u_fn(t)
%   t       – tempo corrente [s]
%
% OUTPUT
%   dxl     – derivata dello stato [velocità; accelerazione]  (2×1)

pos = xl(1);
vel = xl(2);
u0  = leader.u_fn(t);          % ingresso (sconosciuto ai follower)

acc = (u0 - leader.b * vel - leader.k * pos - 0.6 * pos^3) / leader.m;

dxl = [vel; acc];
end
