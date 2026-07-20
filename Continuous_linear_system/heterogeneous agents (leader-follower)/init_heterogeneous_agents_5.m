function agents = init_heterogeneous_agents_5()
% init_heterogeneous_agents_5  –  5 agenti MSD eterogenei sottosmorzati
%
% Parametri adattati dalla Tabella 1 di Chung & Kia (2020),
% "Distributed leader following of an active leader for linear
%  heterogeneous multi-agent systems", Sys. & Control Letters 137.
%
% Tutti i follower sono SOTTOSMORZATI (ζ < 1): senza controllo oscillano
% attorno all'equilibrio con ampiezza decrescente.
%
% Stato: xⁱ = [posizione; velocità]
% Dinamica: ẋⁱ = Aⁱ xⁱ + Bⁱ uⁱ
%
%   Aⁱ = [  0        1   ]      Bⁱ = [   0   ]
%        [ -kⁱ/mⁱ  -bⁱ/mⁱ ]         [ 1/mⁱ ]

params = [
%   k       b       m
    1.0,    0.5,    5.0;    % follower 1  ωₙ=0.447, ζ=0.112
    2.0,    0.5,    15.0;   % follower 2  ωₙ=0.365, ζ=0.046
    2.5,    1.5,    10.0;   % follower 3  ωₙ=0.500, ζ=0.212
    3.0,    0.8,    8.0;    % follower 4  ωₙ=0.612, ζ=0.092
    3.5,    1.5,    5.0;    % follower 5  ωₙ=0.837, ζ=0.260
];

N      = size(params, 1);
agents = cell(N, 1);

for i = 1:N
    k = params(i, 1);
    b = params(i, 2);
    m = params(i, 3);

    A = [0,    1;
        -k/m, -b/m];
    B = [0;
         1/m];
    C = [1, 0];   % output: sola posizione

    agents{i}.k        = k;
    agents{i}.b        = b;
    agents{i}.m        = m;
    agents{i}.A        = A;
    agents{i}.B        = B;
    agents{i}.C        = C;
    agents{i}.n        = 2;
    agents{i}.m_input  = 1;
    agents{i}.omega_n  = sqrt(k / m);
    agents{i}.zeta     = b / (2 * sqrt(k * m));
    % agents{i}.is_controllable = (rank(ctrb(A, B)) == 2);
    Co = [B, A*B];
    agents{i}.is_controllable = (rank(Co) == 2);
end
end
