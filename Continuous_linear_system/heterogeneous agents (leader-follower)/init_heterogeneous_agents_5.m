function agents = init_heterogeneous_agents_5()
% init_heterogeneous_agents_5 
%
% 
%
% State: xⁱ = [position; speed]
% Dynamics: ẋⁱ = Aⁱ xⁱ + Bⁱ uⁱ
%


params = [
%   k       b       m
    1.0,    0.5,    5.0;   
    2.0,    0.5,    15.0;   
    2.5,    1.5,    10.0;   
    3.0,    0.8,    8.0;    
    3.5,    1.5,    5.0;    
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
    C = [1, 0];   

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
