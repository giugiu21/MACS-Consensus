function G = compute_finite_gramian(A, B, T)
% compute_finite_gramian  –  Gramiano di controllabilità a orizzonte finito
%
%   G(T) = ∫₀ᵀ e^{Aτ} B Bᵀ e^{Aᵀτ} dτ
%
% Risolto via l'ODE di Lyapunov:
%   Ẇ = A W + W Aᵀ + B Bᵀ ,  W(0) = 0
%
% Il sistema (A,B) è supposto controllabile → G(T) è definita positiva
% per ogni T > 0.
%
% INPUT
%   A  – matrice di stato (n×n)
%   B  – matrice di ingresso (n×m)
%   T  – orizzonte temporale [s]  (scalare positivo)
%
% OUTPUT
%   G  – Gramiano (n×n), simmetrico definito positivo

n   = size(A, 1);
BBT = B * B';

% Funzione destra dell'ODE vettorializzata
rhs = @(~, w) reshape(A * reshape(w, n, n) + ...
                       reshape(w, n, n) * A' + BBT, [], 1);

opts    = odeset('RelTol', 1e-9, 'AbsTol', 1e-11);
[~, Wh] = ode45(rhs, [0, T], zeros(n^2, 1), opts);

G = reshape(Wh(end, :)', n, n);
G = (G + G') / 2;       % forza simmetria (evita deriva numerica)
end
