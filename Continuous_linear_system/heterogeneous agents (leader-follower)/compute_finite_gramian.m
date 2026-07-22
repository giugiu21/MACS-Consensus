function G = compute_finite_gramian(A, B, T)
% compute_finite_gramian
%G(T) = ∫₀ᵀ e^{Aτ} B Bᵀ e^{Aᵀτ} dτ

% Lyapunov resolution: Ẇ = A W + W Aᵀ + B Bᵀ ,  W(0) = 0

% output: Gramiano (Gn×n), symmetric positive definite

n   = size(A, 1);
BBT = B * B';

rhs = @(~, w) reshape(A * reshape(w, n, n) + ...
                       reshape(w, n, n) * A' + BBT, [], 1);

opts    = odeset('RelTol', 1e-9, 'AbsTol', 1e-11);
[~, Wh] = ode45(rhs, [0, T], zeros(n^2, 1), opts);

G = reshape(Wh(end, :)', n, n);
G = (G + G') / 2;      
end
