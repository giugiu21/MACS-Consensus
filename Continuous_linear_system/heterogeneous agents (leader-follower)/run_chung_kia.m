function results = run_chung_kia(agents, leader, graph, T_s, x_init, x0_init, time, dt)
% run_chung_kia  –  Theorem 4.1, Chung & Kia (2020)
%
% Algoritmo distribuito leader-following per follower LTI eterogenei
% con leader attivo a dinamica sconosciuta.
%
% Garantisce:  xⁱ(tₖ₊₁) = x⁰(tₖ)   per ogni follower i, ad ogni epoca k.
% L'ingresso di ogni follower è MINIMUM-ENERGY nell'intervallo [tₖ, tₖ₊₁].
%
% ─────────────────────────────────────────────────────────────────────────
% CONVENZIONE GRAFO (come nel paper):
%   A_adj(i,j) = 1  ↔  follower i RICEVE da follower j
%   N0_in           =  indici dei follower con accesso diretto al leader
%
% Il leader è il "global sink": la sua informazione raggiunge tutti
% i follower in modo diretto o indiretto.
% ─────────────────────────────────────────────────────────────────────────
%
% INPUT
%   agents   cell(N,1)  Ogni struct ha: .A (n×n), .B (n×1), .n
%   leader   struct     .k, .b, .m, .u_fn(t)   (MSD non-lineare)
%   graph    struct     .A_adj (N×N), .N0_in (vettore riga)
%   T_s      scalar     Periodo di campionamento / lunghezza epoca [s]
%   x_init   n×N        Condizioni iniziali follower
%   x0_init  n×1        Condizione iniziale leader
%   time     1×M        Vettore dei tempi
%   dt       scalar     Passo di integrazione [s]
%
% OUTPUT  results  struct
%   .pos        N×M   posizioni follower
%   .vel        N×M   velocità follower
%   .u_hist     N×M   ingressi di controllo
%   .leader_pos 1×M   posizione leader
%   .leader_vel 1×M   velocità leader
%   .x_hist     n×N×M stati completi follower
%   .x0_hist    n×M   stato leader

N  = length(agents);
n  = agents{1}.n;
M  = length(time);

% ── Storage ──────────────────────────────────────────────────────────────
x_hist  = zeros(n, N, M);
x0_hist = zeros(n, M);
u_hist  = zeros(N, M);

% ── Stati correnti ────────────────────────────────────────────────────────
x  = x_init;    % n×N  stati follower
xl = x0_init;   % n×1  stato leader

% ── Dati d'epoca (costanti per [tₖ, tₖ₊₁]) ──────────────────────────────
x_tk    = zeros(n, N);  % xⁱ(tₖ)
x0_tk   = zeros(n, 1);  % x⁰(tₖ)
Gk      = cell(N, 1);   % Gramiano completo  G^i_k = G^i(T_s)
Gk_inv  = cell(N, 1);   % (G^i_k)⁻¹
eATk    = cell(N, 1);   % e^{Aⁱ Tₛ}

% ── ODE di Lyapunov per Gʲₖ(t) = Wʲ(t) · Φʲ(t) ─────────────────────────
% Ẇʲ = Aʲ Wʲ + Wʲ Aʲᵀ + Bʲ Bʲᵀ,  Wʲ(tₖ) = 0
% Integrato con passo Euler, resettato ogni epoca.
W = cell(N, 1);
for j = 1:N
    W{j} = zeros(n, n);
end

% Deadzone: nelle prime 'deadzone' secondi di ogni epoca si salta il
% termine di correzione inter-campionamento (G^j_k(t) è numericamente
% singolare vicino a tₖ).  Vedi Remark 4.2 del paper.
deadzone = 15 * dt;

prev_epoch = -1;

% ── Loop principale ───────────────────────────────────────────────────────
for s = 1:M
    t    = time(s);
    k_ep = floor(t / T_s + 1e-10);   % indice epoca (offset per fp)
    t_k  = k_ep * T_s;
    t_k1 = (k_ep + 1) * T_s;
    t_el = t - t_k;                   % tempo trascorso nell'epoca
    t_rm = t_k1 - t;                  % tempo rimanente nell'epoca

    % ── Inizializzazione epoca ────────────────────────────────────────────
    if k_ep ~= prev_epoch
        prev_epoch = k_ep;
        x_tk  = x;
        x0_tk = xl;

        for i = 1:N
            Ai = agents{i}.A;
            Bi = agents{i}.B;

            % G^i_k = ∫₀^{Tₛ} e^{Aⁱτ} Bⁱ Bⁱᵀ e^{Aⁱᵀτ} dτ
            Gki       = compute_finite_gramian(Ai, Bi, T_s);
            Gk{i}     = Gki;
            Gk_inv{i} = Gki \ eye(n);    % (G^i_k)⁻¹

            % e^{Aⁱ Tₛ}  (usato più volte: precalcolato)
            eATk{i} = expm(Ai * T_s);

            % Reset ODE Lyapunov
            W{i} = zeros(n, n);
        end
    end

    % ── Salva storia ──────────────────────────────────────────────────────
    x_hist(:, :, s) = x;
    x0_hist(:, s)   = xl;

    % ── Legge di controllo – Theorem 4.1, eq. (9) ────────────────────────
    u = zeros(N, 1);

    t_rm_s = max(t_rm, 1e-9);    % tempo rimanente "sicuro"

    for i = 1:N
        Ai = agents{i}.A;
        Bi = agents{i}.B;

        % e^{Aⁱᵀ (tₖ₊₁ − t)}
        eAiT_rm    = expm(Ai' * t_rm_s);

        % e^{Aⁱ Tₛ} xⁱ(tₖ)
        eAiTk_xik  = eATk{i} * x_tk(:, i);

        % Indicatore connessione al leader e grado uscita
        I_i   = double(ismember(i, graph.N0_in));
        d_out = sum(graph.A_adj(i, :));
        denom = I_i + d_out;

        if denom == 0
            % Agente isolato (non dovrebbe succedere con grafo valido)
            u(i) = 0;
            continue;
        end

        w_l = I_i  / denom;
        w_f = 1.0  / denom;

        % ── Termine leader  (Lemma 4.1, primo addendo di (9)) ────────────
        % v_l = x⁰(tₖ) − F^{i0} − e^{Aⁱ Tₛ} xⁱ(tₖ)     [F^{i0}=0]
        v_l = zeros(n, 1);
        if I_i
            v_l = x0_tk - eAiTk_xik;
        end

        % ── Termini follower  (secondo addendo di (9)) ────────────────────
        %
        %   v_f = Σⱼ aᵢⱼ { [Gʲₖ (Gʲₖ(t))⁻¹ (xʲ(t) − e^{Aʲ tel} xʲ(tₖ))]
        %                  +  [e^{Aʲ Tₛ} xʲ(tₖ) − e^{Aⁱ Tₛ} xⁱ(tₖ)] }
        %
        %   termine-1: correzione inter-campionamento  (usa stato corrente xʲ(t))
        %   termine-2: accordo ai tempi di campionamento  (usa solo xʲ(tₖ))
        v_f = zeros(n, 1);

        for j = 1:N
            if graph.A_adj(i, j)    % a_{ij}=1: i riceve da j
                Aj = agents{j}.A;

                % e^{Aʲ Tₛ} xʲ(tₖ)
                eAjTk_xjk = eATk{j} * x_tk(:, j);

                % ── Termine-2: target mismatch ai tempi di campionamento ──
                %  [F^{ij} = 0 → nessuna formazione]
                t2 = eAjTk_xjk - eAiTk_xik;

                % ── Termine-1: correzione inter-campionamento ─────────────
                t1 = zeros(n, 1);

                if t_el > deadzone
                    % e^{Aʲ(t−tₖ)} xʲ(tₖ)  =  risposta libera di j
                    eAj_tel  = expm(Aj * t_el);
                    xj_free  = eAj_tel * x_tk(:, j);
                    xj_diff  = x(:, j) - xj_free;   % innovazione

                    % Gʲₖ(t) = Wʲ(t) · e^{Aʲᵀ(tₖ₊₁−t)}
                    Phi_j = expm(Aj' * t_rm_s);
                    Gjkt  = W{j} * Phi_j;

                    % Gʲₖ · (Gʲₖ(t))⁻¹ · xj_diff
                    % Nota: se xʲ(t) segue esattamente (10), questo
                    % termine è costante = x⁰(tₖ) − e^{Aʲ Tₛ} xʲ(tₖ).
                    % In pratica vale la stessa cosa con piccoli errori numerici.
                    if rcond(Gjkt) > 1e-12
                        t1 = Gk{j} * (Gjkt \ xj_diff);
                    end
                    % else: lascia t1 = 0 (Gjkt ancora mal condizionata)
                end

                v_f = v_f + t1 + t2;
            end
        end

        % ── Assembla ingresso: uⁱ = Bⁱᵀ e^{Aⁱᵀ(…)} (Gⁱₖ)⁻¹ (w_l v_l + w_f v_f)
        v    = w_l * v_l + w_f * v_f;
        u(i) = Bi' * eAiT_rm * (Gk_inv{i} * v);
    end

    u_hist(:, s) = u;

    % ── Integrazione RK4 – follower ───────────────────────────────────────
    for i = 1:N
        Ai = agents{i}.A;
        Bi = agents{i}.B;
        xi = x(:, i);
        ui = u(i);
        f  = @(z) Ai * z + Bi * ui;
        k1 = f(xi);
        k2 = f(xi + dt/2 * k1);
        k3 = f(xi + dt/2 * k2);
        k4 = f(xi + dt   * k3);
        x(:, i) = xi + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
    end

    % ── Integrazione RK4 – leader (non-lineare) ───────────────────────────
    f_l = @(z, s_t) leader_dynamics_ck(z, leader, s_t);
    k1 = f_l(xl,            t);
    k2 = f_l(xl + dt/2*k1,  t + dt/2);
    k3 = f_l(xl + dt/2*k2,  t + dt/2);
    k4 = f_l(xl + dt   *k3, t + dt);
    xl = xl + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);

    % ── Integrazione Euler – ODE Lyapunov per Wʲ ─────────────────────────
    % Ẇʲ = Aʲ Wʲ + Wʲ Aʲᵀ + Bʲ Bʲᵀ
    for j = 1:N
        Aj   = agents{j}.A;
        Bj   = agents{j}.B;
        dW   = Aj * W{j} + W{j} * Aj' + Bj * Bj';
        W{j} = W{j} + dt * dW;
    end
end

% ── Formatta output ───────────────────────────────────────────────────────
results.x_hist     = x_hist;
results.x0_hist    = x0_hist;
results.u_hist     = u_hist;
results.time       = time;
results.pos        = reshape(x_hist(1, :, :), N, M);   % N×M
results.vel        = reshape(x_hist(2, :, :), N, M);   % N×M
results.leader_pos = x0_hist(1, :);                     % 1×M
results.leader_vel = x0_hist(2, :);                     % 1×M
end
