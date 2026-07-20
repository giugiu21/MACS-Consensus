% main_chung_kia.m ────────────────────────────────────────────────────────
%
% Leader-following eterogeneo per 5 agenti massa-molla-smorzatore
% Implementa il Theorem 4.1 di:
%
%   Yi-Fan Chung & Solmaz S. Kia (2020)
%   "Distributed leader following of an active leader for linear
%    heterogeneous multi-agent systems"
%   Systems & Control Letters 137, 104621
%
% ── GARANZIA DELL'ALGORITMO ───────────────────────────────────────────────
%   xⁱ(tₖ₊₁) = x⁰(tₖ)   ∀ follower i, ∀ epoca k
%   (i follower raggiungono lo stato campionato del leader entro la
%    successiva finestra di campionamento, con energia minima)

clear; clc; close all;

%% ─── 1. Agenti follower eterogenei ─────────────────────────────────────
agents = init_heterogeneous_agents_5();
N = length(agents);

fprintf('══════════════════════════════════════════════\n');
fprintf('  LEADER-FOLLOWER ETEROGENEO  (Chung & Kia 2020)\n');
fprintf('══════════════════════════════════════════════\n\n');
fprintf('Follower (%d agenti MSD eterogenei):\n', N);
for i = 1:N
    a = agents{i};
    if a.zeta < 1; tipo = 'sottosmorzato'; else; tipo = 'sovrasmorzato'; end
    fprintf('  F%d: k=%.1f  b=%.1f  m=%.1f  |  ωₙ=%.3f rad/s  ζ=%.3f  (%s)\n', ...
        i, a.k, a.b, a.m, a.omega_n, a.zeta, tipo);
end

%% ─── 2. Leader (MSD non-lineare, eq. 13 nel paper) ─────────────────────
% m⁰ẍ⁰ + b⁰ẋ⁰ + k⁰x⁰ + 0.6(x⁰)³ = u⁰(t)
% u⁰(t) è SCONOSCIUTO ai follower; essi vedono solo i campioni x⁰(tₖ).

leader.k    = 1.2;
leader.b    = 2.0;
leader.m    = 5.0;
leader.u_fn = @(t) 2 * sin(0.5 * t);   % ingresso leader (non-noto ai follower)

fprintf('\nLeader (non-lineare): k=%.1f  b=%.1f  m=%.1f  u(t)=2·sin(0.5t)\n', ...
    leader.k, leader.b, leader.m);

%% ─── 3. Grafo di interazione ────────────────────────────────────────────
% Grafo aciclico diretto (DAG), leader come global sink.
% F1 e F2 hanno accesso diretto al leader (N0_in).

graph.N0_in  = [1, 2];      % follower che vedono il leader

graph.A_adj  = zeros(N, N);
graph.A_adj(3, 1) = 1;      % F3 riceve da F1
graph.A_adj(3, 2) = 1;      % F3 riceve da F2
graph.A_adj(4, 2) = 1;      % F4 riceve da F2
graph.A_adj(5, 3) = 1;      % F5 riceve da F3
graph.A_adj(5, 4) = 1;      % F5 riceve da F4

fprintf('\nTopologia grafo (A_adj):\n');
disp(graph.A_adj);
fprintf('Follower con accesso diretto al leader: %s\n', ...
    num2str(graph.N0_in));

%% ─── 4. Parametri simulazione ───────────────────────────────────────────
T_s     = 1.0;      % periodo di campionamento [s] %1.0
dt      = 0.001;    % passo di integrazione [s]
T_total = 10.0;     % tempo totale [s]
time    = 0 : dt : T_total;

fprintf('\nParametri simulazione:\n');
fprintf('  T_s = %.2f s  |  dt = %.4f s  |  T_total = %.1f s\n', ...
    T_s, dt, T_total);

%% ─── 5. Condizioni iniziali ─────────────────────────────────────────────
% Come nell'Esempio 5.1 del paper: follower sparsi in posizione.

x0_init = [1.0; 0.5];    % leader: posizione=1 m, velocità=0.5 m/s

x_init  = zeros(2, N);
x_init(:, 1) = [ 0.0;  0.0];
x_init(:, 2) = [-0.5;  0.0];
x_init(:, 3) = [-1.0;  0.0];
x_init(:, 4) = [-1.5;  0.0];
x_init(:, 5) = [-2.0;  0.0];

%% ─── 6. Esegui simulazione ──────────────────────────────────────────────
fprintf('\nAvvio simulazione ...\n');
tic;
results = run_chung_kia(agents, leader, graph, T_s, x_init, x0_init, time, dt);
elapsed = toc;
fprintf('Simulazione completata in %.2f s.\n', elapsed);

%% ─── 7. Verifica: xⁱ(tₖ₊₁) ≈ x⁰(tₖ) ──────────────────────────────────
fprintf('\n── Verifica convergenza ─────────────────────────────────────────\n');
fprintf('  Epoca   x⁰(tₖ) [m]    Posizioni follower a tₖ₊₁ [m]       Err max\n');
fprintf('  ─────────────────────────────────────────────────────────────────\n');

for k = 0 : floor(T_total / T_s) - 1
    t_sample  = k * T_s;
    t_arrival = (k + 1) * T_s;
    idx_sam   = min(round(t_sample  / dt) + 1, length(time));
    idx_arr   = min(round(t_arrival / dt) + 1, length(time));

    ldr_pos = results.leader_pos(idx_sam);
    fol_pos = results.pos(:, idx_arr)';   % 1×N

    max_err = max(abs(fol_pos - ldr_pos));

    fol_str = sprintf('%.3f ', fol_pos);
    fprintf('   k=%d   %+7.4f      [ %s]    %.2e\n', ...
        k, ldr_pos, fol_str, max_err);
end

%% ─── 8. Grafici ─────────────────────────────────────────────────────────
col   = lines(N);
lbl_f = compose('Follower %d', 1:N);
t_pl  = results.time;
n_ep  = floor(T_total / T_s);

fig = figure('Name', 'Chung & Kia 2020 – Leader-Follower Eterogeneo', ...
             'Position', [50, 50, 1150, 820]);
tl  = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Leader-following eterogeneo – Theorem 4.1 (Chung & Kia, 2020)', ...
    'FontSize', 12, 'FontWeight', 'bold');

% ── Posizioni ────────────────────────────────────────────────────────────
ax1 = nexttile;
hold(ax1, 'on');

for i = 1:N
    plot(ax1, t_pl, results.pos(i, :), 'Color', col(i,:), 'LineWidth', 1.3);
end
plot(ax1, t_pl, results.leader_pos, 'k--', 'LineWidth', 2.0, ...
    'DisplayName', 'Leader x⁰(t)');

% Linee verticali agli istanti di campionamento
for k = 0:n_ep
    xline(ax1, k * T_s, ':', 'Color', [.65 .65 .65], 'LineWidth', 0.9);
end

% Marcatori: (+) stato campionato del leader, (×) arrivo dei follower
for k = 0 : n_ep - 1
    idx_s = min(round(k * T_s / dt) + 1, length(time));
    idx_a = min(round((k+1) * T_s / dt) + 1, length(time));

    plot(ax1, k * T_s, results.leader_pos(idx_s), ...
        'k+', 'MarkerSize', 11, 'LineWidth', 2.2);
    plot(ax1, (k+1) * T_s, results.pos(:, idx_a)', ...
        'gx', 'MarkerSize', 9, 'LineWidth', 2.0);
end

grid(ax1, 'on');
xlabel(ax1, 'tempo [s]');
ylabel(ax1, 'posizione [m]');
legend(ax1, [lbl_f, {'Leader x⁰(t)'}], 'Location', 'best', 'FontSize', 8);
title(ax1, ['Posizioni  —  (+) campione leader x⁰(tₖ)  |  ' ...
    '(×) arrivo follower a tₖ₊₁']);

% ── Velocità ─────────────────────────────────────────────────────────────
ax2 = nexttile;
hold(ax2, 'on');
for i = 1:N
    plot(ax2, t_pl, results.vel(i, :), 'Color', col(i,:), 'LineWidth', 1.3);
end
plot(ax2, t_pl, results.leader_vel, 'k--', 'LineWidth', 2.0);
for k = 0:n_ep
    xline(ax2, k * T_s, ':', 'Color', [.65 .65 .65], 'LineWidth', 0.9);
end
grid(ax2, 'on');
xlabel(ax2, 'tempo [s]');
ylabel(ax2, 'velocità [m/s]');
title(ax2, 'Velocità');

% ── Ingressi di controllo ────────────────────────────────────────────────
ax3 = nexttile;
hold(ax3, 'on');
for i = 1:N
    plot(ax3, t_pl, results.u_hist(i, :), 'Color', col(i,:), 'LineWidth', 1.1);
end
for k = 0:n_ep
    xline(ax3, k * T_s, ':', 'Color', [.65 .65 .65], 'LineWidth', 0.9);
end
grid(ax3, 'on');
xlabel(ax3, 'tempo [s]');
ylabel(ax3, 'forza [N]');
legend(ax3, lbl_f, 'Location', 'best', 'FontSize', 8);
title(ax3, 'Ingressi di controllo (minimum-energy)');

fprintf('\nCompletato. La figura mostra le traiettorie di posizione, velocità\n');
fprintf('e ingresso. Ogni follower deve raggiungere x⁰(tₖ) entro tₖ₊₁.\n');


results_dir = fullfile(fileparts(mfilename('fullpath')), 'results');
plot_chung_kia_results(results, agents, graph, T_s, time, results_dir);

% Animazione file: animate_chung_kia_v2
animate_chung_kia(time, results, agents, T_s);

% Oppure salvando il video:
% animate_chung_kia_v2(time, results, agents, graph, T_s, 'leader_follower.mp4');