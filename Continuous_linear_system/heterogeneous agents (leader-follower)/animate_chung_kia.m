function animate_chung_kia(results, agents, graph, T_s, time, varargin)
% animate_chung_kia  –  Animazione del consenso leader-follower (Chung & Kia 2020)
%
% Mostra in tempo reale:
%   - Posizioni degli agenti su asse orizzontale (staggered per livello)
%   - Frecce di comunicazione continua tra follower (grigie)
%   - Flash rosso agli istanti di campionamento leader→F1,F2
%   - Grafo di comunicazione animato nel pannello laterale
%   - Frecce di velocità istantanea per ogni agente
%
% USO:
%   animate_chung_kia(results, agents, graph, T_s, time)
%   animate_chung_kia(results, agents, graph, T_s, time, 'speed', 3)
%   animate_chung_kia(results, agents, graph, T_s, time, 'save_gif', true)
%
% OPZIONI:
%   'speed'    – moltiplicatore velocità animazione (default: 2)
%   'save_gif' – true per salvare GIF (default: false)

%% ── Parse opzioni ────────────────────────────────────────────────────────
p = inputParser;
addParameter(p, 'speed',    2,     @isnumeric);
addParameter(p, 'save_gif', false, @islogical);
parse(p, varargin{:});

speed    = p.Results.speed;
save_gif = p.Results.save_gif;

%% ── Setup base ───────────────────────────────────────────────────────────
N       = length(agents);
n_steps = length(time);
dt_val  = time(2) - time(1);

% Subsample per fluidità animazione
target_fps  = 30;
step_skip   = max(1, round(speed / (target_fps * dt_val)));
anim_idx    = 1 : step_skip : n_steps;

n_ep = floor(time(end) / T_s);

%% ── Palette colori ───────────────────────────────────────────────────────
bg_dark    = [0.10, 0.10, 0.16];
bg_panel   = [0.07, 0.07, 0.12];
col_grid   = [0.25, 0.25, 0.30];
col_leader = [0.92, 0.20, 0.20];
col_text   = [0.92, 0.92, 0.95];
col_epoch  = [1.00, 0.85, 0.20];
col_err    = [0.50, 0.95, 0.60];

% Colori per livello gerarchico
col_hop1 = [0.25, 0.55, 0.95];   % F1, F2  – blu
col_hop2 = [0.20, 0.78, 0.45];   % F3, F4  – verde
col_hop3 = [0.72, 0.30, 0.90];   % F5      – viola

agent_col = [col_hop1; col_hop1; col_hop2; col_hop2; col_hop3];

%% ── Posizioni y per visualizzazione staggered ────────────────────────────
% (asse x = posizione fisica, asse y = livello agente)
y_lead = 6.5;
y_pos  = [5.0, 5.0, 3.5, 3.5, 2.0];   % F1..F5

% Range posizione
all_pos = [results.pos(:); results.leader_pos(:)];
x_min   = min(all_pos) - 0.5;
x_max   = max(all_pos) + 0.5;

%% ── Crea figura ──────────────────────────────────────────────────────────
fig = figure('Name', 'Chung & Kia – Animazione', ...
    'Position', [40, 40, 1150, 580], ...
    'Color', bg_dark, ...
    'NumberTitle', 'off');

tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'tight', 'Padding', 'compact');
%tl.BackgroundColor = bg_dark;

%% ════════════════════════════════════════════════════════════════════════
%  PANNELLO PRINCIPALE  (2/3 dello spazio)
%% ════════════════════════════════════════════════════════════════════════
ax = nexttile(tl, 1, [1, 2]);
ax.Color            = bg_panel;
ax.XColor           = col_text;
ax.YColor           = col_text;
ax.GridColor        = col_grid;
ax.GridAlpha        = 0.5;
ax.MinorGridAlpha   = 0.2;
ax.FontSize         = 10;
hold(ax, 'on');
grid(ax, 'on');

xlim(ax, [x_min, x_max]);
ylim(ax, [1.0, 7.5]);
xlabel(ax, 'posizione [m]', 'Color', col_text, 'FontSize', 11);

% Etichette asse y — valori unici
yticks(ax, [y_pos(5), mean([y_pos(4), y_pos(3)]), y_pos(2), y_lead]);
yticklabels(ax, {'F5  (hop 3)', 'F3-F4  (hop 2)', 'F1-F2  (N⁰ᵢₙ)', 'Leader'});

% Titolo dinamico
h_title = title(ax, 't = 0.000 s  |  epoca k = 0', ...
    'Color', col_text, 'FontSize', 12, 'FontWeight', 'bold');

%% ── Tracce storiche (faint) ──────────────────────────────────────────────
for i = 1:N
    c = agent_col(i,:);
    plot(ax, results.pos(i,:), y_pos(i) * ones(1,n_steps), ...
        '-', 'Color', [c, 0.12], 'LineWidth', 1.2);
end
plot(ax, results.leader_pos, y_lead * ones(1,n_steps), ...
    '-', 'Color', [col_leader, 0.12], 'LineWidth', 1.2);

%% ── Linee orizzontali di riferimento per livello ────────────────────────
for i = 1:N
    plot(ax, [x_min, x_max], [y_pos(i), y_pos(i)], ...
        '-', 'Color', [agent_col(i,:), 0.18], 'LineWidth', 0.6);
end
plot(ax, [x_min, x_max], [y_lead, y_lead], ...
    '-', 'Color', [col_leader, 0.18], 'LineWidth', 0.6);

%% ── Marcatori campioni leader (+) ────────────────────────────────────────
for k = 0:n_ep-1
    idx_s = min(round(k * T_s / dt_val) + 1, n_steps);
    plot(ax, results.leader_pos(idx_s), y_lead, 'w+', ...
        'MarkerSize', 12, 'LineWidth', 2.0);
end

%% ── Linee di comunicazione continua (follower→follower) ─────────────────
% Aggiornate ad ogni frame con le posizioni correnti
% Struttura bordi: A_adj(i,j)=1 → i riceve da j
edges = [];   % [j_sender, i_receiver] secondo la convenzione del paper
for i = 1:N
    for j = 1:N
        if graph.A_adj(i,j)
            edges(end+1,:) = [j, i]; %#ok<AGROW>
        end
    end
end
n_edges = size(edges, 1);

h_comm = gobjects(n_edges, 1);
for e = 1:n_edges
    j_s = edges(e,1);   % sender
    i_r = edges(e,2);   % receiver
    h_comm(e) = plot(ax, ...
        [results.pos(j_s,1), results.pos(i_r,1)], ...
        [y_pos(j_s), y_pos(i_r)], ...
        '-', 'Color', [0.75, 0.75, 0.75, 0.35], 'LineWidth', 1.3);
end

%% ── Linee di campionamento leader→F1, leader→F2 (flash) ─────────────────
h_flash1 = plot(ax, [results.leader_pos(1), results.pos(1,1)], ...
    [y_lead, y_pos(1)], '--', 'Color', [col_leader, 0.0], 'LineWidth', 2.5);
h_flash2 = plot(ax, [results.leader_pos(1), results.pos(2,1)], ...
    [y_lead, y_pos(2)], '--', 'Color', [col_leader, 0.0], 'LineWidth', 2.5);

%% ── Dot agenti (animati) ─────────────────────────────────────────────────
h_dot  = gobjects(N, 1);
h_vel  = gobjects(N, 1);
h_lbl  = gobjects(N, 1);

for i = 1:N
    h_dot(i) = plot(ax, results.pos(i,1), y_pos(i), 'o', ...
        'MarkerSize', 20, ...
        'MarkerFaceColor', agent_col(i,:), ...
        'MarkerEdgeColor', 'white', ...
        'LineWidth', 1.8);
    % Etichetta dentro il cerchio
    h_lbl(i) = text(ax, results.pos(i,1), y_pos(i), sprintf('F%d',i), ...
        'Color', 'white', 'FontSize', 8, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    % Freccia velocità
    h_vel(i) = quiver(ax, results.pos(i,1), y_pos(i)+0.38, ...
        0, 0, 0, ...
        'Color', agent_col(i,:), 'LineWidth', 1.8, ...
        'MaxHeadSize', 1.5, 'AutoScale', 'off');
end

% Dot leader
h_lead_dot = plot(ax, results.leader_pos(1), y_lead, 'pentagram', ...
    'MarkerSize', 24, ...
    'MarkerFaceColor', col_leader, ...
    'MarkerEdgeColor', 'white', 'LineWidth', 1.8);
h_lead_lbl = text(ax, results.leader_pos(1), y_lead, 'L', ...
    'Color', 'white', 'FontSize', 9, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
h_lead_vel = quiver(ax, results.leader_pos(1), y_lead+0.42, ...
    0, 0, 0, ...
    'Color', col_leader, 'LineWidth', 1.8, ...
    'MaxHeadSize', 1.5, 'AutoScale', 'off');

%% ════════════════════════════════════════════════════════════════════════
%  PANNELLO GRAFO  (1/3 dello spazio)
%% ════════════════════════════════════════════════════════════════════════
ax_g = nexttile(tl, 3);
ax_g.Color = bg_panel;
hold(ax_g, 'on');
axis(ax_g, 'off');
xlim(ax_g, [-0.3, 1.3]);
ylim(ax_g, [-0.45, 1.35]);

title(ax_g, 'Grafo di comunicazione', ...
    'Color', col_text, 'FontSize', 11, 'FontWeight', 'bold');

% Posizioni nodi nel pannello grafo
gnx = [0.15, 0.85, 0.15, 0.85, 0.50];   % F1..F5
gny = [0.90, 0.90, 0.55, 0.55, 0.18];
gnx_l = 0.50;  gny_l = 1.22;            % Leader

% Spigoli follower→follower (statici, grigi)
for e = 1:n_edges
    j_s = edges(e,1);
    i_r = edges(e,2);
    plot(ax_g, [gnx(j_s), gnx(i_r)], [gny(j_s), gny(i_r)], ...
        '-', 'Color', [0.7, 0.7, 0.7, 0.45], 'LineWidth', 1.5);
end

% Spigoli leader→F1, F2 (flash)
g_fl1 = plot(ax_g, [gnx_l, gnx(1)], [gny_l, gny(1)], ...
    '--', 'Color', [col_leader, 0.15], 'LineWidth', 2.2);
g_fl2 = plot(ax_g, [gnx_l, gnx(2)], [gny_l, gny(2)], ...
    '--', 'Color', [col_leader, 0.15], 'LineWidth', 2.2);

% Nodi follower
for i = 1:N
    plot(ax_g, gnx(i), gny(i), 'o', ...
        'MarkerSize', 26, ...
        'MarkerFaceColor', agent_col(i,:), ...
        'MarkerEdgeColor', 'white', 'LineWidth', 1.5);
    text(ax_g, gnx(i), gny(i), sprintf('F%d',i), ...
        'Color', 'white', 'FontSize', 10, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
end

% Nodo leader
plot(ax_g, gnx_l, gny_l, 'pentagram', ...
    'MarkerSize', 28, ...
    'MarkerFaceColor', col_leader, ...
    'MarkerEdgeColor', 'white', 'LineWidth', 1.8);
text(ax_g, gnx_l, gny_l, 'L', ...
    'Color', 'white', 'FontSize', 10, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');

% Legenda colori
leg_y = -0.08;
text(ax_g, 0.5, leg_y, 'N⁰ᵢₙ = {F1, F2}  (campionamento periodico)', ...
    'Color', [0.7, 0.85, 1.0], 'FontSize', 8, ...
    'HorizontalAlignment', 'center');

% Testi dinamici
h_epoch = text(ax_g, 0.5, -0.20, 'epoca k = 0', ...
    'Color', col_epoch, 'FontSize', 12, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center');

h_err = text(ax_g, 0.5, -0.32, 'max |err| = —', ...
    'Color', col_err, 'FontSize', 10, ...
    'HorizontalAlignment', 'center');

h_time_g = text(ax_g, 0.5, -0.42, 't = 0.000 s', ...
    'Color', [0.7, 0.7, 0.7], 'FontSize', 9, ...
    'HorizontalAlignment', 'center');

%% ── Legenda comunicazione ────────────────────────────────────────────────
text(ax_g, -0.25, 0.42, {'── comunicazione', '     continua xʲ(t)'}, ...
    'Color', [0.75, 0.75, 0.75], 'FontSize', 8);
text(ax_g, -0.25, 0.25, {'-- campionamento', '     x⁰(tₖ) ogni T_s'}, ...
    'Color', col_leader, 'FontSize', 8);

%% ════════════════════════════════════════════════════════════════════════
%  LOOP ANIMAZIONE
%% ════════════════════════════════════════════════════════════════════════
vel_scale   = 0.25;    % scala frecce velocità [m / (m/s)]
flash_total = 12;      % frame di durata del flash
flash_cnt   = 0;
prev_epoch  = -1;

gif_data = {};

fprintf('Animazione avviata — chiudi la finestra per interrompere.\n');

for idx = 1:length(anim_idx)

    if ~isvalid(fig), break; end   % finestra chiusa dall'utente

    s    = anim_idx(idx);
    t    = time(s);
    k_ep = floor(t / T_s + 1e-10);

    % ── Rileva inizio nuova epoca (flash) ─────────────────────────────
    if k_ep ~= prev_epoch
        prev_epoch = k_ep;
        flash_cnt  = flash_total;
    end

    % ── Aggiorna posizioni follower ───────────────────────────────────
    for i = 1:N
        set(h_dot(i), 'XData', results.pos(i,s));
        set(h_lbl(i), 'Position', [results.pos(i,s), y_pos(i), 0]);

        % Freccia velocità (limitata per non uscire dal grafico)
        vx = max(-0.9, min(0.9, results.vel(i,s) * vel_scale));
        set(h_vel(i), ...
            'XData', results.pos(i,s), ...
            'YData', y_pos(i) + 0.38, ...
            'UData', vx, 'VData', 0);
    end

    % ── Aggiorna leader ───────────────────────────────────────────────
    set(h_lead_dot, 'XData', results.leader_pos(s));
    set(h_lead_lbl, 'Position', [results.leader_pos(s), y_lead, 0]);
    vx_l = max(-0.9, min(0.9, results.leader_vel(s) * vel_scale));
    set(h_lead_vel, ...
        'XData', results.leader_pos(s), ...
        'YData', y_lead + 0.42, ...
        'UData', vx_l, 'VData', 0);

    % ── Aggiorna linee comunicazione continua ─────────────────────────
    for e = 1:n_edges
        j_s = edges(e,1);
        i_r = edges(e,2);
        set(h_comm(e), ...
            'XData', [results.pos(j_s,s), results.pos(i_r,s)], ...
            'YData', [y_pos(j_s), y_pos(i_r)]);
    end

    % ── Flash campionamento ───────────────────────────────────────────
    if flash_cnt > 0
        alpha_f = flash_cnt / flash_total;

        set(h_flash1, ...
            'XData', [results.leader_pos(s), results.pos(1,s)], ...
            'YData', [y_lead, y_pos(1)], ...
            'Color',  [col_leader, alpha_f]);
        set(h_flash2, ...
            'XData', [results.leader_pos(s), results.pos(2,s)], ...
            'YData', [y_lead, y_pos(2)], ...
            'Color',  [col_leader, alpha_f]);
        set(g_fl1, 'Color', [col_leader, alpha_f]);
        set(g_fl2, 'Color', [col_leader, alpha_f]);
        flash_cnt = flash_cnt - 1;
    else
        set(h_flash1, 'Color', [col_leader, 0.0]);
        set(h_flash2, 'Color', [col_leader, 0.0]);
        set(g_fl1,    'Color', [col_leader, 0.10]);
        set(g_fl2,    'Color', [col_leader, 0.10]);
    end

    % ── Aggiorna testi ────────────────────────────────────────────────
    set(h_title,  'String', sprintf('t = %.3f s  |  epoca k = %d', t, k_ep));
    set(h_epoch,  'String', sprintf('epoca  k = %d', k_ep));
    set(h_time_g, 'String', sprintf('t = %.3f s', t));

    % Errore corrente rispetto all'ultimo campione
    idx_sam  = min(round(k_ep * T_s / dt_val) + 1, n_steps);
    ldr_tgt  = results.leader_pos(idx_sam);
    max_err  = max(abs(results.pos(:,s) - ldr_tgt));
    set(h_err, 'String', sprintf('max |err| = %.4f m', max_err));

    drawnow limitrate;

    % ── Cattura frame GIF ─────────────────────────────────────────────
    if save_gif
        frame = getframe(fig);
        gif_data{end+1} = frame2im(frame); %#ok<AGROW>
    end
end

% ── Salva GIF ────────────────────────────────────────────────────────────
if save_gif && ~isempty(gif_data)
    fname = 'chung_kia_animation.gif';
    fprintf('Salvataggio GIF in %s ...\n', fname);
    for idx = 1:length(gif_data)
        [imind, cm] = rgb2ind(gif_data{idx}, 128);
        if idx == 1
            imwrite(imind, cm, fname, 'gif', ...
                'Loopcount', inf, 'DelayTime', 1/target_fps);
        else
            imwrite(imind, cm, fname, 'gif', ...
                'WriteMode', 'append', 'DelayTime', 1/target_fps);
        end
    end
    fprintf('GIF salvata.\n');
end

fprintf('Animazione completata.\n');
end
