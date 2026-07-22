function animate_chung_kia(time, results, agents, T_s, video_filename)
if nargin < 6
    video_filename = "";
end

% data:
N        = length(agents);
n_steps  = length(time);
dt_val   = time(2) - time(1);
q_foll   = results.pos;          % N × n_steps
q_leader = results.leader_pos;   % 1 × n_steps

% x axis:
all_q    = [q_foll(:); q_leader(:)];
q_min    = min(all_q);
q_max    = max(all_q);
q_margin = 0.2 * max(1, q_max - q_min);

wall_x   = q_min - q_margin * 0.6;

% y axis:
y_foll   = 1:N;
y_lead   = N + 1.5;

y_labels = [compose('F%d', 1:N), {'Leader'}];
y_ticks  = [y_foll, y_lead];

% figure:
figure('Name', 'Chung & Kia – Leader-Follower Animation', ...
    'Position', [100, 100, 1050, 560]);

hold on;
grid on;

xlabel('posizione [m]');
ylabel('agente');
title('leader-follower eterogeneo — Chung & Kia (2020)');

xlim([wall_x, q_max + q_margin]);
ylim([0.5, y_lead + 0.8]);

set(gca, 'YTick', y_ticks, 'YTickLabel', y_labels);

col_hop1   = [0.20, 0.50, 0.90];   % blue
col_hop2   = [0.15, 0.72, 0.40];   % green
col_hop3   = [0.65, 0.25, 0.85];   % purple
col_leader = [0.90, 0.15, 0.15];   % leader - red

agent_col = [col_hop1; col_hop1; col_hop2; col_hop2; col_hop3];

% follower
spring_lines = gobjects(N, 1);
mass_markers = gobjects(N, 1);

for i = 1:N
    spring_lines(i) = plot( ...
        [wall_x, q_foll(i, 1)], [y_foll(i), y_foll(i)], ...
        '-', 'Color', agent_col(i,:), 'LineWidth', 1.2);

    mass_markers(i) = plot( ...
        q_foll(i, 1), y_foll(i), 's', ...
        'MarkerSize', 16, ...
        'MarkerFaceColor', agent_col(i,:), ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.0);
end

% leader
leader_spring = plot( ...
    [wall_x, q_leader(1)], [y_lead, y_lead], ...
    '-', 'Color', col_leader, 'LineWidth', 1.5);

leader_marker = plot( ...
    q_leader(1), y_lead, 'p', ...
    'MarkerSize', 20, ...
    'MarkerFaceColor', col_leader, ...
    'MarkerEdgeColor', 'k', ...
    'LineWidth', 1.2);


% vertical target line
target_line = xline(q_leader(1), '--r', ...
    sprintf('x^0(t_k) = %.3f', q_leader(1)), ...
    'LineWidth', 1.8, 'Color', col_leader, 'Alpha', 0.7);

% vertical mean line
mean_line = xline(mean(q_foll(:, 1)), ':', ...
    'media follower', ...
    'LineWidth', 1.2, 'Color', [0.4, 0.4, 0.4]);

yline(N + 0.75, '-', 'Color', [0.7, 0.7, 0.7], 'LineWidth', 0.8);

txt_epoch = text( ...
    wall_x + 0.05 * (q_max - wall_x), ...
    y_lead + 0.55, ...
    'k = 0', ...
    'FontSize', 10, 'Color', [0.8, 0.5, 0.0], 'FontWeight', 'bold');

txt_err = text( ...
    wall_x + 0.05 * (q_max - wall_x), ...
    y_lead + 0.25, ...
    'max |err| = —', ...
    'FontSize', 9, 'Color', [0.1, 0.6, 0.2]);

% legend
legend( ...
    [mass_markers; leader_marker; target_line; mean_line], ...
    [compose('F%d', 1:N), ...
    {'leader x^0(t)'}, ...
    {'target x^0(t_k)'}, ...
    {'media follower'}], ...
    'Location', 'southeast', ...
    'FontSize', 8);

% video
if strlength(string(video_filename)) > 0
    writer           = VideoWriter(video_filename, 'MPEG-4');
    writer.FrameRate = 30;
    open(writer);
else
    writer = [];
end

skip = max(1, floor(n_steps / 1500));

prev_epoch = -1;

% animation loop
for k = 1 : skip : n_steps

    t    = time(k);
    k_ep = floor(t / T_s + 1e-10);


    q    = q_foll(:, k);          % pos follower
    q_l  = q_leader(k);           % pos leader

    % update mass - spring (follower)
    for i = 1:N
        set(spring_lines(i), ...
            'XData', [wall_x, q(i)], ...
            'YData', [y_foll(i), y_foll(i)]);
        set(mass_markers(i), ...
            'XData', q(i), ...
            'YData', y_foll(i));
    end

    % update mass - spring (leader)
    set(leader_spring, 'XData', [wall_x, q_l], 'YData', [y_lead, y_lead]);
    set(leader_marker, 'XData', q_l, 'YData', y_lead);

    % update target
    if k_ep ~= prev_epoch
        prev_epoch = k_ep;
        idx_sam    = min(round(k_ep * T_s / dt_val) + 1, n_steps);
        q_target   = q_leader(idx_sam);

        target_line.Value = q_target;
        target_line.Label = sprintf('x^0(t_%d) = %.3f m', k_ep, q_target);
        
        target_line.LineWidth = 2.8;
        target_line.Alpha     = 1.0;
    else
        
        if target_line.LineWidth > 1.8
            target_line.LineWidth = target_line.LineWidth - 0.08;
            target_line.Alpha     = max(0.5, target_line.Alpha - 0.02);
        end
    end

    % update mean follower
    mean_line.Value = mean(q);

    
    idx_sam_cur = min(round(k_ep * T_s / dt_val) + 1, n_steps);
    q_tgt_cur   = q_leader(idx_sam_cur);
    max_err      = max(abs(q - q_tgt_cur));

    set(txt_epoch, 'String', sprintf('epoca  k = %d', k_ep));
    set(txt_err,   'String', sprintf('max |err| = %.4f m', max_err));

    title(sprintf( ...
        'leader-follower eterogeneo — Chung & Kia (2020) — t = %.3f s', t));

    drawnow;

    if ~isempty(writer)
        frame = getframe(gcf);
        writeVideo(writer, frame);
    end
end

if ~isempty(writer)
    close(writer);
    fprintf('Video salvato: %s\n', video_filename);
end

end
