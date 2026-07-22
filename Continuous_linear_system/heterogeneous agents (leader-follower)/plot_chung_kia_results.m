function plot_chung_kia_results(results, agents, graph, T_s, time, results_dir)
% plot_chung_kia_results 

if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

N       = length(agents);
n_steps = length(time);
n_ep    = floor(time(end) / T_s);

plot_end_time = time(end);
plot_mask     = time <= plot_end_time;

agent_names = compose('agent %d', 1:N);

% err(i, k) = |x^i(t_{k+1}) − x^0(t_k)|   (error position)
epoch_errors = zeros(N, n_ep);

for k = 0 : n_ep - 1
    idx_sam = min(round(k       * T_s / (time(2)-time(1))) + 1, n_steps);
    idx_arr = min(round((k+1)   * T_s / (time(2)-time(1))) + 1, n_steps);

    ldr_pos_k = results.leader_pos(idx_sam);
    for i = 1:N
        epoch_errors(i, k+1) = abs(results.pos(i, idx_arr) - ldr_pos_k);
    end
end

epoch_indices = 0 : n_ep - 1;  

% fig.1

fig1 = figure( ...
    'Name', 'open-loop vs CK leader-follower', ...
    'Position', [100, 100, 1000, 650]);

layout1 = tiledlayout(2, 1);
layout1.TileSpacing = 'compact';
layout1.Padding     = 'compact';


nexttile;


x_init_ol = squeeze(results.x_hist(:, :, 1));
y_open = zeros(N, n_steps);

for i = 1:N
    Ai  = agents{i}.A;
    xi  = x_init_ol(:, i);
    for s = 1:n_steps
        t_s = time(s);
        y_open(i, s) = agents{i}.C * expm(Ai * t_s) * xi;
    end
end

plot( ...
    time(plot_mask), ...
    y_open(:, plot_mask)', ...
    'LineWidth', 1.0);

grid on;
xlabel('time [s]');
ylabel('position');
title('Open-loop: nessun controllo, nessuna comunicazione');
xlim([0, plot_end_time]);


nexttile;

plot( ...
    time(plot_mask), ...
    results.pos(:, plot_mask)', ...
    'LineWidth', 1.0);

hold on;

plot( ...
    time(plot_mask), ...
    results.leader_pos(plot_mask), ...
    'k--', 'LineWidth', 1.5);


for k = 0:n_ep
    xline(k * T_s, ':', 'Color', [.65 .65 .65], 'LineWidth', 0.8);
end


for k = 0 : n_ep - 1
    dt_val = time(2) - time(1);
    idx_s  = min(round(k     * T_s / dt_val) + 1, n_steps);
    idx_a  = min(round((k+1) * T_s / dt_val) + 1, n_steps);
    plot(k     * T_s, results.leader_pos(idx_s), 'k+', ...
        'MarkerSize', 10, 'LineWidth', 2.0);
    plot((k+1) * T_s, results.pos(:, idx_a)', 'gx', ...
        'MarkerSize', 8,  'LineWidth', 1.8);
end

grid on;
xlabel('time [s]');
ylabel('position');
title(sprintf('CK leader-follower — campionamento periodico T_s=%.1f s', T_s));
xlim([0, plot_end_time]);
legend([agent_names, {'leader x⁰(t)'}], 'Location', 'best');

title(layout1, 'Open-loop vs CK leader-follower');

saveas(fig1, fullfile(results_dir, 'ck_openloop_vs_leaderfollower.png'));
saveas(fig1, fullfile(results_dir, 'ck_openloop_vs_leaderfollower.fig'));

% fig.2

fig2 = figure( ...
    'Name', 'CK 2x2 comparison', ...
    'Position', [100, 100, 1200, 780]);

layout2 = tiledlayout(2, 2);
layout2.TileSpacing = 'compact';
layout2.Padding     = 'compact';

nexttile;

plot(time(plot_mask), results.pos(:, plot_mask)', 'LineWidth', 1.0);
hold on;
plot(time(plot_mask), results.leader_pos(plot_mask), 'k--', 'LineWidth', 1.5);
for k = 0:n_ep
    xline(k * T_s, ':', 'Color', [.65 .65 .65], 'LineWidth', 0.8);
end
grid on;
xlabel('time [s]');
ylabel('position');
title('posizioni — CK leader-follower');
xlim([0, plot_end_time]);
legend([agent_names, {'leader'}], 'Location', 'best');


nexttile;

plot(time(plot_mask), results.vel(:, plot_mask)', 'LineWidth', 1.0);
hold on;
plot(time(plot_mask), results.leader_vel(plot_mask), 'k--', 'LineWidth', 1.5);
for k = 0:n_ep
    xline(k * T_s, ':', 'Color', [.65 .65 .65], 'LineWidth', 0.8);
end
grid on;
xlabel('time [s]');
ylabel('velocity [m/s]');
title('velocità');
xlim([0, plot_end_time]);


nexttile;

plot(time(plot_mask), results.u_hist(:, plot_mask)', 'LineWidth', 1.0);
for k = 0:n_ep
    xline(k * T_s, ':', 'Color', [.65 .65 .65], 'LineWidth', 0.8);
end
grid on;
xlabel('time [s]');
ylabel('force [N]');
title('ingressi di controllo (minimum-energy)');
xlim([0, plot_end_time]);
legend(agent_names, 'Location', 'best');


nexttile;

semilogy(epoch_indices, epoch_errors', 'LineWidth', 1.2);
hold on;
semilogy(epoch_indices, max(epoch_errors)', 'k--', 'LineWidth', 1.5, ...
    'DisplayName', 'max su tutti gli agenti');
grid on;
xlabel('epoch k');
ylabel('|x^i(t_{k+1}) − x^0(t_k)|  [m]');
title('errore di tracking per epoca (scala log)');
legend([agent_names, {'max'}], 'Location', 'best');

title(layout2, ...
    sprintf('CK leader-follower eterogeneo — T_s=%.1f s', T_s));

saveas(fig2, fullfile(results_dir, 'ck_comparison_2x2.png'));
saveas(fig2, fullfile(results_dir, 'ck_comparison_2x2.fig'));

% fig.3

fig3 = figure( ...
    'Name', 'tracking error per epoch', ...
    'Position', [100, 100, 1100, 520]);

layout3 = tiledlayout(1, 2);
layout3.TileSpacing = 'compact';
layout3.Padding     = 'compact';


nexttile;

mean_err = mean(epoch_errors(:, 2:end), 2);   

bar(1:N, mean_err);
grid on;
xlabel('agent');
ylabel('mean |error| [m]');
title('errore medio per agente (epoche k≥1)');
xticks(1:N);


nexttile;

bar(epoch_indices, max(epoch_errors)', 0.6);
grid on;
xlabel('epoch k');
ylabel('max |error| su agenti [m]');
title(sprintf('max errore per epoca — total mean = %.2e m', ...
    mean(max(epoch_errors(:, 2:end)))));

title(layout3, 'Errore di tracking xⁱ(tₖ₊₁) − x⁰(tₖ)');

saveas(fig3, fullfile(results_dir, 'ck_tracking_error_bar.png'));
saveas(fig3, fullfile(results_dir, 'ck_tracking_error_bar.fig'));

% fig.4

selected_agents = graph.N0_in(1);  

fig4 = figure( ...
    'Name', 'tracking error vs zero threshold', ...
    'Position', [100, 100, 1100, 620]);

layout4 = tiledlayout(2, 1);
layout4.TileSpacing = 'compact';
layout4.Padding     = 'compact';


nexttile;


leader_sampled = zeros(1, n_steps);
dt_val = time(2) - time(1);
for s = 1:n_steps
    k_ep = floor(time(s) / T_s + 1e-10);
    idx_s = min(round(k_ep * T_s / dt_val) + 1, n_steps);
    leader_sampled(s) = results.leader_pos(idx_s);
end

tracking_err_F1 = abs(results.pos(1, :) - leader_sampled);

plot(time(plot_mask), tracking_err_F1(plot_mask), 'LineWidth', 1.1);
hold on;
plot(time(plot_mask), zeros(1, sum(plot_mask)), 'r--', 'LineWidth', 1.1);

for k = 0 : n_ep - 1
    idx_a = min(round((k+1) * T_s / dt_val) + 1, n_steps);
    plot((k+1)*T_s, tracking_err_F1(idx_a), 'ko', ...
        'MarkerSize', 5, 'MarkerFaceColor', 'k');
end

grid on;
xlabel('time [s]');
ylabel('|xⁱ(t) − x⁰(tₖ)| [m]');
title(sprintf('agent %d (direttamente connesso al leader)', selected_agents));
legend('errore corrente', 'target (zero)', 'arrivo a tₖ₊₁', ...
    'Location', 'best');
xlim([0, plot_end_time]);


nexttile;

tracking_err_F5 = abs(results.pos(N, :) - leader_sampled);

plot(time(plot_mask), tracking_err_F5(plot_mask), 'LineWidth', 1.1);
hold on;
plot(time(plot_mask), zeros(1, sum(plot_mask)), 'r--', 'LineWidth', 1.1);
for k = 0 : n_ep - 1
    idx_a = min(round((k+1) * T_s / dt_val) + 1, n_steps);
    plot((k+1)*T_s, tracking_err_F5(idx_a), 'ko', ...
        'MarkerSize', 5, 'MarkerFaceColor', 'k');
end

grid on;
xlabel('time [s]');
ylabel('|xⁱ(t) − x⁰(tₖ)| [m]');
title(sprintf('agent %d (due livelli nella gerarchia)', N));
legend('errore corrente', 'target (zero)', 'arrivo a tₖ₊₁', ...
    'Location', 'best');
xlim([0, plot_end_time]);

title(layout4, ...
    'Errore di tracking vs soglia — confronto gerarchia');

saveas(fig4, fullfile(results_dir, 'ck_error_vs_threshold.png'));
saveas(fig4, fullfile(results_dir, 'ck_error_vs_threshold.fig'));

fprintf('\nGrafici salvati in: %s\n', results_dir);
fprintf('  ck_openloop_vs_leaderfollower.{png,fig}\n');
fprintf('  ck_comparison_2x2.{png,fig}\n');
fprintf('  ck_tracking_error_bar.{png,fig}\n');
fprintf('  ck_error_vs_threshold.{png,fig}\n');

end
