function plot_topology_sweep(summary_table, plant_label, results_dir)
%PLOT_TOPOLOGY_SWEEP Four-panel comparison of the topology/size sweep.
%
% Panel 1 : algebraic connectivity lambda_2 vs N, one curve per topology
%           (how connectivity scales with network size).
% Panel 2 : event-triggered settling time vs N (convergence speed).
% Panel 3 : event-triggered broadcasts vs N (communication cost).
% Panel 4 : settling time vs lambda_2 across every run (the underlying law:
%           faster consensus comes from larger algebraic connectivity).

labels = unique(summary_table.topology, 'stable');
num_labels = numel(labels);
colors = lines(num_labels);

fig = figure('Name', sprintf('Topology sweep (%s)', plant_label), ...
    'Position', [100, 100, 1100, 800]);

% Panel 1: lambda_2 vs N
subplot(2, 2, 1);
hold on;
for idx = 1:num_labels
    rows = summary_table.topology == labels(idx);
    sub = sortrows(summary_table(rows, :), 'N');
    plot(sub.N, sub.lambda2, '-o', 'LineWidth', 1.5, ...
        'Color', colors(idx, :), 'MarkerFaceColor', colors(idx, :));
end
grid on;
xlabel('number of agents N');
ylabel('\lambda_2 (algebraic connectivity)');
title('connectivity vs network size');
legend(labels, 'Location', 'northwest', 'Interpreter', 'none');

% Panel 2: settling time vs N
subplot(2, 2, 2);
hold on;
for idx = 1:num_labels
    rows = summary_table.topology == labels(idx);
    sub = sortrows(summary_table(rows, :), 'N');
    plot(sub.N, sub.tau_event, '-o', 'LineWidth', 1.5, ...
        'Color', colors(idx, :), 'MarkerFaceColor', colors(idx, :));
end
grid on;
xlabel('number of agents N');
ylabel('settling time \tau [s]');
title('event-triggered convergence speed vs N');

% Panel 3: broadcasts vs N
subplot(2, 2, 3);
hold on;
for idx = 1:num_labels
    rows = summary_table.topology == labels(idx);
    sub = sortrows(summary_table(rows, :), 'N');
    errorbar(sub.N, sub.events_mean, sub.events_std, '-o', 'LineWidth', 1.5, ...
        'Color', colors(idx, :), 'MarkerFaceColor', colors(idx, :));
end
grid on;
xlabel('number of agents N');
ylabel('total broadcasts');
title('event-triggered communication cost vs N');

% Panel 4: settling time vs lambda_2 (all runs)
subplot(2, 2, 4);
hold on;
for idx = 1:num_labels
    rows = summary_table.topology == labels(idx);
    sub = summary_table(rows, :);
    scatter(sub.lambda2, sub.tau_event, 45, colors(idx, :), 'filled');
end
grid on;
xlabel('\lambda_2 (algebraic connectivity)');
ylabel('settling time \tau [s]');
title('convergence speed is set by \lambda_2');

sgtitle(sprintf('Network topology & size sweep — %s plant', plant_label), ...
    'Interpreter', 'none');

if nargin >= 3 && ~isempty(results_dir)
    stem = fullfile(results_dir, sprintf('topology_sweep_%s', plant_label));
    saveas(fig, [stem, '.png']);
    saveas(fig, [stem, '.fig']);
end
end
