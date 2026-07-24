function plot_ring_gallery(topo_list, N_showcase, plant_label, results_dir)
%PLOT_RING_GALLERY Draw the ring communication graphs used in the sweep.
%
% One panel per ring degree k in topo_list, all drawn at N_showcase nodes,
% so the effect of the interconnection axis (k) on the network structure is
% visible at a glance: k = 2 is the bare cycle, larger k adds the chords
% that raise the algebraic connectivity lambda_2. Nodes are placed on a
% circle and edges are read straight from the adjacency matrix, so this
% needs no Graph/Network toolbox.

num_topo = numel(topo_list);
fig = figure('Name', sprintf('Ring structures (%s)', plant_label), ...
    'Position', [100, 100, 300 * num_topo, 360]);

for idx = 1:num_topo
    topo = topo_list(idx);
    graph = build_consensus_graph(N_showcase, topo.type, ...
        struct('show_plot', false, 'k', topo.k));

    ax = subplot(1, num_topo, idx);
    draw_ring(ax, graph);
    title(ax, sprintf('%s\n%d nodes, %d edges, \\lambda_2 = %.3f', ...
        topo.label, graph.N, graph.num_edges, graph.lambda2), ...
        'Interpreter', 'tex');
end

sgtitle(sprintf('Ring communication graphs (N = %d) — %s plant', ...
    N_showcase, plant_label), 'Interpreter', 'none');

if nargin >= 4 && ~isempty(results_dir)
    stem = fullfile(results_dir, sprintf('ring_structures_%s', plant_label));
    saveas(fig, [stem, '.png']);
    saveas(fig, [stem, '.fig']);
end
end

function draw_ring(ax, graph)
% draw one graph with the nodes evenly spaced on a circle
N = graph.N;
theta = 2 * pi * (0:N-1) / N + pi/2;   % first node at the top
xs = cos(theta);
ys = sin(theta);

hold(ax, 'on');

% edges from the (symmetric) adjacency matrix
Adj = graph.Adj;
for i = 1:N
    for j = i+1:N
        if Adj(i, j) ~= 0
            plot(ax, [xs(i), xs(j)], [ys(i), ys(j)], '-', ...
                'Color', [0.65, 0.65, 0.65], 'LineWidth', 1.0);
        end
    end
end

% nodes
plot(ax, xs, ys, 'o', 'MarkerSize', 9, ...
    'MarkerFaceColor', [0.10, 0.45, 0.80], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 0.8);

% node indices, pushed slightly outward
for i = 1:N
    text(ax, 1.16 * xs(i), 1.16 * ys(i), num2str(i), ...
        'HorizontalAlignment', 'center', 'FontSize', 8);
end

axis(ax, 'equal');
axis(ax, 'off');
xlim(ax, [-1.4, 1.4]);
ylim(ax, [-1.4, 1.4]);
end
