function graph = build_consensus_graph(N, graph_type, opts)
% build a connected communication graph and (optionally) plot lambda_2
%
% The whole simulation stack only ever reads graph.N, graph.Adj and
% graph.L, so the network topology is fully encapsulated here: changing
% the topology or its number of interconnections requires no change
% downstream.
%
% graph_type (interconnections grow from top to bottom, at fixed N):
%   'path'     : N-1 edges, minimally connected chain (smallest lambda_2)
%   'cycle'    : N edges, ring, every node has degree 2
%   'star'     : N-1 edges, one hub connected to all the other nodes
%   'ring-k'   : k-regular ring, each node linked to its k nearest
%                neighbours on the ring (k even, N*k/2 edges). Tune k with
%                opts.k to sweep the number of interconnections at fixed N;
%                k = 2 reproduces 'cycle', k = N-1 reproduces 'complete'.
%   'complete' : N(N-1)/2 edges, every pair connected (largest lambda_2)
%
% opts (optional struct):
%   .k         : degree for 'ring-k' (even integer, default 2)
%   .show_plot : open the lambda_2 bar chart (default true). Set false in
%                sweeps so the loop does not spawn one figure per graph.

if nargin < 2 || isempty(graph_type)
    graph_type = 'path';
end
if nargin < 3 || isempty(opts)
    opts = struct();
end
if ~isfield(opts, 'show_plot')
    opts.show_plot = true;
end
if ~isfield(opts, 'k')
    opts.k = 2;
end

graph_type = char(graph_type);

switch lower(graph_type)
    case 'path'
        Adj = zeros(N, N);
        for i = 1:N-1
            Adj(i, i+1) = 1;
            Adj(i+1, i) = 1;
        end

    case 'cycle'
        Adj = build_ring_k_adjacency(N, 2);

    case 'star'
        Adj = zeros(N, N);
        Adj(1, 2:N) = 1;   % node 1 is the hub
        Adj(2:N, 1) = 1;

    case 'ring-k'
        Adj = build_ring_k_adjacency(N, opts.k);

    case 'complete'
        Adj = ones(N, N) - eye(N);

    otherwise
        error('build_consensus_graph:unknownType', ...
            'unknown graph type "%s"', graph_type);
end

Deg = diag(sum(Adj, 2));
L = Deg - Adj;

lambda = eig(L);
lambda_sorted = sort(real(lambda));
lambda2 = lambda_sorted(2);

num_edges = nnz(Adj) / 2;

if opts.show_plot
    figure('Name', 'Laplacian second eigenvalue');
    bar(1, lambda2, 0.35);
    grid on;
    xlim([0.5 1.5]);
    ylim([0, max(lambda2 * 1.15, eps)]);
    set(gca, 'XTick', 1, 'XTickLabel', {'\lambda_2'});
    ylabel('eigenvalue');
    title(sprintf('Second Laplacian eigenvalue (%s graph, N = %d)', ...
        graph_type, N));
    text(1, lambda2, sprintf('%.4g', lambda2), ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom');
end

graph.N = N;
graph.type = graph_type;
graph.Adj = Adj;
graph.Deg = Deg;
graph.L = L;
graph.lambda = lambda_sorted;
graph.is_connected = lambda2 > 1e-8;
graph.lambda2 = lambda2;
graph.num_edges = num_edges;
graph.mean_degree = 2 * num_edges / N;
end

function Adj = build_ring_k_adjacency(N, k)
% k-regular ring: each node is linked to its k nearest neighbours on the
% ring (k/2 on each side). k must be even and in [2, N-1].

if mod(k, 2) ~= 0
    error('build_consensus_graph:oddDegree', ...
        'ring-k requires an even degree k, got k = %d', k);
end
if k < 2 || k > N - 1
    error('build_consensus_graph:degreeRange', ...
        'ring-k requires 2 <= k <= N-1 (N = %d), got k = %d', N, k);
end

Adj = zeros(N, N);
half = k / 2;
for i = 1:N
    for d = 1:half
        j = mod(i - 1 + d, N) + 1;   % neighbour d steps ahead on the ring
        Adj(i, j) = 1;
        Adj(j, i) = 1;
    end
end
end
