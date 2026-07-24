function summary_table = sweep_network_topology(agent, K, topo_list, N_list, sim_cfg)
%SWEEP_NETWORK_TOPOLOGY Compare consensus across topologies and network size.
%
% For every (topology, N) combination this runs the continuous and the
% event-triggered consensus over sim_cfg.seeds random initial conditions
% and aggregates, into one table row:
%   - the algebraic connectivity lambda_2 and the edge count (the
%     structural quantities that set the convergence speed),
%   - the settling time of both schemes (mean over the seeds),
%   - the communication cost of the event-triggered scheme (broadcasts),
%   - the fraction of seeds that reach consensus and the settled level.
%
% Inputs:
%   agent      : plant struct (init_mass_spring_damper_agent / init_undamped_agent)
%   K          : consensus gain (compute_lqr_consensus_gain)
%   topo_list  : struct array with fields .label, .type, .k (see main)
%   N_list     : vector of agent counts to sweep
%   sim_cfg    : struct with .dt, .time, .seeds, .consensus_tol,
%                .trigger_type, .sigma, .epsilon_trigger, .trigger_params

num_topo = numel(topo_list);
num_N = numel(N_list);
seeds = sim_cfg.seeds;
num_seeds = numel(seeds);
rows = num_topo * num_N;

% one accumulator per output column
topology                = strings(rows, 1);
N_col                   = zeros(rows, 1);
num_edges               = zeros(rows, 1);
mean_degree             = zeros(rows, 1);
lambda2                 = zeros(rows, 1);
tau_continuous          = zeros(rows, 1);
tau_event               = zeros(rows, 1);
events_mean             = zeros(rows, 1);
events_std              = zeros(rows, 1);
norm_final_disagreement = zeros(rows, 1);
frac_consensus          = zeros(rows, 1);


% physical trigger parameters do not depend on N: fill them once
trigger_params = sim_cfg.trigger_params;
trigger_params.mass = agent.mass;
trigger_params.k_spring = agent.k_spring;
trigger_params.C = agent.C;

row = 0;
for topo_idx = 1:num_topo
    topo = topo_list(topo_idx);

    for N_idx = 1:num_N
        row = row + 1;
        N = N_list(N_idx);

        % ring-k needs an even degree in [2, N-1]; clamp so small N never
        % crashes the sweep (ignored by the other topologies)
        graph_opts = struct('show_plot', false, ...   % never plot inside the sweep
            'k', min(topo.k, 2 * floor((N - 1) / 2)));

        graph = build_consensus_graph(N, topo.type, graph_opts);
        mode_info = check_consensus_modes(agent, graph, K);

        tau_c = nan(1, num_seeds);
        tau_e = nan(1, num_seeds);
        events = zeros(1, num_seeds);
        normalized_final = zeros(1, num_seeds);
        reached = false(1, num_seeds);

        for seed_idx = 1:num_seeds
            x0 = make_random_initial_condition(seeds(seed_idx), N, agent.n);

            cont_result = run_continuous_consensus( ...
                x0, agent, graph, K, sim_cfg.time, sim_cfg.dt);
            event_result = run_event_triggered_consensus( ...
                x0, agent, graph, K, sim_cfg.time, sim_cfg.dt, ...
                sim_cfg.sigma, sim_cfg.epsilon_trigger, ...
                sim_cfg.trigger_type, trigger_params);

            tau_c(seed_idx) = measure_convergence_time( ...
                cont_result.output_disagreement, sim_cfg.time, sim_cfg.consensus_tol);
            tau_e(seed_idx) = measure_convergence_time( ...
                event_result.output_disagreement, sim_cfg.time, sim_cfg.consensus_tol);
            events(seed_idx) = event_result.total_events;
            [reached(seed_idx), ~, normalized_final(seed_idx)] = ...
                classify_run_outcome(event_result.output_disagreement, sim_cfg.consensus_tol);
        end

        topology(row)                = string(topo.label);
        N_col(row)                   = N;
        num_edges(row)               = graph.num_edges;
        mean_degree(row)             = graph.mean_degree;
        lambda2(row)                 = graph.lambda2;
        tau_continuous(row)          = mean(tau_c, 'omitnan');
        tau_event(row)               = mean(tau_e, 'omitnan');
        events_mean(row)             = mean(events);
        events_std(row)              = std(events);
        norm_final_disagreement(row) = mean(normalized_final);
        frac_consensus(row)          = mean(reached);


        fprintf('  %-18s N = %2d | edges = %3d | lambda2 = %7.4f | tau_evt = %6.3f s | events = %7.1f | consensus = %3.0f%%\n', ...
            topo.label, N, graph.num_edges, graph.lambda2, ...
            tau_event(row), events_mean(row), 100 * frac_consensus(row));
    end
end

summary_table = table(topology, N_col, num_edges, mean_degree, lambda2, ...
    tau_continuous, tau_event, events_mean, events_std, ...
    norm_final_disagreement, frac_consensus, ...
    'VariableNames', {'topology', 'N', 'num_edges', 'mean_degree', ...
    'lambda2', 'tau_continuous', 'tau_event', 'events_mean', 'events_std', ...
    'norm_final_disagreement', 'frac_consensus'});
end
