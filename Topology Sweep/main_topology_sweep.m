% Network topology & size sweep for event-triggered consensus.
%
% Consensus speed is governed by the algebraic connectivity lambda_2 of the
% communication Laplacian: more interconnections -> larger lambda_2 ->
% faster agreement, but (in the event-triggered scheme) more broadcasts.
% All graphs are k-regular RINGS, so both axes stay in one family:
%   1. the number of agents N              (network size),
%   2. the number of connections per node k (ring degree, N*k/2 edges),
% and reports, for each combination, connectivity, settling time and
% communication cost. It runs on either the damped mass-spring-damper plant
% or the undamped oscillator by flipping a single switch below.
%
% Requires: Control System Toolbox (lqr) for compute_lqr_consensus_gain.

clear;
clc;
close all;

%% ------------------------------------------------------------------ paths
% This folder is self-contained: every core function it needs is copied into
% lib/ (see lib/README.md), so the sweep runs without adding any other module
% of the repository to the path.
script_dir     = fileparts(mfilename('fullpath'));                 % this folder
functions_dir  = fullfile(script_dir, 'functions');               % sweep helpers
lib_dir        = fullfile(script_dir, 'lib');                      % vendored copies of the core functions
results_dir    = fullfile(script_dir, 'results');                 % output folder
addpath(script_dir, functions_dir, lib_dir);

if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% =====================================================================
%  USER CONFIGURATION  -- edit this block only
%  =====================================================================

% ---- plant -----------------------------------------------------------
plant_type = 'damped';        % 'damped' | 'undamped'

% Every graph here is a k-regular RING: node i is linked to its k nearest
% neighbours on the ring (k/2 on each side, N*k/2 edges total). 
%   - N : number of nodes           (network SIZE axis)
%   - k : connections per node       (INTERCONNECTION axis, even, k <= N-1)
% k = 2 is the plain cycle; larger k adds interconnections and raises lambda_2.

% ---- network SIZE  ----------------------------------------------
N_list = [10, 14, 18]; 

% ---- network INTERCONNECTION axis (ring degree k) -------------------
k_list = [2, 4, 6];          % connections per node 

% one ring-k topology per degree; N is swept inside sweep_network_topology
topo_list = repmat(make_topology('ring-k', '', 2), numel(k_list), 1);
for i = 1:numel(k_list)
    topo_list(i) = make_topology('ring-k', ...
        sprintf('ring (k=%d)', k_list(i)), k_list(i));
end

% add a star (hub + N-1 leaves): lambda_2 = 1 for every N, lambda_max = N.
% k is unused for the star, so any value is fine. This is the contrast case:
% lambda_2 stays constant while lambda_max = N grows with the network size.
topo_list(end + 1) = make_topology('star', 'star', 2);

% number of nodes used only to DRAW the ring structures 
N_showcase = 12;

% ---- simulation / trigger -------------------------------------------
num_seeds     = 8;              % random initial conditions per combination
dt            = 0.001;          % integration step
T             = 60;             % horizon [s]
sigma         = 0.005;           % trigger threshold factor
epsilon_trig  = 1e-5;           % trigger dead-zone
consensus_tol = 5e-2;           % settling tolerance on normalized disagreement

%% ------------------------------------------------------------------ run

switch lower(plant_type)
    case 'damped'
        agent = init_mass_spring_damper_agent();
        trigger_type = "state-disagreement";
    case 'undamped'
        agent = init_undamped_agent();
        trigger_type = "relative";         
    otherwise
        error('plant_type must be ''damped'' or ''undamped''');
end

K = compute_lqr_consensus_gain(agent);

% trigger parameters 
trigger_params = struct();
trigger_params.W = diag([1, 0.5]);
trigger_params.mass = agent.mass;
trigger_params.k_spring = agent.k_spring;
trigger_params.C = agent.C;
trigger_params.state_gain = 0.5;
trigger_params.disagreement_gain = 0.9;
trigger_params.disagreement_tol = 1e-2;

sim_cfg = struct();
sim_cfg.dt = dt;
sim_cfg.time = 0:dt:T;
sim_cfg.seeds = 1:num_seeds;
sim_cfg.consensus_tol = consensus_tol;
sim_cfg.trigger_type = trigger_type;
sim_cfg.sigma = sigma;
sim_cfg.epsilon_trigger = epsilon_trig;
sim_cfg.trigger_params = trigger_params;

fprintf('Topology sweep on the %s plant\n', plant_type);
fprintf('  %d topologies x %d network sizes x %d seeds (T = %.0f s)\n', ...
    numel(topo_list), numel(N_list), num_seeds, T);
fprintf('  trigger rule: %s | sigma = %.3g\n\n', char(trigger_type), sigma);

%% run the sweep
summary_table = sweep_network_topology(agent, K, topo_list, N_list, sim_cfg);

fprintf('\n');
disp(summary_table);

if ~all(summary_table.all_modes_stable)
    warning('Some (topology, N) combinations have unstable disagreement modes.');
end

%% save + plot
save(fullfile(results_dir, sprintf('topology_sweep_%s_results.mat', plant_type)), ...
    'summary_table', 'topo_list', 'N_list', 'sim_cfg', 'agent', 'K', 'plant_type');
writetable(summary_table, ...
    fullfile(results_dir, sprintf('topology_sweep_%s_summary.csv', plant_type)));

plot_topology_sweep(summary_table, plant_type, results_dir);
plot_ring_gallery(topo_list, N_showcase, plant_type, results_dir);

fprintf('Saved results in:\n%s\n', results_dir);

%% ------------------------------------------------------------------ local
function topo = make_topology(type, label, k)

if nargin < 3
    k = 2;
end
topo = struct('type', type, 'label', label, 'k', k);
end
