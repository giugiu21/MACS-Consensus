%UNDAMPED SYSTEM - -> traiettoria desiderata nel caso con leader -> caso continuo |->caso senza leader
%                                                                   |-> caso con leader -> confronto tra grafi(?)

%                                                                -> caso trigger (diversi trigger_type) |-> caso senza leader
%                                                                                         |-> caso con leader

%                                                                             confronto con GARCIA?

clear;
clc;
close all;

%% Path setup
% adding all the scripts used to the same Matlab path to run
script_dir = fileparts(mfilename('fullpath'));

addpath(script_dir);
add_matlab_paths();
results_dir = fullfile(script_dir, 'results');

if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% Simulation settings

dt = 0.001; %integration step
T = 100; %total time of simulation

time = 0:dt:T;
num_steps = numel(time);

%this was added to maybe later do a comparison between seeds
%for now I only use the first seed
seeds = [1, 2, 3];
seed = seeds(1);

%% Initialize agent and communication graph

agent = init_undamped_agent(); %inizializzo i sistemi massa molla smorzatore

N = 5; % numero totale di agenti

%qui in teoria ho 3 opzioni id grafo (per ora ne uso solo 1):
%.   - 'path'
%.   - 'cycle'
%.   - 'connected'
graph = build_consensus_graph(N, 'path', struct('show_plot', false)); %costruisco il grafo

%computing the GAIN using LQR (linear quadratic regulator)
K_consensus = compute_lqr_consensus_gain(agent);

%% Leader-follower reference configuration

leader_id = 1; % leader chosen

%Possibilità di cambiare la posizione ad un valore diverso da 0
%Per ora mantengo 0 per un confronto più chiaro tra tutte le casistiche
%inoltre con posizione diversa da 0 il trigger_type "state-relative" ovviamente non funziona
%.   - non funziona perchè il calcolo della threshold si basa sullo stato assoluto del sistema che con una posizione desiderata è necessariamente diverso da 0
%.     questo vuol dire che quando gli agenti si avvicinano allo stato desiderato la threshold smette di scendere perchè non tende a 0 ma tende al valore dello stato
%      e quindi di conseguenza vengono tollerati errori maggiori


omega_n = sqrt(agent.k_spring / agent.mass);     % natural frequency
A_ref = 1;                                       % trajectory amplitude
x_desired_fun = @(t) A_ref * [sin(omega_n * t); omega_n * cos(omega_n * t)];
agent.x_desired = x_desired_fun(0);              % engine fallback field


agent_pinned = agent;
agent_pinned.K_reference = K_consensus;
agent_free = agent;
agent_free.K_reference = zeros(agent.m_input, agent.n);  % disables the leader



% Required equilibrium feedforward:
% !!!! this computes the force needed to keep the mass in the position p_d
% without it the spring will default back to the equilibrium position = 0
%in questo momento essendo la posizione desiderata = 0, u_desired = 0
%u_desired = agent.k_spring * p_desired;

% Reference feedback gain.
% It is kept separate from K_consensus even if the same numerically
% same Gain for: neighbour consensus and tracking leader-follower
%Might not be the best idea numerically maybe we need to differentiate?????
%K_reference = K_consensus;

%% Trigger configuration

%options available for trigger type:
%       - "absolute"
%.      - "relative"
%.      - "state-relative" -> potrebbe creare problemi con p_d, comunicazioni smettono troppo presto??
%.      - "state-disagreement"
%.      - "weighted"
%.      - ...?

trigger_type = "state-disagreement";

sigma = 0.05; % the smaller the number the more frequent the communications between agents
epsilon_trigger = 1e-5;

trigger_params = struct();

trigger_params.W = diag([1, 0.5]); %weigth matrix for the state, position is more important than velocity
trigger_params.mass = agent.mass;
trigger_params.k_spring = agent.k_spring;
trigger_params.C = agent.C;

trigger_params.system_type = 'undamped';
trigger_params.minimum_inter_event_time = 0.5; % [s]

%state relative, state-disagreement parameters
trigger_params.state_gain = 0.05; %0.5 prima
trigger_params.disagreement_gain = 1.0;
trigger_params.disagreement_tol = 1e-1;
trigger_params.dt = dt;


%% Initial condition

x0 = make_random_initial_condition(seed, N, agent.n);

%% Common options

common_opts = struct();

common_opts.leader_id = leader_id;
common_opts.system = "no-equilibrium"; 

common_opts.K_reference = K_consensus;

common_opts.x_desired_fun = x_desired_fun;
%common_opts.u_desired = u_desired;

common_opts.sigma = sigma;
common_opts.epsilon_trigger = epsilon_trigger;

common_opts.trigger_type = trigger_type;
common_opts.trigger_params = trigger_params;

%% Run open-loop case
% seeing how the agents behave and if they reach consensus with no control law

fprintf('Running open-loop case...\n');

results.open_loop = ...
    run_open_loop_consensus( ...
        x0, ...
        agent, ...
        graph, ...
        time, ...
        dt);

%% Run continuous leaderless case

%CONTROLLO: u_i = u_i,cons + u_d -K_reference * sum(a_ij * (x_i - x_d))
% tutti gli agenti ricevono x_d

opts = common_opts;

opts.communication_mode = "continuous";
opts.control_case = "leaderless";

fprintf('Running continuous leaderless case...\n');

results.continuous.leaderless = ...
    run_continuous_consensus( ...
        x0, ...
        agent, ...
        graph, ...
        K_consensus, ...
        time, ...
        dt, ...
        opts);

%% Run continuous leader-follower case
%CONTROLLO:  u = - (L x K_consensus) * x
%CONTROLLO: -> leader: u_1 = u_1,cons + u_d - K_reference(x_1(t) - x_d(t))
%           -> altri agenti: u_i = u_i,cons + u_d 
%feedback diretto rispetto ad x_d soltanto al leader


opts = common_opts;

opts.communication_mode = "continuous";
opts.control_case = "leader_follower";

fprintf('Running continuous leader-follower case...\n');

results.continuous.leader_follower = ...
    run_continuous_consensus( ...
        x0, ...
        agent, ...
        graph, ...
        K_consensus, ...
        time, ...
        dt, ...
        opts);

%% Run event-triggered leaderless case
%CONTROLLO: u_i = u_i,cons + u_d -K_reference * sum(a_ij * (x_i(t) - x_d(t)))
%usa gli stati trasmessi?: x_hat_i è l'ultimo stato trasmesso dall'agente i
opts = common_opts;

opts.communication_mode = "event_triggered";
opts.control_case = "leaderless";

fprintf('Running event-triggered leaderless case...\n');

results.trigger.leaderless = ...
    run_event_triggered_consensus( ...
        x0, ...
        agent, ...
        graph, ...
        K_consensus, ...
        time, ...
        dt, ...
        opts);

%% Run event-triggered leader-follower case
%CONTROLLO: -> leader: u_1 = -K_consensus * sum(a_ij * (x_hat_1 - x_hat_j) - K_reference * (x_1 - x_d) + u_d)
%           -> altri agenti: u_i = -K_consensus * sum(a_ij * (x_hat_1 - x_hat_j) + u_d) 
%feedback diretto rispetto ad x_d(t) è soltanto nel leader


opts = common_opts;

opts.communication_mode = "event_triggered";
opts.control_case = "leader_follower";

fprintf('Running event-triggered leader-follower case...\n');

results.trigger.leader_follower = ...
    run_event_triggered_consensus( ...
        x0, ...
        agent, ...
        graph, ...
        K_consensus, ...
        time, ...
        dt, ...
        opts);

%% Getting the results

r_open = results.open_loop;

r_cont_free = results.continuous.leaderless;
r_cont_leader = results.continuous.leader_follower;

r_trig_free = results.trigger.leaderless;
r_trig_leader = results.trigger.leader_follower;


%% Console summary

fprintf('\n');
fprintf('===============================================\n');
fprintf('UNDAMPED CONSENSUS COMPARISON\n');
fprintf('===============================================\n');

fprintf('Leader ID: %d\n', leader_id);
fprintf('Natural Frequency: %d\n', omega_n);
fprintf('Trajectory Amplitude: %d\n', A_ref);

fprintf('Trigger type: %s\n', char(trigger_type));
fprintf('Sigma: %.6e\n', sigma);
fprintf('Epsilon: %.6e\n', epsilon_trigger);

fprintf('\nFinal output disagreement:\n');

fprintf('  open-loop:                  %.6e\n', ...
    r_open.final_output_disagreement);

fprintf('  continuous leaderless:      %.6e\n', ...
    r_cont_free.final_output_disagreement);

fprintf('  continuous leader-follower: %.6e\n', ...
    r_cont_leader.final_output_disagreement);

fprintf('  trigger leaderless:         %.6e\n', ...
    r_trig_free.final_output_disagreement);

fprintf('  trigger leader-follower:    %.6e\n', ...
    r_trig_leader.final_output_disagreement);

fprintf('\nEvent-triggered communications:\n');

fprintf('  leaderless total events:      %d\n', ...
    r_trig_free.total_events);

fprintf('  leader-follower total events: %d\n', ...
    r_trig_leader.total_events);

fprintf('\nEvents per agent — leaderless:\n');

for agent_id = 1:N
    fprintf('  agent %d: %d\n', ...
        agent_id, ...
        r_trig_free.events_per_agent(agent_id));
end

fprintf('\nEvents per agent — leader-follower:\n');

for agent_id = 1:N
    fprintf('  agent %d: %d\n', ...
        agent_id, ...
        r_trig_leader.events_per_agent(agent_id));
end


%% Plot window

plot_end_time = 80;

plot_mask = time <= plot_end_time;

%% Figure: open-loop vs leaderless consensus

fig_leaderless_comparison = figure( ...
    'Name', 'open-loop and leaderless comparison', ...
    'Position', [100, 100, 1000, 850]);

layout_leaderless = tiledlayout(3, 1);

layout_leaderless.TileSpacing = 'compact';
layout_leaderless.Padding = 'compact';

%% Open-loop

nexttile;

plot( ...
    time(plot_mask), ...
    r_open.y_history(:, plot_mask)', ...
    'LineWidth', 1.0);

grid on;

xlabel('time [s]');
ylabel('position');

title('Open-loop: no control, no communication');

xlim([0, plot_end_time]);

%% Continuous leaderless

nexttile;

plot( ...
    time(plot_mask), ...
    r_cont_free.y_history(:, plot_mask)', ...
    'LineWidth', 1.0);

grid on;

xlabel('time [s]');
ylabel('position');

title('Leaderless consensus — continuous communication');

xlim([0, plot_end_time]);

%% Figure 1 Event-triggered leaderless

nexttile;

plot( ...
    time(plot_mask), ...
    r_trig_free.y_history(:, plot_mask)', ...
    'LineWidth', 1.0);

grid on;

xlabel('time [s]');
ylabel('position');

title(sprintf( ...
    'Leaderless consensus — event-triggered (%s)', ...
    trigger_type));

xlim([0, plot_end_time]);

title(layout_leaderless, ...
    'Effect of the consensus controller');

saveas( ...
    fig_leaderless_comparison, ...
    fullfile(results_dir, ...
    'undamped_open_loop_vs_leaderless_outputs.png'));

saveas( ...
    fig_leaderless_comparison, ...
    fullfile(results_dir, ...
    'undamped_open_loop_vs_leaderless_outputs.fig'));

%% Figure 2: outputs for all four cases

fig_outputs = figure( ...
    'Name', 'continuous and event-triggered consensus comparison', ...
    'Position', [100, 100, 1200, 780]);

layout_outputs = tiledlayout(2, 2);

layout_outputs.TileSpacing = 'compact';
layout_outputs.Padding = 'compact';

%% Continuous leaderless

nexttile;

plot( ...
    time(plot_mask), ...
    r_cont_free.y_history(:, plot_mask)', ...
    'LineWidth', 1.0);

grid on;

xlabel('time [s]');
ylabel('position');

title('leaderless — continuous');

xlim([0, plot_end_time]);

%% Event-triggered leaderless

nexttile;

plot( ...
    time(plot_mask), ...
    r_trig_free.y_history(:, plot_mask)', ...
    'LineWidth', 1.0);

grid on;

xlabel('time [s]');
ylabel('position');

title(sprintf( ...
    'leaderless — event-triggered (%s)', ...
    trigger_type));

xlim([0, plot_end_time]);

%% Continuous leader-follower

nexttile;

plot( ...
    time(plot_mask), ...
    r_cont_leader.y_history(:, plot_mask)', ...
    'LineWidth', 1.0);

hold on;
grid on;


xlabel('time [s]');
ylabel('position');

title('leader-follower — continuous');

xlim([0, plot_end_time]);

legend( ...
    [compose('agent %d', 1:N), {'p_d'}], ...
    'Location', ...
    'best');

%% Event-triggered leader-follower

nexttile;

plot( ...
    time(plot_mask), ...
    r_trig_leader.y_history(:, plot_mask)', ...
    'LineWidth', 1.0);

hold on;
grid on;


xlabel('time [s]');
ylabel('position');

title(sprintf( ...
    'leader-follower — event-triggered (%s)', ...
    trigger_type));

xlim([0, plot_end_time]);

title(layout_outputs, ...
    'Undamped mass-spring-damper consensus comparison');

saveas( ...
    fig_outputs, ...
    fullfile(results_dir, 'undamped_consensus_outputs.png'));

saveas( ...
    fig_outputs, ...
    fullfile(results_dir, 'undamped_consensus_outputs.fig'));


%% Figure 4: trigger times, leaderless vs leader-follower

fig_trigger_times = figure( ...
    'Name', 'event-triggered communication times', ...
    'Position', [100, 100, 1100, 620]);

layout_triggers = tiledlayout(2, 1);

layout_triggers.TileSpacing = 'compact';
layout_triggers.Padding = 'compact';

%% Leaderless trigger times

[event_agents_free, event_steps_free] = ...
    find(r_trig_free.event_history);

event_times_free = time(event_steps_free);

nexttile;

scatter( ...
    event_times_free, ...
    event_agents_free, ...
    8, ...
    'filled');

grid on;

xlabel('time [s]');
ylabel('agent');

title('leaderless');

xlim([time(1), time(end)]);
ylim([0.5, N + 0.5]);

yticks(1:N);

%% Leader-follower trigger times

[event_agents_leader, event_steps_leader] = ...
    find(r_trig_leader.event_history);

event_times_leader = time(event_steps_leader);

nexttile;

scatter( ...
    event_times_leader, ...
    event_agents_leader, ...
    8, ...
    'filled');

grid on;

xlabel('time [s]');
ylabel('agent');

title('leader-follower');

xlim([time(1), time(end)]);
ylim([0.5, N + 0.5]);

yticks(1:N);

title(layout_triggers, ...
    sprintf('Event-triggered communication times — %s', ...
    trigger_type));

saveas( ...
    fig_trigger_times, ...
    fullfile(results_dir, 'undamped_trigger_times.png'));

saveas( ...
    fig_trigger_times, ...
    fullfile(results_dir, 'undamped_trigger_times.fig'));

%% Figure 5: cumulative events per agent

fig_cumulative = figure( ...
    'Name', 'cumulative events', ...
    'Position', [100, 100, 1100, 620]);

layout_cumulative = tiledlayout(1, 2);

layout_cumulative.TileSpacing = 'compact';
layout_cumulative.Padding = 'compact';

agent_names = compose('agent %d', 1:N);

%% Leaderless cumulative events

nexttile;

plot( ...
    time, ...
    r_trig_free.cumulative_events', ...
    'LineWidth', 1.2);

grid on;

xlabel('time [s]');
ylabel('cumulative events');

title('leaderless');

legend( ...
    agent_names, ...
    'Location', ...
    'northwest');

%% Leader-follower cumulative events

nexttile;

plot( ...
    time, ...
    r_trig_leader.cumulative_events', ...
    'LineWidth', 1.2);

grid on;

xlabel('time [s]');
ylabel('cumulative events');

title('leader-follower');

legend( ...
    agent_names, ...
    'Location', ...
    'northwest');

title(layout_cumulative, ...
    'Cumulative event-triggered communications');

saveas( ...
    fig_cumulative, ...
    fullfile(results_dir, 'undamped_cumulative_events.png'));

saveas( ...
    fig_cumulative, ...
    fullfile(results_dir, 'undamped_cumulative_events.fig'));

%% Figure 6: total events per agent

fig_events_bar = figure( ...
    'Name', 'events per agent', ...
    'Position', [100, 100, 1000, 520]);

layout_events_bar = tiledlayout(1, 2);

layout_events_bar.TileSpacing = 'compact';
layout_events_bar.Padding = 'compact';

%% Leaderless events per agent

nexttile;

bar(1:N, r_trig_free.events_per_agent);

grid on;

xlabel('agent');
ylabel('number of events');

title(sprintf( ...
    'leaderless — total = %d', ...
    r_trig_free.total_events));

xticks(1:N);

%% Leader-follower events per agent

nexttile;

bar(1:N, r_trig_leader.events_per_agent);

grid on;

xlabel('agent');
ylabel('number of events');

title(sprintf( ...
    'leader-follower — total = %d', ...
    r_trig_leader.total_events));

xticks(1:N);

title(layout_events_bar, ...
    'Event-triggered updates per agent');

saveas( ...
    fig_events_bar, ...
    fullfile(results_dir, 'undamped_events_per_agent.png'));

saveas( ...
    fig_events_bar, ...
    fullfile(results_dir, 'undamped_events_per_agent.fig'));

%% Figure 7: trigger value and threshold for one selected agent

selected_agent = leader_id;

fig_trigger_condition = figure( ...
    'Name', 'trigger condition', ...
    'Position', [100, 100, 1100, 620]);

layout_condition = tiledlayout(2, 1);

layout_condition.TileSpacing = 'compact';
layout_condition.Padding = 'compact';

%% Leaderless condition

nexttile;

plot( ...
    time, ...
    r_trig_free.trigger_value_history(selected_agent, :), ...
    'LineWidth', 1.1);

hold on;

plot( ...
    time, ...
    r_trig_free.threshold_history(selected_agent, :), ...
    '--', ...
    'LineWidth', 1.1);

grid on;

xlabel('time [s]');
ylabel('trigger quantity');

title(sprintf( ...
    'leaderless — agent %d', ...
    selected_agent));

legend( ...
    'trigger value', ...
    'threshold', ...
    'Location', ...
    'best');

%% Leader-follower condition

nexttile;

plot( ...
    time, ...
    r_trig_leader.trigger_value_history(selected_agent, :), ...
    'LineWidth', 1.1);

hold on;

plot( ...
    time, ...
    r_trig_leader.threshold_history(selected_agent, :), ...
    '--', ...
    'LineWidth', 1.1);

grid on;

xlabel('time [s]');
ylabel('trigger quantity');

title(sprintf( ...
    'leader-follower — agent %d', ...
    selected_agent));

legend( ...
    'trigger value', ...
    'threshold', ...
    'Location', ...
    'best');

title(layout_condition, ...
    'Trigger value and threshold comparison');

saveas( ...
    fig_trigger_condition, ...
    fullfile(results_dir, 'undamped_trigger_condition.png'));

saveas( ...
    fig_trigger_condition, ...
    fullfile(results_dir, 'undamped_trigger_condition.fig'));