%UNDAMPED SYSTEM - -> tutti si coordinano per arrivare  -> caso continuo |->caso senza leader
%                     allo stato di equilibrio                     |-> caso con leader -> confronto tra grafi(?)
%                      (oscillazione continua)
%                                                        -> caso trigger (diversi trigger_type) |-> caso senza leader
%                                                                                               |-> caso con leader

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
seed = seeds(2);

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

leader_id = 1;

% The leader is an autonomous undamped oscillator.
% It receives no reference input and no consensus control.
%
% Leader dynamics:
%       x_dot_L = A*x_L
%
% Followers synchronize their state with the leader trajectory.

%% Trigger configuration

%options available for trigger type:
%       - "absolute"
%.      - "relative"
%.      - "state-relative" -> lo stato cambia sempre.
%.      - "state-disagreement"
%.      - "weighted"
%.      - ...?

trigger_type = "state-relative";

sigma = 0.2; % the smaller the number the more frequent the communications between agents
epsilon_trigger = 5e-2;

trigger_params = struct();

trigger_params.W = diag([1, 0.5]); %weigth matrix for the state, position is more important than velocity
trigger_params.mass = agent.mass;
trigger_params.k_spring = agent.k_spring;
trigger_params.C = agent.C;

trigger_params.system_type = 'undamped';
trigger_params.minimum_inter_event_time = 0.4; % [s] %con 0.5 già svalvola

%state relative, state-disagreement parameters
trigger_params.state_gain = 0.01; %0.5 prima
trigger_params.disagreement_gain = 0.4;
trigger_params.disagreement_tol = 1e-1;
trigger_params.dt = dt;


%% Initial condition

x0 = make_random_initial_condition(seed, N, agent.n);

%Condizione iniziale non nulla per il leader
%x0(leader_id) = [
%    1.0;
%    0.0
%];

%% Common options

common_opts = struct();

common_opts.leader_id = leader_id;

common_opts.sigma = sigma;
common_opts.epsilon_trigger = epsilon_trigger;

common_opts.trigger_type = trigger_type;
common_opts.trigger_params = trigger_params;

common_opts.system = "equilibrium";

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

%CONTROLLO: u_i = -K_consensus * sum(a_ij * (x_i - x_j))
% nessun agente riceve x_d, nessun agente riceve il feedback di riferimento, il feedforward è ignorato

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
%CONTROLLO: -> leader: u_1 = u_1,cons + u_d - K_reference(x_1 - x_d)
%           -> altri agenti: u_i = u_i,cons + u_d 
%feedback diretto rispetto ad x_d soltanto al leader
% ATTENZIONE -> in questo momento u_d = 0

%----------CASO CON AGGIUNTA DI UNA P_DESIRED DIVERSA DA 0---------
%in questo caso u_d è diverso da 0
%.        !!!!!Qua tocca capire se va bene dare a tutti il + u_d perchè così facendo i follower 
%         ricevono indirettamente p_d visto che u_d è calcolato in base a p_d
%         MA se non sommo u_d gli agenti non hanno la forza necessaria per arrivare a p_d
%         se sommo u_d però gli agenti conoscono indirettamente p_d quindi non capisco se è leader-follower puro

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
%CONTROLLO: u_i = -K_consensus * sum(a_ij * (x_hat_i - x_hat_j))
%usa gli stati trasmessi: x_hat_i è l'ultimo stato trasmesso dall'agente i
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
%feedback diretto rispetto ad x_d è soltanto nel leader
% ATTENZIONE -> in questo momento u_d = 0


%----------CASO CON AGGIUNTA DI UNA P_DESIRED DIVERSA DA 0---------
%in questo caso u_d è diverso da 0
%.        !!!!!Qua tocca capire se va bene dare a tutti il + u_d perchè così facendo i follower 
%         ricevono indirettamente p_d visto che u_d è calcolato in base a p_d
%         MA se non sommo u_d gli agenti non hanno la forza necessaria per arrivare a p_d
%         se sommo u_d però gli agenti conoscono indirettamente p_d quindi non capisco se è leader-follower puro

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

%% Run model-based event-triggered leaderless case
%Tra un evento e l'altro x_hat non resta congelato (ZOH)
%ma evolve con il modello open-loop dell'agente x_hat_dot = A*x_hat

opts = common_opts;

opts.communication_mode = "event_triggered";
opts.control_case = "leaderless";
opts.hold_mode = "model_based";

fprintf('Running model-based leaderless case...\n');

results.model.leaderless = ...
    run_event_triggered_consensus( ...
        x0, ...
        agent, ...
        graph, ...
        K_consensus, ...
        time, ...
        dt, ...
        opts);

%% Run model-based event-triggered leader-follower case

opts = common_opts;

opts.communication_mode = "event_triggered";
opts.control_case = "leader_follower";
opts.hold_mode = "model_based";

fprintf('Running model-based leader-follower case...\n');

results.model.leader_follower = ...
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

r_model_free = results.model.leaderless;
r_model_leader = results.model.leader_follower;


%% Console summary

fprintf('\n');
fprintf('===============================================\n');
fprintf('UNDAMPED CONSENSUS COMPARISON\n');
fprintf('===============================================\n');

omega_n = sqrt(agent.k_spring / agent.mass);
natural_period = 2 * pi / omega_n;

fprintf('Leader ID: %d\n', leader_id);
fprintf('Natural frequency: %.6f rad/s\n', omega_n);
fprintf('Natural period: %.6f s\n', natural_period);

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

%% Figure 8: model-based event-triggered communication times

fig_model_events = figure( ...
    'Name', 'model-based event-triggered communication times', ...
    'Position', [100, 100, 1100, 620]);

layout_model_events = tiledlayout(2, 1);

layout_model_events.TileSpacing = 'compact';
layout_model_events.Padding = 'compact';

%% Model-based leaderless trigger times

[event_agents_model_free, event_steps_model_free] = ...
    find(r_model_free.event_history);

event_times_model_free = time(event_steps_model_free);

nexttile;

scatter( ...
    event_times_model_free, ...
    event_agents_model_free, ...
    8, ...
    'filled');

grid on;

xlabel('time [s]');
ylabel('agent');

title('leaderless');

xlim([time(1), time(end)]);
ylim([0.5, N + 0.5]);

yticks(1:N);

%% Model-based leader-follower trigger times

[event_agents_model_leader, event_steps_model_leader] = ...
    find(r_model_leader.event_history);

event_times_model_leader = time(event_steps_model_leader);

nexttile;

scatter( ...
    event_times_model_leader, ...
    event_agents_model_leader, ...
    8, ...
    'filled');

grid on;

xlabel('time [s]');
ylabel('agent');

title('leader-follower');

xlim([time(1), time(end)]);
ylim([0.5, N + 0.5]);

yticks(1:N);

title(layout_model_events, ...
    sprintf( ...
    'Model-based event-triggered communication times — %s', ...
    trigger_type));

saveas( ...
    fig_model_events, ...
    fullfile(results_dir, 'undamped_model_based_trigger_times.png'));

saveas( ...
    fig_model_events, ...
    fullfile(results_dir, 'undamped_model_based_trigger_times.fig'));

%% Figure 9: outputs — ZOH vs model-based

fig_zm_outputs = figure( ...
    'Name', 'ZOH vs model-based outputs', ...
    'Position', [100, 100, 1200, 780]);

layout_zm_outputs = tiledlayout(2, 2);

layout_zm_outputs.TileSpacing = 'compact';
layout_zm_outputs.Padding = 'compact';

%% Leaderless — ZOH

nexttile;

plot( ...
    time(plot_mask), ...
    r_trig_free.y_history(:, plot_mask)', ...
    'LineWidth', 1.0);

grid on;

xlabel('time [s]');
ylabel('position');

title('leaderless — ZOH');

xlim([0, plot_end_time]);

%% Leaderless — model-based

nexttile;

plot( ...
    time(plot_mask), ...
    r_model_free.y_history(:, plot_mask)', ...
    'LineWidth', 1.0);

grid on;

xlabel('time [s]');
ylabel('position');

title('leaderless — model-based');

xlim([0, plot_end_time]);

%% Leader-follower — ZOH

nexttile;

plot( ...
    time(plot_mask), ...
    r_trig_leader.y_history(:, plot_mask)', ...
    'LineWidth', 1.0);

grid on;

xlabel('time [s]');
ylabel('position');

title('leader-follower — ZOH');

xlim([0, plot_end_time]);

%% Leader-follower — model-based

nexttile;

plot( ...
    time(plot_mask), ...
    r_model_leader.y_history(:, plot_mask)', ...
    'LineWidth', 1.0);

grid on;

xlabel('time [s]');
ylabel('position');

title('leader-follower — model-based');

xlim([0, plot_end_time]);

title(layout_zm_outputs, ...
    'Consensus outputs — ZOH vs model-based');

saveas( ...
    fig_zm_outputs, ...
    fullfile(results_dir, 'undamped_zoh_vs_model_outputs.png'));

saveas( ...
    fig_zm_outputs, ...
    fullfile(results_dir, 'undamped_zoh_vs_model_outputs.fig'));

%% Figure 10: trigger times — ZOH vs model-based

fig_zm_triggers = figure( ...
    'Name', 'ZOH vs model-based trigger times', ...
    'Position', [100, 100, 1200, 780]);

layout_zm_triggers = tiledlayout(2, 2);

layout_zm_triggers.TileSpacing = 'compact';
layout_zm_triggers.Padding = 'compact';

%% Leaderless — ZOH

[ev_agents, ev_steps] = find(r_trig_free.event_history);

nexttile;

scatter(time(ev_steps), ev_agents, 8, 'filled');

grid on;

xlabel('time [s]');
ylabel('agent');

title('leaderless — ZOH');

xlim([time(1), time(end)]);
ylim([0.5, N + 0.5]);

yticks(1:N);

%% Leaderless — model-based

[ev_agents, ev_steps] = find(r_model_free.event_history);

nexttile;

scatter(time(ev_steps), ev_agents, 8, 'filled');

grid on;

xlabel('time [s]');
ylabel('agent');

title('leaderless — model-based');

xlim([time(1), time(end)]);
ylim([0.5, N + 0.5]);

yticks(1:N);

%% Leader-follower — ZOH

[ev_agents, ev_steps] = find(r_trig_leader.event_history);

nexttile;

scatter(time(ev_steps), ev_agents, 8, 'filled');

grid on;

xlabel('time [s]');
ylabel('agent');

title('leader-follower — ZOH');

xlim([time(1), time(end)]);
ylim([0.5, N + 0.5]);

yticks(1:N);

%% Leader-follower — model-based

[ev_agents, ev_steps] = find(r_model_leader.event_history);

nexttile;

scatter(time(ev_steps), ev_agents, 8, 'filled');

grid on;

xlabel('time [s]');
ylabel('agent');

title('leader-follower — model-based');

xlim([time(1), time(end)]);
ylim([0.5, N + 0.5]);

yticks(1:N);

title(layout_zm_triggers, ...
    sprintf( ...
    'Event-triggered communication times — ZOH vs model-based (%s)', ...
    trigger_type));

saveas( ...
    fig_zm_triggers, ...
    fullfile(results_dir, 'undamped_zoh_vs_model_trigger_times.png'));

saveas( ...
    fig_zm_triggers, ...
    fullfile(results_dir, 'undamped_zoh_vs_model_trigger_times.fig'));

%% Figure 11: events per agent — ZOH vs model-based

fig_zm_events = figure( ...
    'Name', 'ZOH vs model-based events per agent', ...
    'Position', [100, 100, 1000, 520]);

layout_zm_events = tiledlayout(1, 2);

layout_zm_events.TileSpacing = 'compact';
layout_zm_events.Padding = 'compact';

%% Leaderless events per agent

nexttile;

bar(1:N, ...
    [r_trig_free.events_per_agent, r_model_free.events_per_agent]);

grid on;

xlabel('agent');
ylabel('number of events');

title(sprintf( ...
    'leaderless — ZOH = %d, model-based = %d', ...
    r_trig_free.total_events, ...
    r_model_free.total_events));

xticks(1:N);

legend('ZOH', 'model-based', 'Location', 'northeast');

%% Leader-follower events per agent

nexttile;

bar(1:N, ...
    [r_trig_leader.events_per_agent, r_model_leader.events_per_agent]);

grid on;

xlabel('agent');
ylabel('number of events');

title(sprintf( ...
    'leader-follower — ZOH = %d, model-based = %d', ...
    r_trig_leader.total_events, ...
    r_model_leader.total_events));

xticks(1:N);

legend('ZOH', 'model-based', 'Location', 'northeast');

title(layout_zm_events, ...
    'Event-triggered updates per agent — ZOH vs model-based');

saveas( ...
    fig_zm_events, ...
    fullfile(results_dir, 'undamped_zoh_vs_model_events_per_agent.png'));

saveas( ...
    fig_zm_events, ...
    fullfile(results_dir, 'undamped_zoh_vs_model_events_per_agent.fig'));