%COMPARE_damped_TRIGGER_THRESHOLDS
% Compare the communication/performance trade-off of the four selected
% trigger types: absolute, relative, state-relative and state-disagreement.
%
% The experiment keeps the plant, graph, initial condition, time horizon,
% controller, hold mode and minimum inter-event time fixed. Only the trigger
% rule and its parameters change. Results are written to:
%
%   results/damped_trigger_threshold_comparison/

clear;
clc;
close all;

%% Path and output setup

script_dir = fileparts(mfilename('fullpath'));

addpath(script_dir);
add_matlab_paths();

results_dir = fullfile( ...
    script_dir, ...
    'results', ...
    'damped_trigger_threshold_comparison');

if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% Experiment configuration

dt = 1e-3;
T = 40;
time = 0:dt:T;

N = 5;
seed = 1;
leader_id = 1;

% Use "model_based" to compare the thresholds with the Garcia-style model
% propagation, or "zoh" to compare them with sampled states held constant.
hold_mode = "zoh";

% Evaluate both architectures using exactly the same trigger configurations.
control_cases = ["leaderless", "leader_follower"];

% Set this to zero to isolate the effect of the threshold. A positive value
% imposes the same communication dead time on every trigger rule.
minimum_inter_event_time = 0;

epsilon_trigger = 1e-2;

% Threshold coefficient used by the absolute and relative trigger rules.
% Decreasing sigma makes their thresholds smaller and communications more
% frequent; increasing it has the opposite effect.
sigma = 0.8;

%% Common plant, graph, controller and initial condition

agent = init_damped_agent();
graph = build_consensus_graph( ...
    N, ...
    'path', ...
    struct('show_plot', false));

K_consensus = compute_lqr_consensus_gain(agent);
x0 = make_random_initial_condition(seed, N, agent.n);

%% Trigger configurations

all_trigger_specs = build_default_trigger_specs( ...
    agent, ...
    epsilon_trigger, ...
    dt, ...
    'damped', ...
    minimum_inter_event_time);

selected_trigger_types = [ ...
    "absolute", ...
    "relative", ...
    "state-relative", ...
    "state-disagreement"];

all_trigger_types = string({all_trigger_specs.type});
selected_mask = ismember(all_trigger_types, selected_trigger_types);
trigger_specs = all_trigger_specs(selected_mask);

if numel(trigger_specs) ~= numel(selected_trigger_types)
    error( ...
        'compare_damped_trigger_thresholds:MissingTrigger', ...
        'One or more selected trigger types are not available.');
end

% Use the same damped tuning already adopted in main_damped.m for the
% two state-dependent rules. Parameters for the selected rules are defined in
% build_default_trigger_specs.m and can be tuned there or below.
for spec_id = 1:numel(trigger_specs)

    trigger_key = string(trigger_specs(spec_id).type);

    if trigger_key == "absolute" || trigger_key == "relative"

        trigger_specs(spec_id).sigma = sigma;

    elseif trigger_key == "state-relative" || ...
            trigger_key == "state-disagreement"

        trigger_specs(spec_id).params.state_gain = 0.5;
        trigger_specs(spec_id).params.disagreement_gain = 0.4;
        trigger_specs(spec_id).params.disagreement_tol = 1e-1;
    end
end

%% Preallocate comparison data

num_specs = numel(trigger_specs);
num_cases = numel(control_cases);
num_runs = num_specs * num_cases;
num_steps = numel(time);

trigger_name = strings(num_runs, 1);
control_case_column = strings(num_runs, 1);
hold_mode_column = repmat(hold_mode, num_runs, 1);

total_events = zeros(num_runs, 1);
mean_events_per_agent = zeros(num_runs, 1);
communication_rate_percent = zeros(num_runs, 1);
mean_inter_event_time = nan(num_runs, 1);
final_output_disagreement = zeros(num_runs, 1);
final_state_disagreement = zeros(num_runs, 1);
rms_output_disagreement = zeros(num_runs, 1);

events_per_agent = zeros(num_specs, N, num_cases);
cumulative_events = zeros(num_specs, num_steps, num_cases);

%% Run every selected trigger with identical experiment data

row_id = 0;

fprintf('\n');
fprintf('============================================================\n');
fprintf('damped TRIGGER THRESHOLD COMPARISON\n');
fprintf('hold mode: %s | T: %.1f s | dt: %.4f s | seed: %d\n', ...
    char(hold_mode), T, dt, seed);
fprintf('============================================================\n');

for case_id = 1:num_cases

    control_case = control_cases(case_id);

    for spec_id = 1:num_specs

        row_id = row_id + 1;
        spec = trigger_specs(spec_id);

        opts = struct();
        opts.communication_mode = "event_triggered";
        opts.control_case = control_case;
        opts.system = "equilibrium";
        opts.leader_id = leader_id;
        opts.hold_mode = hold_mode;
        opts.sigma = spec.sigma;
        opts.epsilon_trigger = spec.epsilon;
        opts.trigger_type = string(spec.type);
        opts.trigger_params = spec.params;

        fprintf( ...
            'Running %-17s | %-15s ... ', ...
            char(control_case), ...
            char(string(spec.name)));

        result = run_event_triggered_consensus( ...
            x0, ...
            agent, ...
            graph, ...
            K_consensus, ...
            time, ...
            dt, ...
            opts);

        trigger_name(row_id) = string(spec.name);
        control_case_column(row_id) = control_case;

        total_events(row_id) = result.total_events;
        mean_events_per_agent(row_id) = mean(result.events_per_agent);
        communication_rate_percent(row_id) = ...
            100 * mean(result.event_history(:));
        mean_inter_event_time(row_id) = compute_mean_inter_event_time( ...
            result.event_history, ...
            time);

        final_output_disagreement(row_id) = ...
            result.final_output_disagreement;

        final_state_disagreement(row_id) = compute_state_disagreement( ...
            result.final_state, ...
            agent.n, ...
            N);

        rms_output_disagreement(row_id) = ...
            sqrt(mean(result.output_disagreement .^ 2));

        events_per_agent(spec_id, :, case_id) = ...
            result.events_per_agent(:)';

        cumulative_events(spec_id, :, case_id) = ...
            sum(cumsum(result.event_history, 2), 1);

        fprintf( ...
            '%6d events | final output disagreement %.3e\n', ...
            result.total_events, ...
            result.final_output_disagreement);

        clear result;
    end
end

%% Summary table

comparison_table = table( ...
    control_case_column, ...
    trigger_name, ...
    hold_mode_column, ...
    total_events, ...
    mean_events_per_agent, ...
    communication_rate_percent, ...
    mean_inter_event_time, ...
    final_output_disagreement, ...
    final_state_disagreement, ...
    rms_output_disagreement, ...
    'VariableNames', { ...
        'ControlCase', ...
        'Trigger', ...
        'HoldMode', ...
        'TotalEvents', ...
        'MeanEventsPerAgent', ...
        'CommunicationRatePercent', ...
        'MeanInterEventTime', ...
        'FinalOutputDisagreement', ...
        'FinalStateDisagreement', ...
        'RmsOutputDisagreement'});

comparison_table = sortrows( ...
    comparison_table, ...
    {'ControlCase', 'TotalEvents'}, ...
    {'ascend', 'ascend'});

fprintf('\nSorted comparison (fewer events first):\n');
disp(comparison_table);

writetable( ...
    comparison_table, ...
    fullfile(results_dir, 'damped_trigger_threshold_comparison.csv'));

save( ...
    fullfile(results_dir, 'damped_trigger_threshold_comparison.mat'), ...
    'comparison_table', ...
    'trigger_specs', ...
    'control_cases', ...
    'hold_mode', ...
    'time', ...
    'events_per_agent', ...
    'cumulative_events', ...
    'x0');

%% Figure 1: total communications and final disagreement

figure_summary = figure( ...
    'Name', 'damped trigger threshold comparison', ...
    'Position', [100, 100, 1300, 760]);

layout_summary = tiledlayout(num_cases, 2);
layout_summary.TileSpacing = 'compact';
layout_summary.Padding = 'compact';

spec_names = string({trigger_specs.name});

for case_id = 1:num_cases

    row_mask = control_case_column == control_cases(case_id);
    case_events = total_events(row_mask);
    case_final_disagreement = final_state_disagreement(row_mask);

    nexttile;
    bar(case_events);
    grid on;
    ylabel('total events');
    title(sprintf('%s: communications', char(control_cases(case_id))));
    xticks(1:num_specs);
    xticklabels(spec_names);
    xtickangle(30);

    nexttile;
    semilogy( ...
        max(case_final_disagreement, eps), ...
        'o-', ...
        'LineWidth', 1.2, ...
        'MarkerSize', 6);
    grid on;
    ylabel('final state disagreement');
    title(sprintf('%s: consensus quality', char(control_cases(case_id))));
    xticks(1:num_specs);
    xticklabels(spec_names);
    xtickangle(30);
end

title( ...
    layout_summary, ...
    sprintf( ...
        'damped trigger comparison - %s hold', ...
        char(hold_mode)));

saveas( ...
    figure_summary, ...
    fullfile(results_dir, 'damped_trigger_threshold_summary.png'));

savefig( ...
    figure_summary, ...
    fullfile(results_dir, 'damped_trigger_threshold_summary.fig'));

%% Figure 2: cumulative communications

figure_cumulative = figure( ...
    'Name', 'damped cumulative communications by trigger', ...
    'Position', [100, 100, 1200, 720]);

layout_cumulative = tiledlayout(num_cases, 1);
layout_cumulative.TileSpacing = 'compact';
layout_cumulative.Padding = 'compact';

for case_id = 1:num_cases

    nexttile;
    plot( ...
        time, ...
        squeeze(cumulative_events(:, :, case_id))', ...
        'LineWidth', 1.1);
    grid on;
    xlabel('time [s]');
    ylabel('cumulative events');
    title(char(control_cases(case_id)));
    legend(spec_names, 'Location', 'eastoutside');
end

title( ...
    layout_cumulative, ...
    sprintf( ...
        'Cumulative communications - damped, %s hold', ...
        char(hold_mode)));

saveas( ...
    figure_cumulative, ...
    fullfile(results_dir, 'damped_trigger_cumulative_events.png'));

savefig( ...
    figure_cumulative, ...
    fullfile(results_dir, 'damped_trigger_cumulative_events.fig'));

fprintf('\nResults saved in:\n  %s\n', results_dir);


function mean_interval = compute_mean_inter_event_time(event_history, time)
% Return the mean interval between consecutive events over all agents.

    intervals = [];

    for agent_id = 1:size(event_history, 1)

        event_times = time(logical(event_history(agent_id, :)));

        if numel(event_times) >= 2
            intervals = [intervals, diff(event_times)]; %#ok<AGROW>
        end
    end

    if isempty(intervals)
        mean_interval = NaN;
    else
        mean_interval = mean(intervals);
    end
end


function disagreement = compute_state_disagreement(state, n, N)
% Return the Frobenius norm of deviations from the network mean state.

    state_matrix = reshape(state, n, N);
    mean_state = mean(state_matrix, 2);
    disagreement = norm(state_matrix - mean_state, 'fro');
end
