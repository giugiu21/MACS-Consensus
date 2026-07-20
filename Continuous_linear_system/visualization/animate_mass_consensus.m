function animate_mass_consensus( ...
    time, y_history, agent, N, video_filename)
% Animate agent positions and optionally save an MP4 video.
%
% y_history must have dimensions:
% N x numel(time)

if nargin < 5
    video_filename = "";
end

%% Input checks

if size(y_history, 1) ~= N
    error( ...
        'animate_mass_consensus:InvalidNumberOfAgents', ...
        ['y_history must contain %d rows, one for each agent. ' ...
         'It currently contains %d rows.'], ...
        N, ...
        size(y_history, 1));
end

if size(y_history, 2) ~= numel(time)
    error( ...
        'animate_mass_consensus:InvalidTimeLength', ...
        ['y_history contains %d time samples, while time ' ...
         'contains %d samples.'], ...
        size(y_history, 2), ...
        numel(time));
end

% y_history already contains the agent positions
q_history = y_history;

%% Video settings

frame_rate = 30;
video_duration = 10; % durata massima del video [s]

% Intervallo della simulazione mostrato nel video.
% Per esempio: i primi 20 secondi di simulazione vengono mostrati
% in un video di 10 secondi.
simulation_start_time = 0;
simulation_end_time = 20;

simulation_end_time = min(simulation_end_time, time(end));

valid_indices = find( ...
    time >= simulation_start_time & ...
    time <= simulation_end_time);

number_of_frames = min( ...
    numel(valid_indices), ...
    round(frame_rate * video_duration));

frame_positions = round(linspace( ...
    1, ...
    numel(valid_indices), ...
    number_of_frames));

frame_indices = valid_indices(frame_positions);

%% Plot limits

q_min = min(q_history(:));
q_max = max(q_history(:));

q_range = q_max - q_min;
q_margin = 0.2 * max(1, q_range);

wall_x = q_min - 0.5 * q_margin;

%% Create figure

fig = figure( ...
    'Name', 'Mass-spring-damper consensus animation', ...
    'NumberTitle', 'off', ...
    'Position', [100, 100, 1000, 600]);

ax = axes('Parent', fig);

hold(ax, 'on');
grid(ax, 'on');

xlabel(ax, 'position q');
ylabel(ax, 'agent');

xlim(ax, [q_min - q_margin, q_max + q_margin]);
ylim(ax, [0.5, N + 0.5]);

yticks(ax, 1:N);

%% Create graphical objects

mass_markers = gobjects(N, 1);
spring_lines = gobjects(N, 1);

for i = 1:N

    spring_lines(i) = plot( ...
        ax, ...
        [wall_x, q_history(i, 1)], ...
        [i, i], ...
        'LineWidth', 1.0);

    mass_markers(i) = plot( ...
        ax, ...
        q_history(i, 1), ...
        i, ...
        's', ...
        'MarkerSize', 14, ...
        'MarkerFaceColor', 'auto');
end

mean_line = xline( ...
    ax, ...
    mean(q_history(:, 1)), ...
    '--', ...
    'mean position');

%% Open video writer

writer = [];

if strlength(string(video_filename)) > 0

    video_folder = fileparts(video_filename);

    if ~isempty(video_folder) && ~exist(video_folder, 'dir')
        mkdir(video_folder);
    end

    writer = VideoWriter(video_filename, 'MPEG-4');
    writer.FrameRate = frame_rate;
    writer.Quality = 95;

    open(writer);
end

%% Animation

try

    for frame_id = 1:numel(frame_indices)

        k = frame_indices(frame_id);

        q = q_history(:, k);
        q_mean = mean(q);

        % Check that the figure still exists
        if ~isgraphics(fig, 'figure') || ~isgraphics(ax, 'axes')
            error( ...
                'animate_mass_consensus:FigureClosed', ...
                ['The animation figure was closed before the video ' ...
                 'generation was completed.']);
        end

        for i = 1:N

            set( ...
                spring_lines(i), ...
                'XData', [wall_x, q(i)], ...
                'YData', [i, i]);

            set( ...
                mass_markers(i), ...
                'XData', q(i), ...
                'YData', i);
        end

        mean_line.Value = q_mean;

        title( ...
            ax, ...
            sprintf( ...
                'Mass-spring-damper consensus, t = %.2f s', ...
                time(k)));

        drawnow;

        if ~isempty(writer)

            % Capture the axes instead of relying on the current figure
            frame = getframe(ax);

            writeVideo(writer, frame);
        end
    end

catch animation_error

    if ~isempty(writer)
        close(writer);
    end

    rethrow(animation_error);
end

%% Close video correctly

if ~isempty(writer)
    close(writer);
end

end