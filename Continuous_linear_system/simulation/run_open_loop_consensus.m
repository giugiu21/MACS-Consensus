function result = run_open_loop_consensus( ...
    x0, agent, graph, time, dt)
%RUN_OPEN_LOOP_AGENTS
% Simulate N independent agents without control and without communication.
%
% For every agent:
%
%   x_i_dot = A x_i
%   u_i = 0
%
% This case provides a baseline for comparison with continuous and
% event-triggered consensus.

    %% Dimensions

    N = graph.N;
    n = agent.n;
    num_steps = numel(time);

    A_global = kron(eye(N), agent.A);
    C_global = kron(eye(N), agent.C);

    %% Validate initial condition

    x = x0(:);

    if numel(x) ~= N * n
        error( ...
            'run_open_loop_agents:BadInitialCondition', ...
            'x0 must contain N * agent.n elements.');
    end

    %% Preallocate histories

    x_history = zeros(N * n, num_steps);
    y_history = zeros(N, num_steps);

    output_disagreement = zeros(1, num_steps);

    % No communication and no triggering
    communication_history = false(N, num_steps);
    event_history = false(N, num_steps);

    %% Simulation loop

    for step_id = 1:num_steps

        %% Save current state

        x_history(:, step_id) = x;

        %% Compute outputs

        y = C_global * x;

        y_history(:, step_id) = y;

        %% Output disagreement

        output_disagreement(step_id) = ...
            norm(y - mean(y) * ones(N, 1));

        %% Open-loop propagation

        if step_id < num_steps

            dx = A_global * x;

            % Forward Euler integration
            x = x + dt * dx;
        end
    end

    %% Result structure

    result = struct();

    result.time = time;

    result.x_history = x_history;
    result.y_history = y_history;
    result.output_history = y_history;

    result.output_disagreement = output_disagreement;

    result.communication_history = communication_history;
    result.event_history = event_history;

    result.communications_per_agent = zeros(N, 1);
    result.total_communications = 0;

    result.final_state = x;
    result.final_output_disagreement = ...
        output_disagreement(end);

    result.communication_mode = "none";
    result.control_case = "open_loop";
end