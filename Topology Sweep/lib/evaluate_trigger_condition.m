function [should_trigger, trigger_value, threshold] = evaluate_trigger_condition( ...
    error_vector, disagreement_vector, sigma, epsilon_trigger, n, ...
    trigger_type, trigger_params, state_vector)
% evaluate one local event-triggering condition

if nargin < 6 || isempty(trigger_type)
    trigger_type = 'relative';
end

if nargin < 7 || isempty(trigger_params)
    trigger_params = struct();
end

if nargin < 8
    state_vector = [];
end

switch lower(char(trigger_type))
    case 'absolute'
        trigger_value = norm(error_vector)^2;
        threshold = sigma + epsilon_trigger;

    case 'relative'
        trigger_value = norm(error_vector)^2;
        threshold = sigma * norm(disagreement_vector)^2 + epsilon_trigger;

    case 'state-relative'
        state_vector = require_state_vector(state_vector, n, trigger_type);
        state_gain = get_scalar_param(trigger_params, 'state_gain', 0.5);

        trigger_value = norm(error_vector)^2;
        threshold = state_gain * norm(state_vector)^2 + epsilon_trigger;

    case 'state-disagreement'
        state_vector = require_state_vector(state_vector, n, trigger_type);
        state_gain = get_scalar_param(trigger_params, 'state_gain', 0.5);
        disagreement_gain = get_scalar_param( ...
            trigger_params, 'disagreement_gain', 0.9);
        disagreement_tol = get_scalar_param( ...
            trigger_params, 'disagreement_tol', 1e-2);

        trigger_value = norm(error_vector)^2;
        threshold = state_gain * norm(state_vector)^2 + epsilon_trigger;

        if norm(disagreement_vector) <= disagreement_tol
            threshold = disagreement_gain * norm(disagreement_vector)^2 + ...
                epsilon_trigger;
        end

    case 'weighted'
        W = get_trigger_matrix(trigger_params, n);
        trigger_value = error_vector' * W * error_vector;
        threshold = sigma * ...
            (disagreement_vector' * W * disagreement_vector) + ...
            epsilon_trigger;

    case 'energy'
        W = get_energy_matrix(trigger_params, n);
        trigger_value = error_vector' * W * error_vector;
        threshold = sigma * ...
            (disagreement_vector' * W * disagreement_vector) + ...
            epsilon_trigger;

    case 'output'
        C = get_output_matrix(trigger_params, n);
        error_output = C * error_vector;
        disagreement_output = C * disagreement_vector;

        trigger_value = norm(error_output)^2;
        threshold = sigma * norm(disagreement_output)^2 + epsilon_trigger;

    case {'exponential', 'garcia_exponential'}
        [beta, lambda_trigger, t] = get_exponential_trigger_params( ...
            trigger_params, sigma);

        trigger_value = norm(error_vector, 2);
        threshold = beta * exp(-lambda_trigger * t);

    otherwise
        error('evaluate_trigger_condition:UnknownTriggerType', ...
            'Unknown trigger_type: %s.', char(trigger_type));
end

should_trigger = trigger_value >= threshold;
end


function state_vector = require_state_vector(state_vector, n, trigger_type)
% validate state-based trigger input

if isempty(state_vector)
    error('evaluate_trigger_condition:MissingStateVector', ...
        'state_vector is required for trigger_type: %s.', ...
        char(trigger_type));
end

state_vector = state_vector(:);

if numel(state_vector) ~= n
    error('evaluate_trigger_condition:BadStateVector', ...
        'state_vector must have n elements.');
end
end


function value = get_scalar_param(trigger_params, field_name, default_value)
% read an optional finite scalar trigger parameter

if isfield(trigger_params, field_name)
    value = trigger_params.(field_name);
else
    value = default_value;
end

if ~isscalar(value) || ~isfinite(value)
    error('evaluate_trigger_condition:BadScalarParam', ...
        'trigger_params.%s must be a finite scalar.', field_name);
end
end


function [beta, lambda_trigger, t] = get_exponential_trigger_params( ...
    trigger_params, sigma)
% return parameters for the Garcia exponential trigger

if isfield(trigger_params, 'beta')
    beta = trigger_params.beta;
elseif ~isempty(sigma)
    beta = sigma;
else
    error('evaluate_trigger_condition:MissingBeta', ...
        ['trigger_params.beta is required for exponential trigger ', ...
        'when sigma is empty.']);
end

if isfield(trigger_params, 'lambda')
    lambda_trigger = trigger_params.lambda;
elseif isfield(trigger_params, 'lambda_trigger')
    lambda_trigger = trigger_params.lambda_trigger;
else
    error('evaluate_trigger_condition:MissingLambda', ...
        'trigger_params.lambda is required for exponential trigger.');
end

if isfield(trigger_params, 'time')
    t = trigger_params.time;
elseif isfield(trigger_params, 't')
    t = trigger_params.t;
else
    error('evaluate_trigger_condition:MissingTime', ...
        'trigger_params.time is required for exponential trigger.');
end

if beta <= 0
    error('evaluate_trigger_condition:BadBeta', ...
        'beta must be positive.');
end

if lambda_trigger <= 0
    error('evaluate_trigger_condition:BadLambda', ...
        'lambda must be positive.');
end

if ~isscalar(t) || ~isfinite(t)
    error('evaluate_trigger_condition:BadTime', ...
        'trigger_params.time must be a finite scalar.');
end
end


function W = get_trigger_matrix(trigger_params, n)
% return a generic positive definite weighting matrix

if isfield(trigger_params, 'W')
    W = trigger_params.W;
else
    W = eye(n);
end

if ~isequal(size(W), [n, n])
    error('evaluate_trigger_condition:BadWeightMatrix', ...
        'trigger_params.W must be an n-by-n matrix.');
end
end


function W = get_energy_matrix(trigger_params, n)
% return mass-spring-damper energy weighting matrix

if n ~= 2
    error('evaluate_trigger_condition:BadEnergyDimension', ...
        'energy trigger is currently defined for n = 2.');
end

if ~isfield(trigger_params, 'mass')
    error('evaluate_trigger_condition:MissingMass', ...
        'trigger_params.mass is required for energy trigger.');
end

if ~isfield(trigger_params, 'k_spring')
    error('evaluate_trigger_condition:MissingSpringConstant', ...
        'trigger_params.k_spring is required for energy trigger.');
end

mass = trigger_params.mass;
k_spring = trigger_params.k_spring;

W = diag([k_spring, mass]);
end


function C = get_output_matrix(trigger_params, n)
% return output matrix for output-based triggering

if isfield(trigger_params, 'C')
    C = trigger_params.C;
else
    C = [1, zeros(1, n - 1)];
end

if size(C, 2) ~= n
    error('evaluate_trigger_condition:BadOutputMatrix', ...
        'trigger_params.C must have n columns.');
end
end
