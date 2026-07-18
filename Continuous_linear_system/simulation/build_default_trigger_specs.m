function trigger_specs = build_default_trigger_specs(agent, epsilon_trigger)
% build the standard trigger configurations used by comparison scripts

if nargin < 2
    epsilon_trigger = 1e-5;
end

common_params = struct();
common_params.W = diag([1, 0.5]);
common_params.mass = agent.mass;
common_params.k_spring = agent.k_spring;
common_params.C = agent.C;

trigger_specs = struct( ...
    'name', {}, ...
    'type', {}, ...
    'sigma', {}, ...
    'epsilon', {}, ...
    'params', {});

trigger_specs(end + 1) = make_trigger_spec( ...
    'absolute', 'absolute', 0.05, epsilon_trigger, common_params);

trigger_specs(end + 1) = make_trigger_spec( ...
    'relative', 'relative', 0.05, epsilon_trigger, common_params);

state_params = common_params;
state_params.state_gain = 0.5;
state_params.disagreement_gain = 0.3;
state_params.disagreement_tol = 1e-2;

trigger_specs(end + 1) = make_trigger_spec( ...
    'state-relative', 'state-relative', state_params.state_gain, ...
    epsilon_trigger, state_params);

trigger_specs(end + 1) = make_trigger_spec( ...
    'state-disagreement', 'state-disagreement', state_params.state_gain, ...
    epsilon_trigger, state_params);

trigger_specs(end + 1) = make_trigger_spec( ...
    'weighted', 'weighted', 0.05, epsilon_trigger, common_params);

trigger_specs(end + 1) = make_trigger_spec( ...
    'energy', 'energy', 0.05, epsilon_trigger, common_params);

trigger_specs(end + 1) = make_trigger_spec( ...
    'output', 'output', 0.05, epsilon_trigger, common_params);

exponential_params = common_params;
exponential_params.beta = 0.05;
exponential_params.lambda = 0.2;

trigger_specs(end + 1) = make_trigger_spec( ...
    'exponential', 'exponential', exponential_params.beta, 0, ...
    exponential_params);
end


function spec = make_trigger_spec(name, type, sigma, epsilon_trigger, params)
% build a trigger configuration struct

spec = struct();
spec.name = name;
spec.type = type;
spec.sigma = sigma;
spec.epsilon = epsilon_trigger;
spec.params = params;
end
