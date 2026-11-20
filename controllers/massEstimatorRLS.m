function [massEstimate, state] = massEstimatorRLS( ...
    thrust, verticalAcceleration, state, P)
% MASSESTIMATORRLS Estimate vehicle mass using recursive least squares.

% Measurement model:
% verticalAcceleration + g = (1/mass) * thrust

phi = thrust;
y = verticalAcceleration + P.g;

% Skip invalid or insufficient excitation
if phi <= 1e-6
    massEstimate = 1 / state.theta;
    return;
end

% RLS gain
K = state.covariance * phi / ...
    (state.forgettingFactor + phi^2 * state.covariance);

% Prediction error
predictionError = y - phi * state.theta;

% Update inverse-mass estimate
state.theta = state.theta + K * predictionError;

% Update covariance
state.covariance = ...
    (state.covariance - K * phi * state.covariance) ...
    / state.forgettingFactor;

% Bound estimated mass between 0.5 and 3 kg
state.theta = min(max(state.theta, 1/3), 1/0.5);

% Return mass estimate
massEstimate = 1 / state.theta;

end