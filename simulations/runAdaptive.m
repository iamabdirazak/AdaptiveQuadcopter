function results = runAdaptive(useAdaptation)

rng(42);

if nargin < 1
    useAdaptation = true;
end

% RUNADAPTIVE Simulate altitude control with online mass estimation.

P = quadcopterParameters();

%% Simulation settings

dt = 0.005;
t = 0:dt:30;
N = length(t);

referenceAltitude = 5;
gains = [2.5, 0.5, 3.2];

payloadTime = 15;
payloadMass = 1.44;

%% Initialize states

X = zeros(12,1);

controllerState.integral = 0;

estimatorState.theta = 1/P.m;
estimatorState.covariance = 100;
estimatorState.forgettingFactor = 0.995;

%% Initialize result arrays

altitude = zeros(1,N);
thrustHistory = zeros(1,N);
massEstimateHistory = zeros(1,N);
actualMassHistory = zeros(1,N);
reference = zeros(1,N);

%% Simulation loop

for k = 1:N

    % Actual plant mass
    Pplant = P;

    if t(k) >= payloadTime
        Pplant.m = payloadMass;
    end

    % Altitude reference
    if t(k) >= 1
        reference(k) = referenceAltitude;
    end

    % Current altitude and tracking error
    h = -X(3);
    error = reference(k) - h;

    % Adaptive controller
    if useAdaptation
    massEstimate = 1/estimatorState.theta;
    else
        massEstimate = P.m;
    end

    [thrust, controllerState] = adaptiveAltitudeController( ...
        error, X(6), gains, controllerState, ...
        massEstimate, P, dt);

    rotorThrust = ones(4,1) * (thrust/4);

    % Measure ideal vertical acceleration from the plant model
    Xdot = quadrotorDynamics(X, rotorThrust, Pplant);
    % Simulated vertical acceleration sensor noise
accelerationNoiseStd = 0.15;  % m/s^2

measuredAcceleration = -Xdot(6) + ...
    accelerationNoiseStd * randn();

verticalAcceleration = measuredAcceleration;

    % Update mass estimate
    if useAdaptation
    [massEstimate, estimatorState] = massEstimatorRLS( ...
        thrust, verticalAcceleration, estimatorState, P);
    end

    % Record results
    altitude(k) = h;
    thrustHistory(k) = thrust;
    massEstimateHistory(k) = massEstimate;
    actualMassHistory(k) = Pplant.m;

    % Integrate plant using RK4
    if k < N

        k1 = quadrotorDynamics(X, rotorThrust, Pplant);

        k2 = quadrotorDynamics( ...
            X + 0.5*dt*k1, rotorThrust, Pplant);

        k3 = quadrotorDynamics( ...
            X + 0.5*dt*k2, rotorThrust, Pplant);

        k4 = quadrotorDynamics( ...
            X + dt*k3, rotorThrust, Pplant);

        X = X + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    end
end

%% Return results

results.time = t;
results.altitude = altitude;
results.thrust = thrustHistory;
results.massEstimate = massEstimateHistory;
results.actualMass = actualMassHistory;
results.reference = reference;

end