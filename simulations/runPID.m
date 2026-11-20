function results = runPID()
% RUNPID Simulate altitude control using fixed-gain PID.

P = quadcopterParameters();

%% Simulation settings

dt = 0.005;
t = 0:dt:15;
N = length(t);

referenceAltitude = 5;
gains = [2.5, 0.5, 3.2];

%% Initialize states

X = zeros(12,1);

controllerState.integral = 0;

altitude = zeros(1,N);
thrustHistory = zeros(1,N);
reference = zeros(1,N);

%% Simulation loop

for k = 1:N

    if t(k) >= 1
        reference(k) = referenceAltitude;
    end

    % Altitude feedback
    h = -X(3);
    error = reference(k) - h;

    % PID controller
    [thrust, controllerState] = pidAltitudeController( ...
        error, X(6), gains, controllerState, P, dt);

    rotorThrust = ones(4,1) * (thrust/4);

    % Record results
    altitude(k) = h;
    thrustHistory(k) = thrust;

    % Integrate plant using RK4
    if k < N
        k1 = quadrotorDynamics(X, rotorThrust, P);
        k2 = quadrotorDynamics(X + 0.5*dt*k1, rotorThrust, P);
        k3 = quadrotorDynamics(X + 0.5*dt*k2, rotorThrust, P);
        k4 = quadrotorDynamics(X + dt*k3, rotorThrust, P);

        X = X + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    end
end

%% Return simulation results

results.time = t;
results.altitude = altitude;
results.thrust = thrustHistory;
results.reference = reference;
results.gains = gains;

end