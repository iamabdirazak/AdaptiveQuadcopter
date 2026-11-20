function results = runPIDPayload()
% RUNPIDPAYLOAD Test fixed-gain PID under a payload increase.

P = quadcopterParameters();

%% Simulation settings

dt = 0.005;
t = 0:dt:30;
N = length(t);

referenceAltitude = 5;
gains = [2.5, 0.5, 3.2];

payloadTime = 15;
payloadMass = 1.44;

%% Initialize

X = zeros(12,1);
controllerState.integral = 0;

altitude = zeros(1,N);
thrustHistory = zeros(1,N);
massHistory = zeros(1,N);
reference = zeros(1,N);

%% Simulation

for k = 1:N

    % Apply payload change to the plant
    Pplant = P;

    if t(k) >= payloadTime
        Pplant.m = payloadMass;
    end

    if t(k) >= 1
        reference(k) = referenceAltitude;
    end

    % Fixed-gain controller retains nominal mass estimate
    h = -X(3);
    error = reference(k) - h;

    [thrust, controllerState] = pidAltitudeController( ...
        error, X(6), gains, controllerState, P, dt);

    rotorThrust = ones(4,1) * (thrust/4);

    % Record results
    altitude(k) = h;
    thrustHistory(k) = thrust;
    massHistory(k) = Pplant.m;

    % Integrate actual plant
    if k < N
        k1 = quadrotorDynamics(X, rotorThrust, Pplant);
        k2 = quadrotorDynamics(X + 0.5*dt*k1, rotorThrust, Pplant);
        k3 = quadrotorDynamics(X + 0.5*dt*k2, rotorThrust, Pplant);
        k4 = quadrotorDynamics(X + dt*k3, rotorThrust, Pplant);

        X = X + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    end
end

%% Results

results.time = t;
results.altitude = altitude;
results.thrust = thrustHistory;
results.reference = reference;
results.mass = massHistory;
results.payloadTime = payloadTime;

end