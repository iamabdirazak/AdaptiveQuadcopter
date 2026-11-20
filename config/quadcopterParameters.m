function P = quadcopterParameters()
% QUADCOPTERPARAMETERS Physical parameters of the quadcopter.

% Physical constants
P.m = 1.2;             % Mass (kg)
P.g = 9.81;            % Gravity (m/s^2)

% Geometry
P.L = 0.22;            % Arm length (m)
P.a = P.L / sqrt(2);   % X-configuration arm projection

% Inertia matrix (kg m^2)
P.J = diag([0.015, 0.015, 0.025]);

% Rotor characteristics
P.kf = 1e-5;           % Thrust coefficient
P.kappa = 0.015;       % Moment-to-thrust ratio

% Control allocation matrix
P.M = [
    1,       1,       1,       1;
   -P.a,    -P.a,     P.a,     P.a;
    P.a,    -P.a,    -P.a,     P.a;
    P.kappa,-P.kappa, P.kappa,-P.kappa
];

end