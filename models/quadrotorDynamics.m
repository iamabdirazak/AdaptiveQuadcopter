function Xdot = quadrotorDynamics(X, rotorThrust, P)
% QUADROTORDYNAMICS 6-DOF rigid-body quadcopter dynamics.
%
% State:
% X = [x y z vx vy vz phi theta psi p q r]'
%
% Frames:
% Inertial: North-East-Down
% Body: Forward-Right-Down

velocity = X(4:6);

phi   = X(7);
theta = X(8);
psi   = X(9);

omega = X(10:12);

% Body-to-NED rotation matrix
R = [
    cos(theta)*cos(psi), ...
    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
    cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);

    cos(theta)*sin(psi), ...
    sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
    cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);

    -sin(theta), ...
    sin(phi)*cos(theta), ...
    cos(phi)*cos(theta)
];

% Translational dynamics
T = sum(rotorThrust);

acceleration = [0; 0; P.g] + ...
    R*[0; 0; -T]/P.m;

% Rotational dynamics
control = P.M * rotorThrust;
tau = control(2:4);

omegaDot = P.J \ ...
    (tau - cross(omega, P.J*omega));

% Euler angle kinematics
E = [
    1, sin(phi)*tan(theta), cos(phi)*tan(theta);
    0, cos(phi),            -sin(phi);
    0, sin(phi)/cos(theta), cos(phi)/cos(theta)
];

angleRates = E*omega;

% Assemble state derivative
Xdot = [
    velocity;
    acceleration;
    angleRates;
    omegaDot
];

end