function validateDynamics()
% VALIDATEDYNAMICS Verify fundamental quadcopter physics.

P = quadcopterParameters();

X = zeros(12,1);

%% Test 1: Hover equilibrium

hoverThrust = ones(4,1) * (P.m * P.g / 4);

Xdot = quadrotorDynamics(X, hoverThrust, P);

assert(all(abs(Xdot) < 1e-10), ...
    'Hover equilibrium test failed.');

fprintf('PASS: Hover equilibrium\n');

%% Test 2: Free fall

zeroThrust = zeros(4,1);

Xdot = quadrotorDynamics(X, zeroThrust, P);

assert(abs(Xdot(6) - P.g) < 1e-10, ...
    'Free-fall test failed.');

fprintf('PASS: Free-fall acceleration\n');

%% Test 3: Vertical acceleration

highThrust = hoverThrust * 1.2;

Xdot = quadrotorDynamics(X, highThrust, P);

expectedAcceleration = -0.2 * P.g;

assert(abs(Xdot(6) - expectedAcceleration) < 1e-10, ...
    'Vertical acceleration test failed.');

fprintf('PASS: Vertical thrust response\n');

%% Test 4: Roll moment

testThrust = hoverThrust;
testThrust(1) = testThrust(1) + 0.5;

Xdot = quadrotorDynamics(X, testThrust, P);

assert(abs(Xdot(10)) > 0, ...
    'Roll moment test failed.');

fprintf('PASS: Roll moment response\n');

%% Test 5: Pitch moment

testThrust = hoverThrust;
testThrust(2) = testThrust(2) + 0.5;

Xdot = quadrotorDynamics(X, testThrust, P);

assert(abs(Xdot(11)) > 0, ...
    'Pitch moment test failed.');

fprintf('PASS: Pitch moment response\n');

%% Test 6: Yaw moment

testThrust = hoverThrust;
testThrust(1) = testThrust(1) + 0.5;

Xdot = quadrotorDynamics(X, testThrust, P);

assert(abs(Xdot(12)) > 0, ...
    'Yaw moment test failed.');

fprintf('PASS: Yaw moment response\n');

fprintf('\nAll dynamics validation tests passed.\n');

end