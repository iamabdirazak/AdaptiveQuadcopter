
%% ADAPTIVE QUADCOPTER FLIGHT CONTROL
% Phase 01: Mathematical Model and Plant Validation
% Model: 6-DOF rigid-body quadcopter
% Coordinates: NED inertial frame, Forward-Right-Down body frame

clear;
clc;
close all;

%% 1. PHYSICAL PARAMETERS

P.m = 1.2;                  % Mass (kg)
P.g = 9.81;                 % Gravity (m/s^2)
P.L = 0.22;                 % Centre-to-rotor distance (m)

P.J = diag([0.015, ...
            0.015, ...
            0.025]);       % Inertia matrix (kg.m^2)

P.kf = 1.0e-5;              % Illustrative thrust coefficient
P.kappa = 0.015;            % Illustrative torque/thrust ratio

P.a = P.L / sqrt(2);

%% 2. ROTOR CONTROL ALLOCATION MATRIX

P.M = [1,      1,      1,      1;
      -P.a,   -P.a,    P.a,    P.a;
       P.a,   -P.a,   -P.a,    P.a;
       P.kappa, -P.kappa, P.kappa, -P.kappa];

%% 3. INITIAL VEHICLE STATE

% State:
% [x y z vx vy vz phi theta psi p q r]

X0 = zeros(12,1);

%% 4. HOVER EQUILIBRIUM TEST

total_thrust = P.m * P.g;

rotor_thrust = total_thrust / 4;

motor_thrusts = rotor_thrust * ones(4,1);

control_inputs = P.M * motor_thrusts;

%% 5. EVALUATE VEHICLE DYNAMICS

Xdot = quadrotorDynamics(X0, motor_thrusts, P);

%% 6. VALIDATION RESULTS

fprintf('\nQUADCOPTER PLANT VALIDATION\n');
fprintf('----------------------------\n');

fprintf('Vehicle mass: %.3f kg\n', P.m);
fprintf('Total hover thrust: %.4f N\n', total_thrust);

fprintf('\nIndividual rotor thrusts:\n');
disp(motor_thrusts);

fprintf('Control allocation output:\n');
fprintf('Collective thrust: %.4f N\n', control_inputs(1));
fprintf('Roll moment:       %.6f Nm\n', control_inputs(2));
fprintf('Pitch moment:      %.6f Nm\n', control_inputs(3));
fprintf('Yaw moment:        %.6f Nm\n', control_inputs(4));

fprintf('\nState derivatives at hover:\n');
disp(Xdot);

%% 7. ASSERTIONS

tolerance = 1e-10;

assert(abs(control_inputs(1) - total_thrust) < tolerance);
assert(norm(control_inputs(2:4)) < tolerance);
assert(norm(Xdot) < tolerance);

fprintf('\nSUCCESS: Hover equilibrium validated.\n');


%% PHASE 02: CLOSED-LOOP PID ALTITUDE CONTROL

fprintf('\nPID ALTITUDE CONTROL SIMULATION\n');

%% 1. SIMULATION PARAMETERS

dt = 0.005;
tEnd = 15;

t_pid = 0:dt:tEnd;
N = length(t_pid);

X_pid = zeros(12,N);

% Initial condition
X_pid(:,1) = zeros(12,1);

% Desired altitude
h_ref = 5;

%% 2. PID PARAMETERS

Kp = 2.5;
Ki = 0.5;
Kd = 3.2;

integral_error = 0;

% Actuator limits
T_min = 0;
T_max = 2.5 * P.m * P.g;

%% 3. DATA STORAGE

altitude = zeros(1,N);
altitude_reference = zeros(1,N);
vertical_velocity = zeros(1,N);
thrust_history = zeros(1,N);
error_history = zeros(1,N);

%% 4. CLOSED-LOOP SIMULATION

for k = 1:N-1

    t = t_pid(k);

    % Current altitude and velocity
    h = -X_pid(3,k);
    vz = X_pid(6,k);

    % Reference command
    if t >= 1
        reference = h_ref;
    else
        reference = 0;
    end

    % Tracking error
    error = reference - h;

    % Derivative of altitude error
    error_derivative = vz;

    % PID acceleration command
    a_cmd = Kp*error + ...
            Ki*integral_error + ...
            Kd*error_derivative;

    % Convert acceleration into collective thrust
    T_raw = P.m*(P.g + a_cmd);

    % Apply actuator saturation
    T = min(max(T_raw,T_min),T_max);

    % Conditional integration anti-windup
    if T == T_raw || ...
       (T >= T_max && error < 0) || ...
       (T <= T_min && error > 0)

        integral_error = integral_error + error*dt;

    end

    % Four equal rotor thrust commands
    rotor_thrust = (T/4)*ones(4,1);

    % Store measurements
    altitude(k) = h;
    altitude_reference(k) = reference;
    vertical_velocity(k) = -vz;
    thrust_history(k) = T;
    error_history(k) = error;

    % RK4 numerical integration
    f = @(state) quadrotorDynamics( ...
        state,rotor_thrust,P);

    x = X_pid(:,k);

    k1 = f(x);
    k2 = f(x + 0.5*dt*k1);
    k3 = f(x + 0.5*dt*k2);
    k4 = f(x + dt*k3);

    X_pid(:,k+1) = x + ...
        (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

end

% Store final measurements
altitude(N) = -X_pid(3,N);
altitude_reference(N) = h_ref;
vertical_velocity(N) = -X_pid(6,N);
thrust_history(N) = thrust_history(N-1);
error_history(N) = h_ref-altitude(N);

%% 5. ALTITUDE TRACKING

figure('Name','PID Altitude Tracking');

plot(t_pid,altitude_reference,'--','LineWidth',1.5);
hold on;
plot(t_pid,altitude,'LineWidth',1.8);

xlabel('Time (s)');
ylabel('Altitude (m)');
legend('Reference','Actual altitude');
title('Closed-Loop PID Altitude Tracking');
grid on;

%% 6. VERTICAL VELOCITY

figure('Name','Vertical Velocity');

plot(t_pid,vertical_velocity,'LineWidth',1.5);

xlabel('Time (s)');
ylabel('Vertical velocity (m/s)');
title('Vertical Velocity Response');
grid on;

%% 7. CONTROL EFFORT

figure('Name','Collective Thrust');

plot(t_pid,thrust_history,'LineWidth',1.5);
hold on;

yline(P.m*P.g,'--','Hover thrust');

xlabel('Time (s)');
ylabel('Total thrust (N)');
title('PID Controller Thrust Command');
grid on;

%% 8. TRACKING ERROR

figure('Name','Altitude Tracking Error');

plot(t_pid,error_history,'LineWidth',1.5);

xlabel('Time (s)');
ylabel('Error (m)');
title('Altitude Tracking Error');
grid on;

%% 9. PERFORMANCE SUMMARY

fprintf('\nPID PERFORMANCE SUMMARY\n');
fprintf('-----------------------\n');

fprintf('Target altitude: %.2f m\n',h_ref);
fprintf('Final altitude: %.4f m\n',altitude(end));

fprintf('Final tracking error: %.4f m\n',error_history(end));

fprintf('Maximum altitude: %.4f m\n',max(altitude));

fprintf('Maximum thrust: %.4f N\n',max(thrust_history));

fprintf('Final vertical velocity: %.4f m/s\n', ...
    vertical_velocity(end));

fprintf('\nPID simulation completed.\n');


%% PHASE 02.2: PID ROBUSTNESS ANALYSIS

fprintf('\nPID ROBUSTNESS ANALYSIS\n');

dt_test = 0.005;
duration = 15;

%% EXPERIMENT A: PID GAIN SENSITIVITY

gainSets = [
    1.5, 0.5, 2.0;
    2.5, 0.5, 3.2;
    4.0, 0.5, 5.0
];

gainNames = {'Conservative','Original','Aggressive'};

figure('Name','PID Gain Sensitivity');
hold on;

for i = 1:size(gainSets,1)

    [t,h,T,e] = runPIDTest( ...
        gainSets(i,1), ...
        gainSets(i,2), ...
        gainSets(i,3), ...
        inf, P.m, P, dt_test, duration);

    plot(t,h,'LineWidth',1.5);

    fprintf('\n%s controller:\n',gainNames{i});
    fprintf('Final altitude: %.4f m\n',h(end));
    fprintf('Maximum altitude: %.4f m\n',max(h));

end

yline(5,'--','Target altitude');

xlabel('Time (s)');
ylabel('Altitude (m)');
title('PID Gain Sensitivity');
legend(gainNames{:},'Target');
grid on;


%% EXPERIMENT B: PAYLOAD CHANGE

massChangeTime = 7;

newMass = 1.20 * P.m;

[t_nom,h_nom,T_nom,e_nom] = runPIDTest( ...
    2.5,0.5,3.2,inf,P.m,P,dt_test,duration);

[t_payload,h_payload,T_payload,e_payload] = runPIDTest( ...
    2.5,0.5,3.2,massChangeTime,newMass,P,dt_test,duration);


%% EXPERIMENT C: COMPARE RECOVERY

figure('Name','Payload Robustness');

plot(t_nom,h_nom,'--','LineWidth',1.5);
hold on;

plot(t_payload,h_payload,'LineWidth',1.8);

yline(5,':','Target altitude');

xline(massChangeTime,'-.','Payload added');

xlabel('Time (s)');
ylabel('Altitude (m)');
title('PID Response to Payload Variation');

legend('Nominal mass','20% payload increase','Target');
grid on;


%% CONTROL EFFORT COMPARISON

figure('Name','Payload Control Effort');

plot(t_nom,T_nom,'--','LineWidth',1.5);
hold on;

plot(t_payload,T_payload,'LineWidth',1.8);

yline(P.m*P.g,':','Nominal hover thrust');

xlabel('Time (s)');
ylabel('Collective thrust (N)');
title('Thrust Response to Payload Change');

legend('Nominal','Payload increased');
grid on;


%% TRACKING ERROR COMPARISON

figure('Name','Payload Tracking Error');

plot(t_nom,e_nom,'--','LineWidth',1.5);
hold on;

plot(t_payload,e_payload,'LineWidth',1.8);

xline(massChangeTime,'-.','Payload added');

xlabel('Time (s)');
ylabel('Altitude error (m)');
title('Tracking Error Under Parameter Variation');

legend('Nominal','Payload increased');
grid on;


%% PERFORMANCE SUMMARY

fprintf('\nPAYLOAD ROBUSTNESS RESULTS\n');
fprintf('--------------------------\n');

fprintf('Original mass: %.3f kg\n',P.m);
fprintf('New mass: %.3f kg\n',newMass);

fprintf('Nominal final altitude: %.4f m\n',h_nom(end));
fprintf('Payload final altitude: %.4f m\n',h_payload(end));

fprintf('Nominal final error: %.4f m\n',e_nom(end));
fprintf('Payload final error: %.4f m\n',e_payload(end));

fprintf('Payload maximum thrust: %.4f N\n',max(T_payload));

fprintf('\nRobustness experiments completed.\n');


%% PHASE 03: INDIRECT ADAPTIVE FLIGHT CONTROL
% Reference model + online mass estimation

fprintf('\nADAPTIVE FLIGHT CONTROL SIMULATION\n');

%% 1. SIMULATION CONFIGURATION

dt_ad = 0.005;
tEnd_ad = 15;

t_ad = 0:dt_ad:tEnd_ad;
N_ad = length(t_ad);

X_ad = zeros(12,N_ad);

%% 2. PLANT UNCERTAINTY

massChangeTime = 7;

actualMassBefore = P.m;
actualMassAfter = 1.20*P.m;

% Controller's initial mass estimate
massEstimate = P.m;

% Parameter estimation variable: theta = 1/m
thetaHat = 1/massEstimate;

% Projection bounds
thetaMin = 1/(1.5*P.m);
thetaMax = 1/(0.7*P.m);

% Adaptation rate
gamma = 5;

%% 3. REFERENCE MODEL

wn = 1.5;
zeta = 1.0;

hModel = 0;
vModel = 0;

%% 4. CONTROLLER PARAMETERS

controllerWn = 2.0;
controllerZeta = 1.0;

Kp_ad = controllerWn^2;
Kd_ad = 2*controllerZeta*controllerWn;

% Physical thrust limit
Tmax_ad = 2.5*P.m*P.g;

%% 5. DATA STORAGE

h_ad = zeros(1,N_ad);
href_ad = zeros(1,N_ad);

massEstimateHistory = zeros(1,N_ad);
actualMassHistory = zeros(1,N_ad);

thrust_ad = zeros(1,N_ad);
error_ad = zeros(1,N_ad);

thetaHistory = zeros(1,N_ad);

previousVelocity = 0;
previousThrust = P.m*P.g;

%% 6. ADAPTIVE SIMULATION LOOP

for k = 1:N_ad-1

    time = t_ad(k);

    % Actual plant mass
    if time >= massChangeTime
        actualMass = actualMassAfter;
    else
        actualMass = actualMassBefore;
    end

    Pplant = P;
    Pplant.m = actualMass;

    % Actual altitude and upward velocity
    h = -X_ad(3,k);
    v = -X_ad(6,k);

    % Desired altitude command
    if time >= 1
        reference = 5;
    else
        reference = 0;
    end

    %% A. ONLINE MASS ESTIMATION

    % Estimate acceleration using measured velocity
    if k > 1

        measuredAcceleration = ...
            (v-previousVelocity)/dt_ad;

        measuredOutput = measuredAcceleration + P.g;

        predictionError = ...
            measuredOutput-thetaHat*previousThrust;

        % Normalized gradient adaptation
        thetaDot = gamma * ...
            previousThrust/(1+previousThrust^2) * ...
            predictionError;

        thetaCandidate = thetaHat + thetaDot*dt_ad;

        % Parameter projection
        thetaHat = min(max(thetaCandidate, ...
            thetaMin),thetaMax);

    end

    massEstimate = 1/thetaHat;

    %% B. REFERENCE MODEL

    modelAcceleration = ...
        wn^2*(reference-hModel) ...
        - 2*zeta*wn*vModel;

    % Reference model integration
    hModel = hModel + vModel*dt_ad;
    vModel = vModel + modelAcceleration*dt_ad;

    %% C. ADAPTIVE CONTROL LAW

    trackingError = hModel-h;
    velocityError = vModel-v;

    accelerationCommand = ...
        modelAcceleration ...
        + Kp_ad*trackingError ...
        + Kd_ad*velocityError;

    % Adaptive mass compensation
    rawThrust = massEstimate* ...
        (P.g+accelerationCommand);

    % Actuator saturation
    T = min(max(rawThrust,0),Tmax_ad);

    rotorThrust = (T/4)*ones(4,1);

    %% D. RECORD DATA

    h_ad(k) = h;
    href_ad(k) = hModel;

    massEstimateHistory(k) = massEstimate;
    actualMassHistory(k) = actualMass;

    thrust_ad(k) = T;
    error_ad(k) = trackingError;

    thetaHistory(k) = thetaHat;

    %% E. UPDATE PLANT

    f = @(state) quadrotorDynamics( ...
        state,rotorThrust,Pplant);

    x = X_ad(:,k);

    k1 = f(x);
    k2 = f(x+0.5*dt_ad*k1);
    k3 = f(x+0.5*dt_ad*k2);
    k4 = f(x+dt_ad*k3);

    X_ad(:,k+1) = x + ...
        (dt_ad/6)*(k1+2*k2+2*k3+k4);

    % Save measurements for next estimator update
    previousVelocity = v;
    previousThrust = T;

end

%% 7. FINAL DATA POINTS

h_ad(N_ad) = -X_ad(3,N_ad);
href_ad(N_ad) = hModel;

massEstimateHistory(N_ad) = 1/thetaHat;
actualMassHistory(N_ad) = actualMass;

thrust_ad(N_ad) = thrust_ad(N_ad-1);
error_ad(N_ad) = href_ad(N_ad)-h_ad(N_ad);

thetaHistory(N_ad) = thetaHat;

%% 8. ALTITUDE TRACKING

figure('Name','Adaptive Altitude Tracking');

plot(t_ad,href_ad,'--','LineWidth',1.5);
hold on;

plot(t_ad,h_ad,'LineWidth',1.8);

xline(massChangeTime,'-.','Payload change');

xlabel('Time (s)');
ylabel('Altitude (m)');

legend('Reference model','Actual altitude');
title('Adaptive Altitude Tracking');

grid on;

%% 9. MASS ESTIMATION

figure('Name','Online Mass Estimation');

plot(t_ad,actualMassHistory,'--','LineWidth',1.5);
hold on;

plot(t_ad,massEstimateHistory,'LineWidth',1.8);

xline(massChangeTime,'-.','Payload change');

xlabel('Time (s)');
ylabel('Mass (kg)');

legend('Actual mass','Estimated mass');
title('Online Mass Parameter Estimation');

grid on;

%% 10. THRUST RESPONSE

figure('Name','Adaptive Thrust Response');

plot(t_ad,thrust_ad,'LineWidth',1.8);
hold on;

yline(P.m*P.g,'--','Original hover thrust');
yline(actualMassAfter*P.g,':','New hover thrust');

xline(massChangeTime,'-.','Payload change');

xlabel('Time (s)');
ylabel('Collective thrust (N)');

title('Adaptive Control Effort');
grid on;

%% 11. TRACKING ERROR

figure('Name','Adaptive Tracking Error');

plot(t_ad,error_ad,'LineWidth',1.8);

xline(massChangeTime,'-.','Payload change');

xlabel('Time (s)');
ylabel('Model tracking error (m)');

title('Reference Model Tracking Error');
grid on;

%% 12. PERFORMANCE SUMMARY

fprintf('\nADAPTIVE CONTROL RESULTS\n');
fprintf('------------------------\n');

fprintf('Initial mass: %.3f kg\n',P.m);

fprintf('Actual final mass: %.3f kg\n', ...
    actualMassHistory(end));

fprintf('Estimated final mass: %.3f kg\n', ...
    massEstimateHistory(end));

fprintf('Final altitude: %.4f m\n',h_ad(end));

fprintf('Final reference altitude: %.4f m\n', ...
    href_ad(end));

fprintf('Final tracking error: %.4f m\n',error_ad(end));

fprintf('Maximum thrust: %.4f N\n',max(thrust_ad));

fprintf('\nAdaptive simulation completed.\n');


%% 8. QUADCOPTER DYNAMICS FUNCTION

function Xdot = quadrotorDynamics(X, rotorThrust, P)

    % Extract states
    velocity = X(4:6);
    angles = X(7:9);
    omega = X(10:12);

    phi = angles(1);
    theta = angles(2);
    psi = angles(3);

    % Rotation matrix: Body to NED
    cphi = cos(phi);
    sphi = sin(phi);

    ctheta = cos(theta);
    stheta = sin(theta);

    cpsi = cos(psi);
    spsi = sin(psi);

    R = [ctheta*cpsi, ...
         sphi*stheta*cpsi-cphi*spsi, ...
         cphi*stheta*cpsi+sphi*spsi;

         ctheta*spsi, ...
         sphi*stheta*spsi+cphi*cpsi, ...
         cphi*stheta*spsi-sphi*cpsi;

         -stheta, ...
         sphi*ctheta, ...
         cphi*ctheta];

    % Total thrust
    T = sum(rotorThrust);

    % Translational dynamics
    gravity = [0; 0; P.g];

    thrustBody = [0; 0; -T];

    acceleration = gravity + R*thrustBody/P.m;

    % Rotational moments
    control = P.M * rotorThrust;

    tau = control(2:4);

    % Rigid-body rotational dynamics
    omegaDot = P.J \ ...
        (tau - cross(omega, P.J*omega));

    % Euler angle kinematics
    E = [1, sphi*tan(theta), cphi*tan(theta);
         0, cphi,           -sphi;
         0, sphi/ctheta,     cphi/ctheta];

    angleRates = E * omega;

    % Assemble state derivatives
    Xdot = [velocity;
            acceleration;
            angleRates;
            omegaDot];

end


function [t,h,T,error] = runPIDTest( ...
    Kp,Ki,Kd,massStepTime,massAfter,P,dt,tEnd)

t = 0:dt:tEnd;
N = length(t);

X = zeros(12,N);

h = zeros(1,N);
T = zeros(1,N);
error = zeros(1,N);

integral = 0;

for k = 1:N-1

    tk = t(k);

    % Payload variation
    if tk >= massStepTime
        actualMass = massAfter;
    else
        actualMass = P.m;
    end

    Pplant = P;
    Pplant.m = actualMass;

    % Altitude feedback
    currentAltitude = -X(3,k);
    downVelocity = X(6,k);

    reference = 5 * (tk >= 1);

    e = reference-currentAltitude;

    % PID acceleration command
    a_cmd = Kp*e + Ki*integral + Kd*downVelocity;

    % Controller retains nominal mass assumption
    rawThrust = P.m*(P.g+a_cmd);

    maxThrust = 2.5*P.m*P.g;

    thrust = min(max(rawThrust,0),maxThrust);

    % Conditional integration
    if thrust == rawThrust || ...
            (thrust >= maxThrust && e < 0) || ...
            (thrust <= 0 && e > 0)

        integral = integral + e*dt;

    end

    rotorThrust = (thrust/4)*ones(4,1);

    h(k) = currentAltitude;
    T(k) = thrust;
    error(k) = e;

    % RK4 integration with current plant mass
    f = @(time,state) plantDerivative( ...
        time,state,rotorThrust,P,massStepTime,massAfter);

    x = X(:,k);

    k1 = f(tk,x);
    k2 = f(tk+dt/2,x+dt*k1/2);
    k3 = f(tk+dt/2,x+dt*k2/2);
    k4 = f(tk+dt,x+dt*k3);

    X(:,k+1) = x + ...
        dt*(k1+2*k2+2*k3+k4)/6;

end

h(N) = -X(3,N);
T(N) = T(N-1);
error(N) = 5-h(N);

end


function Xdot = plantDerivative( ...
    t,X,rotorThrust,P,massStepTime,massAfter)

Pnow = P;

if t >= massStepTime
    Pnow.m = massAfter;
end

Xdot = quadrotorDynamics(X,rotorThrust,Pnow);

end


%% 9. DYNAMIC MODEL VERIFICATION

fprintf('\nDYNAMIC MODEL VERIFICATION\n');
fprintf('==========================\n');

tolerance = 1e-9;

%% TEST 1: FREE FALL

u_freefall = zeros(4,1);

Xdot_freefall = quadrotorDynamics(X0, u_freefall, P);

expected_freefall = [0; 0; P.g];

assert(norm(Xdot_freefall(4:6) - expected_freefall) < tolerance);

fprintf('\nTEST 1: FREE FALL - PASSED\n');
fprintf('Vertical acceleration: %.4f m/s^2\n', ...
    Xdot_freefall(6));


%% TEST 2: VERTICAL THRUST

T_test = 1.2 * P.m * P.g;

u_vertical = (T_test/4) * ones(4,1);

Xdot_vertical = quadrotorDynamics(X0, u_vertical, P);

expected_vertical = P.g - T_test/P.m;

assert(abs(Xdot_vertical(6) - expected_vertical) < tolerance);

fprintf('\nTEST 2: VERTICAL THRUST - PASSED\n');
fprintf('Vertical acceleration: %.4f m/s^2\n', ...
    Xdot_vertical(6));


%% TEST 3: ROLL AND PITCH RESPONSE

delta = 0.1;

% Roll input
u_roll = P.m*P.g/4 * ones(4,1) + ...
    delta*[0;0;1;1];

% Pitch input
u_pitch = P.m*P.g/4 * ones(4,1) + ...
    delta*[1;-1;-1;1];

Xdot_roll = quadrotorDynamics(X0, u_roll, P);
Xdot_pitch = quadrotorDynamics(X0, u_pitch, P);

assert(Xdot_roll(10) > 0);
assert(abs(Xdot_roll(11)) < tolerance);
assert(abs(Xdot_roll(12)) < tolerance);

assert(abs(Xdot_pitch(10)) < tolerance);
assert(Xdot_pitch(11) > 0);
assert(abs(Xdot_pitch(12)) < tolerance);

fprintf('\nTEST 3: ROLL AND PITCH - PASSED\n');
fprintf('Roll angular acceleration: %.4f rad/s^2\n', ...
    Xdot_roll(10));

fprintf('Pitch angular acceleration: %.4f rad/s^2\n', ...
    Xdot_pitch(11));


%% TEST 4: YAW RESPONSE

u_yaw = P.m*P.g/4 * ones(4,1) + ...
    delta*[1;-1;1;-1];

Xdot_yaw = quadrotorDynamics(X0, u_yaw, P);

assert(abs(Xdot_yaw(10)) < tolerance);
assert(abs(Xdot_yaw(11)) < tolerance);
assert(abs(Xdot_yaw(12)) > 0);

fprintf('\nTEST 4: YAW - PASSED\n');
fprintf('Yaw angular acceleration: %.4f rad/s^2\n', ...
    Xdot_yaw(12));


%% VALIDATION SUMMARY

fprintf('\n==========================\n');
fprintf('ALL FOUR TESTS PASSED\n');
fprintf('QUADCOPTER MODEL VERIFIED\n');
fprintf('==========================\n');



%% PHASE 01.3: OPEN-LOOP FLIGHT SIMULATION

fprintf('\nOPEN-LOOP FLIGHT SIMULATION\n');

%% 1. SIMULATION CONFIGURATION

tStart = 0;
tEnd = 30;

tspan = [tStart tEnd];

% Initial state
X0 = zeros(12,1);

% Solver options
options = odeset( ...
    'RelTol', 1e-8, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.01);

%% 2. OPEN-LOOP ROTOR COMMAND

hoverThrust = P.m * P.g;

thrustCommand = @(t) ...
    (t < 5)  * hoverThrust/4 + ...
    (t >= 5 && t < 15) * (1.10*hoverThrust/4) + ...
    (t >= 15) * hoverThrust/4;

%% 3. NUMERICAL INTEGRATION

[t,X] = ode45( ...
    @(t,X) quadrotorDynamics( ...
        X, thrustCommand(t)*ones(4,1), P), ...
    tspan, X0, options);

%% 4. EXTRACT STATES

position = X(:,1:3);
velocity = X(:,4:6);

attitude = X(:,7:9);
angularVelocity = X(:,10:12);

%% 5. POSITION VISUALIZATION

figure('Name','Quadcopter Position');

subplot(3,1,1);
plot(t,position(:,1),'LineWidth',1.5);
ylabel('North (m)');
grid on;

subplot(3,1,2);
plot(t,position(:,2),'LineWidth',1.5);
ylabel('East (m)');
grid on;

subplot(3,1,3);
plot(t,-position(:,3),'LineWidth',1.5);
ylabel('Altitude (m)');
xlabel('Time (s)');
grid on;

sgtitle('Open-Loop Position Response');

%% 6. VELOCITY VISUALIZATION

figure('Name','Quadcopter Velocity');

plot(t,velocity,'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('North','East','Down');
grid on;
title('Translational Velocity');

%% 7. ATTITUDE VISUALIZATION

figure('Name','Quadcopter Attitude');

plot(t,rad2deg(attitude),'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('Roll','Pitch','Yaw');
grid on;
title('Vehicle Attitude');

%% 8. ROTATIONAL VELOCITY

figure('Name','Angular Velocity');

plot(t,rad2deg(angularVelocity),'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Angular velocity (deg/s)');
legend('p','q','r');
grid on;
title('Body Angular Rates');

%% 9. 3D TRAJECTORY

figure('Name','Quadcopter Trajectory');

plot3(position(:,2), ...
      position(:,1), ...
      -position(:,3), ...
      'LineWidth',2);

xlabel('East (m)');
ylabel('North (m)');
zlabel('Altitude (m)');

grid on;
axis equal;
title('Open-Loop Flight Trajectory');
view(45,25);

%% 10. PERFORMANCE SUMMARY

fprintf('\nSIMULATION COMPLETED\n');

fprintf('Simulation duration: %.2f seconds\n',t(end));
fprintf('Final altitude: %.4f m\n',-position(end,3));
fprintf('Final vertical velocity: %.4f m/s\n',-velocity(end,3));

fprintf('\nOpen-loop simulation successful.\n');
