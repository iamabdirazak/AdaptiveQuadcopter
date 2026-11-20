function [thrust, state] = pidAltitudeController( ...
    error, verticalVelocity, gains, state, P, dt)
% PIDALTITUDECONTROLLER Altitude PID control law.
%
% Inputs:
% error            - Reference altitude minus actual altitude (m)
% verticalVelocity - NED vertical velocity (m/s)
% gains             - [Kp, Ki, Kd]
% state.integral    - Accumulated error
% P                 - Quadcopter parameters
% dt                - Simulation time step (s)
%
% Outputs:
% thrust            - Total thrust command (N)
% state             - Updated controller state

Kp = gains(1);
Ki = gains(2);
Kd = gains(3);

% Derivative of altitude error
errorDerivative = verticalVelocity;

% Integrate error
integralCandidate = state.integral + error * dt;

% Acceleration command
accelerationCommand = ...
    Kp*error + Ki*integralCandidate + Kd*errorDerivative;

% Convert acceleration command to thrust
thrustUnbounded = P.m * (P.g + accelerationCommand);

% Thrust limits
thrustMin = 0;
thrustMax = 2.5 * P.m * P.g;

thrust = min(max(thrustUnbounded, thrustMin), thrustMax);

% Conditional integration anti-windup
if thrust == thrustUnbounded || ...
        (thrust == thrustMax && error < 0) || ...
        (thrust == thrustMin && error > 0)

    state.integral = integralCandidate;
end

end