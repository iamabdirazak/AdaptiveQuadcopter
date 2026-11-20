function [thrust, state] = adaptiveAltitudeController( ...
    error, verticalVelocity, gains, state, massEstimate, P, dt)
% ADAPTIVEALTITUDECONTROLLER Altitude control using estimated mass.

Kp = gains(1);
Ki = gains(2);
Kd = gains(3);

% Derivative of altitude error
errorDerivative = verticalVelocity;

% Integral update
integralCandidate = state.integral + error * dt;

% PID acceleration command
accelerationCommand = ...
    Kp*error + Ki*integralCandidate + Kd*errorDerivative;

% Adaptive thrust calculation
thrustUnbounded = massEstimate * ...
    (P.g + accelerationCommand);

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