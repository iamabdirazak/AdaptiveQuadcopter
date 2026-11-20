function compareControllers(pidResults, adaptiveResults)
% COMPARECONTROLLERS Compare fixed PID and adaptive PID performance.

t = pidResults.time;

% Tracking errors
pidError = pidResults.reference - pidResults.altitude;
adaptiveError = adaptiveResults.reference - adaptiveResults.altitude;

%% Altitude comparison

figure('Name', 'Controller Comparison');

subplot(2,1,1);

plot(t, pidResults.reference, '--k', 'LineWidth', 1.2);
hold on;

plot(t, pidResults.altitude, 'LineWidth', 1.5);
plot(t, adaptiveResults.altitude, 'LineWidth', 1.5);

xline(15, ':r', 'Payload Change');

grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude Tracking Comparison');

legend('Reference', 'Fixed PID', 'Adaptive PID');

%% Tracking error comparison

subplot(2,1,2);

plot(t, pidError, 'LineWidth', 1.5);
hold on;

plot(t, adaptiveError, 'LineWidth', 1.5);

xline(15, ':r', 'Payload Change');

grid on;
xlabel('Time (s)');
ylabel('Tracking Error (m)');
title('Tracking Error Comparison');

legend('Fixed PID', 'Adaptive PID');

end