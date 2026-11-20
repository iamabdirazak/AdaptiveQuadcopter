function plotResults(results)

figure('Name', 'Altitude Tracking');

plot(results.time, results.reference, '--', 'LineWidth', 1.5);
hold on;
plot(results.time, results.altitude, 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude Tracking Response');
legend('Reference', 'Actual');

if isfield(results, 'actualMass')

    figure('Name', 'Mass Estimation');

    plot(results.time, results.actualMass, '--', 'LineWidth', 1.5);
    hold on;
    plot(results.time, results.massEstimate, 'LineWidth', 1.5);

    grid on;
    xlabel('Time (s)');
    ylabel('Mass (kg)');
    title('Online Mass Estimation');
    legend('Actual Mass', 'Estimated Mass');

end

end