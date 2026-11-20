%% ADAPTIVE QUADCOPTER FLIGHT CONTROL

clear;
clc;
close all;

%% Project setup

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(projectRoot));

%% 1. Validate dynamics

fprintf('\n=== PHYSICS VALIDATION ===\n');
validateDynamics();

%% 2. Run fixed PID

fprintf('\n=== FIXED PID SIMULATION ===\n');

pidResults = runPIDPayload();

%% 3. Run adaptive PID

fprintf('\n=== ADAPTIVE PID SIMULATION ===\n');

adaptiveResults = runAdaptive();

%% 4. Compare controllers

fprintf('\n=== CONTROLLER COMPARISON ===\n');

compareControllers(pidResults, adaptiveResults);

%% 5. Plot adaptive mass estimation

figure('Name', 'Mass Estimation');

plot(adaptiveResults.time, ...
     adaptiveResults.actualMass, '--', 'LineWidth', 1.5);

hold on;

plot(adaptiveResults.time, ...
     adaptiveResults.massEstimate, 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Mass (kg)');
title('Online Mass Estimation');
legend('Actual Mass', 'Estimated Mass');

fprintf('\n=== EXPERIMENT COMPLETE ===\n');