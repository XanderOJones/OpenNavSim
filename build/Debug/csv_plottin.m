clear all
% close all


% Import data
r_true = readmatrix('true_positions.csv');       % Nx3
r_dr   = readmatrix('estimated_positions.csv');  % Mx3 (e.g., 1x3 for now)
f_true = readmatrix('true_accel.csv');       % Nx3
w_true = readmatrix('true_gyro.csv');  % Mx3 (e.g., 1x3 for now)

% Plot
figure;
plot3(r_true(:,1), r_true(:,2), r_true(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(r_dr(:,1), r_dr(:,2), r_dr(:,3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('Ground Truth', 'Dead Reckoning');
grid on; axis equal;
title('3D Trajectory: Ground Truth vs Dead Reckoning');

figure;
subplot(2, 1, 1)
plot(1:47999, f_true(1:47999, :))
title("Specific Force")
subplot(2, 1, 2)
plot(1:47999, w_true(1:47999, :))
title("Angular Velocity")