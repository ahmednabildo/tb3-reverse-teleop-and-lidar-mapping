close all;
clc;
clear all;

%% Load the original and replay bags
originalBag = ros2bagreader('tb3_nocmd');
replayBag = ros2bagreader('tb3_replay_20261112_155831_273');

%% Read the odometry messages from both bags
originalOdom = readMessages(select(originalBag, 'Topic', '/odom'));
replayOdom = readMessages(select(replayBag, 'Topic', '/odom'));
originalOdom = originalOdom(:); % Use column vectors for point-by-point comparisons.
replayOdom = replayOdom(:);

%% Extract X and Y positions (m)
x = cellfun(@(msg) msg.pose.pose.position.x, originalOdom);
y = cellfun(@(msg) msg.pose.pose.position.y, originalOdom);
x_replay = cellfun(@(msg) msg.pose.pose.position.x, replayOdom);
y_replay = cellfun(@(msg) msg.pose.pose.position.y, replayOdom);

%% Extract elapsed time for each bag
% Use the same calculation as assignment1.m; each bag starts at elapsed time zero.
% Original bag timestamps.
sec_original = cellfun(@(odom) double(odom.header.stamp.sec), originalOdom);
nanosec_original = cellfun(@(odom) double(odom.header.stamp.nanosec), originalOdom);
t_original = (sec_original - sec_original(1)) + ...
    (nanosec_original - nanosec_original(1))*1e-9;

% Replay bag timestamps.
sec_replay = cellfun(@(odom) double(odom.header.stamp.sec), replayOdom);
nanosec_replay = cellfun(@(odom) double(odom.header.stamp.nanosec), replayOdom);
t_replay = (sec_replay - sec_replay(1)) + ...
    (nanosec_replay - nanosec_replay(1))*1e-9;

%% Get the last value of each position array
x_end = x(end);
y_end = y(end);
x_r_end = x_replay(end);
y_r_end = y_replay(end);

%% Scan backward to find where the repeated final values begin
% Compare positions rounded to the same number of decimal places.
decimal_places = 4; % Change this to choose the comparison precision.
x_end_idx = length(x);
while x_end_idx > 1 && round(x(x_end_idx - 1), decimal_places) == round(x_end, decimal_places)
    x_end_idx = x_end_idx - 1;
end

y_end_idx = length(y);
while y_end_idx > 1 && round(y(y_end_idx - 1), decimal_places) == round(y_end, decimal_places)
    y_end_idx = y_end_idx - 1;
end

x_r_end_idx = length(x_replay);
while x_r_end_idx > 1 && round(x_replay(x_r_end_idx - 1), decimal_places) == round(x_r_end, decimal_places)
    x_r_end_idx = x_r_end_idx - 1;
end

y_r_end_idx = length(y_replay);
while y_r_end_idx > 1 && round(y_replay(y_r_end_idx - 1), decimal_places) == round(y_r_end, decimal_places)
    y_r_end_idx = y_r_end_idx - 1;
end

fprintf('Original final-value start indices: x = %d, y = %d.\n', x_end_idx, y_end_idx);
fprintf('Replay final-value start indices: x = %d, y = %d.\n', x_r_end_idx, y_r_end_idx);

t_dif = length(t_replay) - length(t_original); % Number of extra replay samples.
begin_dif = y_r_end_idx - y_end_idx;

%% Trim the extra replay points using the calculated counts
replayOdom = replayOdom(begin_dif + 1 : end - (t_dif - begin_dif));
x_replay = x_replay(begin_dif + 1 : end - (t_dif - begin_dif));
y_replay = y_replay(begin_dif + 1 : end - (t_dif - begin_dif));
sec_replay = sec_replay(begin_dif + 1 : end - (t_dif - begin_dif));
nanosec_replay = nanosec_replay(begin_dif + 1 : end - (t_dif - begin_dif));
t_replay = t_replay(begin_dif + 1 : end - (t_dif - begin_dif));
t_replay = t_replay - t_replay(1);

%% Plot both trajectories on the same axes
figure;
plot(x, y, 'b-', 'LineWidth', 1.5);
hold on;
plot(x_replay, y_replay, 'r--', 'LineWidth', 1.5);
axis equal;
grid on;
xlabel('x (m)');
ylabel('y (m)');
title('Original and replayed trajectories');
legend('Original', 'Replay', 'Location', 'best');
hold off;

%% theta replay
qx_replay = cellfun(@(odom) odom.pose.pose.orientation.x, replayOdom);
qy_replay = cellfun(@(odom) odom.pose.pose.orientation.y, replayOdom);
qz_replay = cellfun(@(odom) odom.pose.pose.orientation.z, replayOdom);
qw_replay = cellfun(@(odom) odom.pose.pose.orientation.w, replayOdom);

theta_replay = atan2(2.*(qw_replay.*qz_replay + qx_replay.*qy_replay), ...
    1 - 2.*(qy_replay.^2 + qz_replay.^2));
theta_replay = unwrap(theta_replay);

%% theta original
%% Get theta for the original bag from the quaternion
qx_original = cellfun(@(odom) odom.pose.pose.orientation.x, originalOdom);
qy_original = cellfun(@(odom) odom.pose.pose.orientation.y, originalOdom);
qz_original = cellfun(@(odom) odom.pose.pose.orientation.z, originalOdom);
qw_original = cellfun(@(odom) odom.pose.pose.orientation.w, originalOdom);

theta_original = atan2(2.*(qw_original.*qz_original + qx_original.*qy_original), ...
    1 - 2.*(qy_original.^2 + qz_original.^2));

theta_original = unwrap(theta_original); % Heading in radians.

%% Calculate errors: replay minus original
assert(length(t_replay) == length(t_original), ...
    'Original and replay must have the same number of samples.');
assert(all(abs(t_replay - t_original) < 1e-6), ...
    'Original and replay elapsed timestamps must match for point-by-point errors.');

x_error = x_replay - x; % X position error (m).
y_error = y_replay - y; % Y position error (m).
theta_error = theta_replay - theta_original;
theta_error = atan2(sin(theta_error), cos(theta_error)); % Heading error in [-pi, pi] (rad).

%% Plot original and replay X, Y, and heading versus time
figure;
subplot(3,1,1);
plot(t_original, x, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_replay, x_replay, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('x (m)');
title('X position');
legend('Original', 'Replay', 'Location', 'best');
hold off;

subplot(3,1,2);
plot(t_original, y, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_replay, y_replay, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('y (m)');
title('Y position');
legend('Original', 'Replay', 'Location', 'best');
hold off;

subplot(3,1,3);
plot(t_original, theta_original, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_replay, theta_replay, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('\theta (rad)');
title('Heading');
legend('Original', 'Replay', 'Location', 'best');
hold off;

%% Load the estimated V command points
commands = load('estimated_commands.mat', 't', 'v_comm', 'v_quantized');
v_comm = commands.v_comm(:);
v_quantized = commands.v_quantized(:);
assert(length(v_comm) == length(t_original) && ...
    all(abs(commands.t(:) - commands.t(1) - t_original) < 1e-6), ...
    'Saved commands must match the original odometry timestamps.');

v_stop = false(size(v_comm));
v_stop(2:end) = v_quantized(2:end) == 0 & v_quantized(1:end-1) ~= 0;
v_increase = v_comm == 1 & ~v_stop;
v_decrease = v_comm == 2 & ~v_stop;

%% Plot X, Y, and heading errors with the V command points
figure;
subplot(3,1,1);
plot(t_original, x_error, 'LineWidth', 1.2);
hold on;
plot(t_original(v_increase), x_error(v_increase), 'r.', 'MarkerSize', 16);
plot(t_original(v_decrease), x_error(v_decrease), 'g.', 'MarkerSize', 16);
plot(t_original(v_stop), x_error(v_stop), 'ms', 'MarkerFaceColor', 'm');
grid on;
xlabel('Time (s)');
ylabel('X error (m)');
title('X position error');
legend('Error', 'v increase (w)', 'v decrease (x)', 'Stop (s/Space)', 'Location', 'best');
hold off;

subplot(3,1,2);
plot(t_original, y_error, 'LineWidth', 1.2);
hold on;
plot(t_original(v_increase), y_error(v_increase), 'r.', 'MarkerSize', 16);
plot(t_original(v_decrease), y_error(v_decrease), 'g.', 'MarkerSize', 16);
plot(t_original(v_stop), y_error(v_stop), 'ms', 'MarkerFaceColor', 'm');
grid on;
xlabel('Time (s)');
ylabel('Y error (m)');
title('Y position error');
legend('Error', 'v increase (w)', 'v decrease (x)', 'Stop (s/Space)', 'Location', 'best');
hold off;

subplot(3,1,3);
plot(t_original, theta_error, 'LineWidth', 1.2);
hold on;
plot(t_original(v_increase), theta_error(v_increase), 'r.', 'MarkerSize', 16);
plot(t_original(v_decrease), theta_error(v_decrease), 'g.', 'MarkerSize', 16);
plot(t_original(v_stop), theta_error(v_stop), 'ms', 'MarkerFaceColor', 'm');
grid on;
xlabel('Time (s)');
ylabel('Heading error (rad)');
title('Heading error');
legend('Error', 'v increase (w)', 'v decrease (x)', 'Stop (s/Space)', 'Location', 'best');
hold off;

%% Plot the same errors without command points
figure;
subplot(3,1,1);
plot(t_original, x_error, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('X error (m)');
title('X position error');

subplot(3,1,2);
plot(t_original, y_error, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Y error (m)');
title('Y position error');

subplot(3,1,3);
plot(t_original, theta_error, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Heading error (rad)');
title('Heading error');

%% Print error extremes in both directions and the maximum absolute error
max_x_error = max(x_error);
min_x_error = min(x_error);
max_abs_x_error = max(abs(x_error));

max_y_error = max(y_error);
min_y_error = min(y_error);
max_abs_y_error = max(abs(y_error));

max_theta_error = max(theta_error);
min_theta_error = min(theta_error);
max_abs_theta_error = max(abs(theta_error));

fprintf('X error (m):       min = %.6f, max = %.6f, max absolute = %.6f\n', ...
    min_x_error, max_x_error, max_abs_x_error);
fprintf('Y error (m):       min = %.6f, max = %.6f, max absolute = %.6f\n', ...
    min_y_error, max_y_error, max_abs_y_error);
fprintf('Heading error (rad): min = %.6f, max = %.6f, max absolute = %.6f\n', ...
    min_theta_error, max_theta_error, max_abs_theta_error);

%% Final configuration error: replay minus original
final_x_error = x_error(end);         % Final X difference (m).
final_y_error = y_error(end);         % Final Y difference (m).
final_theta_error = theta_error(end); % Final heading difference in [-pi, pi] (rad).
final_configuration_error = [final_x_error; final_y_error; final_theta_error];
final_position_error = hypot(final_x_error, final_y_error); % Final XY distance (m).

fprintf('Final configuration error [dx (m), dy (m), dtheta (rad)]: [%.6f, %.6f, %.6f]\n', ...
    final_x_error, final_y_error, final_theta_error);
fprintf('Final position error: %.6f m\n', final_position_error);
