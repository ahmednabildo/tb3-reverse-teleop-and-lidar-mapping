% Keyboard controls:
% w: increase v by 0.01 m/s
% x: decrease v by 0.01 m/s
% a: increase w by 0.1 rad/s (left)
% d: decrease w by 0.1 rad/s (right)
% s or Space: stop v and w
% Ctrl+C: exit
%
% Quaternion: qx, qy, qz, qw describe orientation, not separate angles.
% qw is the scalar part of the unit quaternion.
% theta = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
% unwrap removes jumps between -pi and pi.
% w = d(theta)/dt
% ROS axes: x forward, y left, z up.
% Seen from above: positive w = left, negative w = right.
% Zero w = no rotation.
% a and d change w; they do not directly set its sign.
% Key presses are estimated from odometry.
% v is speed magnitude, so it does not show reverse motion.


close all;
clc;
clear all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%****Bag provided: topics include /odom, /scan, /tf, /tf_static, /clock (no/cmd_vel).*****
bagFolder ="tb3_nocmd"; % folder with metadata.yaml + *.mcap
bag = ros2bagreader(bagFolder);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
odomBag = select(bag, "Topic", "/odom");
msgs = readMessages(odomBag); % Read all odometry messages

% Extract the planar position from each message.
x = cellfun(@(odom) odom.pose.pose.position.x, msgs);
y = cellfun(@(odom) odom.pose.pose.position.y, msgs);

% Plot the robot's trajectory in the x-y plane.
figure;
plot(x, y, 'b-', 'LineWidth', 1.5);
axis equal;
grid on;
xlabel('x (m)');
ylabel('y (m)');
title('Robot trajectory from /odom');

% Get elapsed time from message headers, preserving timestamp precision.

sec = cellfun(@(odom) double(odom.header.stamp.sec), msgs);
nanosec = cellfun(@(odom) double(odom.header.stamp.nanosec), msgs);
t = (sec - sec(1)) + (nanosec - nanosec(1))*1e-9;%sec(1) = 61 ;;; nanosec(1) = 20000000

% Plot x and y positions against elapsed time.
figure;
subplot(2,1,1);
plot(t, x, 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('x (m)');
title('X position');
grid on;

subplot(2,1,2);
plot(t, y, 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('y (m)');
title('Y position');
grid on;

% Differentiate with respect to time using the actual sample spacing.
% These velocities and accelerations are in the odometry frame.
x_dot = gradient(x, t);
y_dot = gradient(y, t);
x_ddot = gradient(x_dot, t);
y_ddot = gradient(y_dot, t);

% Plot first and second derivatives. Differentiation amplifies measurement noise.
figure;
subplot(2,1,1);
plot(t, x_dot, t, y_dot, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('dx/dt', 'dy/dt');
title('First derivatives of position');

subplot(2,1,2);
plot(t, x_ddot, t, y_ddot, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('d^2x/dt^2', 'd^2y/dt^2');
title('Second derivatives of position');

% Extract orientation quaternion from odometry
qx = cellfun(@(odom) odom.pose.pose.orientation.x, msgs);
qy = cellfun(@(odom) odom.pose.pose.orientation.y, msgs);
qz = cellfun(@(odom) odom.pose.pose.orientation.z, msgs);
qw = cellfun(@(odom) odom.pose.pose.orientation.w, msgs);

% Compute the actual robot heading (yaw)
theta = atan2(2.*(qw.*qz + qx.*qy), ...
              1 - 2.*(qy.^2 + qz.^2));

% Remove angle wrapping before differentiating
theta = unwrap(theta);

% Linear speed
v = sqrt(x_dot.^2 + y_dot.^2);

% Angular velocity
w = gradient(theta, t);

% Time derivatives of linear speed and angular velocity.
v_dot = gradient(v, t);         % m/s^2
w_dot = gradient(w, t); % rad/s^2

figure;
plot(t, theta, 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('\theta (rad)');
title('Robot heading');
grid on;

figure;
plot(t, w, 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('\w (rad/s)');
title('Angular velocity');
grid on;

figure;
plot(t, v, 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('v (m/s)');
title('Linear speed');
grid on;

figure;
plot(t, v_dot, 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('dv/dt (m/s^2)');
title('Rate of change of linear speed');
grid on;

figure;
plot(t, w_dot, 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('d\w/dt (rad/s^2)');
title('Angular acceleration');
grid on;

% Behavior segmentation: tolerances reduce sensitivity to numerical noise.
v_tol = 0.001;       % m/s: speeds at or below this are stationary.
w_tol = 0.01;   % rad/s: magnitudes at or below this are straight.

moving = v > v_tol;
behavior = repmat("Stationary", size(v));
behavior(moving & abs(w) <= w_tol) = "Straight";
behavior(moving & w > w_tol) = "Left turn";
behavior(moving & w < -w_tol) = "Right turn";

% Separate colorized x-y trajectory. Each edge uses its starting sample's label.
% NaN separators prevent lines from joining disconnected segments.
figure;
hold on;
classNames = ["Straight", "Left turn", "Right turn"];
classColors = [0 0.45 0.74; 0.1 0.65 0.2; 0.85 0.15 0.15];
xPlot = x(:);
yPlot = y(:);
for k = 1:numel(classNames)
    idx = find(behavior(1:end-1) == classNames(k));
    edgeX = [xPlot(idx).'; xPlot(idx+1).'; nan(1,numel(idx))];
    edgeY = [yPlot(idx).'; yPlot(idx+1).'; nan(1,numel(idx))];
    plot(edgeX(:), edgeY(:), 'Color', classColors(k,:), ...
        'LineWidth', 2, 'DisplayName', classNames(k));
end
axis equal;
grid on;
xlabel('x (m)');
ylabel('y (m)');
title('Trajectory classified by behavior');
legend('show', 'Location', 'best');
hold off;

% Quantize v and w using command steps.
v_step = 0.01; % Linear speed per command level (m/s).
w_step = 0.1;  % Angular velocity per command level (rad/s).

level_hold_time = 0.06; % seconds before accepting a new level
v_onset_tol = 0.0001; % Tolerance for matching the old linear speed (m/s; 10% of step).
w_onset_tol = 0.01;  % Tolerance for matching the old angular velocity (rad/s; 10% of step).

% Use integer levels for comparisons.
v_level = round(v(1)/v_step); % Currently accepted integer linear-speed level.
w_level = round(w(1)/w_step); % Currently accepted integer angular-velocity level.

v_candidate = v_level; % Possible new linear-speed level being checked.
w_candidate = w_level; % Possible new angular-velocity level being checked.

v_candidate_start = 1; % Sample index where the current v candidate first appeared.
w_candidate_start = 1; % Sample index where the current w candidate first appeared.

v_last_confirmed = 0; % Sample index where the last v change was confirmed; 0 means none.
w_last_confirmed = 0; % Sample index where the last w change was confirmed; 0 means none.

v_quantized = zeros(size(v)); % Accepted linear-speed values (m/s) at each sample.
w_quantized = zeros(size(w)); % Accepted angular-velocity values (rad/s) at each sample.

v_quantized(1) = v_level*v_step;
w_quantized(1) = w_level*w_step;

for i = 2:length(t) % i is the current sample index.
    next_v_level = round(v(i)/v_step); % Nearest integer level for the current measured v.
    if next_v_level ~= v_candidate
        v_candidate = next_v_level;
        v_candidate_start = i;
    end
    if v_candidate ~= v_level && t(i) - t(v_candidate_start) >= level_hold_time
        % Find the start of the change after the previous confirmed change.
        v_transition_start = v_candidate_start; % Estimated starting sample of the v change.
        for k = v_candidate_start:-1:max(1, v_last_confirmed+1) % k searches backward through samples.
            v_transition_start = k;
            if abs(v(k) - v_level*v_step) <= v_onset_tol % find where v(k) = previous v_level*v_step
                break;
            end
        end
        v_level = v_candidate;
        v_quantized(v_transition_start:i) = v_level*v_step;
        v_last_confirmed = i;
    end
    v_quantized(i) = v_level*v_step;

    next_w_level = round(w(i)/w_step); % Nearest integer level for the current measured w.
    if next_w_level ~= w_candidate
        w_candidate = next_w_level;
        w_candidate_start = i;
    end
    if w_candidate ~= w_level && t(i) - t(w_candidate_start) >= level_hold_time
        w_transition_start = w_candidate_start; % Estimated starting sample of the w change.
        for k = w_candidate_start:-1:max(1, w_last_confirmed+1)
            w_transition_start = k;
            if abs(w(k) - w_level*w_step) <= w_onset_tol
                break;
            end
        end
        w_level = w_candidate;
        w_quantized(w_transition_start:i) = w_level*w_step;
        w_last_confirmed = i;
    end
    w_quantized(i) = w_level*w_step;
end

% Commands: 1 = increase, 2 = decrease, 0 = no change.
v_comm = zeros(size(t));
w_comm = zeros(size(t));
v_stop = false(size(t));
for i = 2:length(t)
    if v_quantized(i) == 0 && v_quantized(i-1) ~= 0
        v_stop(i) = true;
    end
    if v_quantized(i) > v_quantized(i-1)
        v_comm(i) = 1;
    elseif v_quantized(i) < v_quantized(i-1)
        v_comm(i) = 2;
    end
    if w_quantized(i) > w_quantized(i-1)
        w_comm(i) = 1;
    elseif w_quantized(i) < w_quantized(i-1)
        w_comm(i) = 2;
    end
end
% Mark w at the same time as the stop in v.
w_stop = v_stop;
v_increase_times = t(v_comm == 1);
v_decrease_times = t(v_comm == 2);
w_increase_times = t(w_comm == 1);
w_decrease_times = t(w_comm == 2);
v_stop_times = t(v_stop);
w_stop_times = t(w_stop);

% Keyboard buttons for w.
w_left = w_comm == 1 & ~w_stop;  % Increasing w: a pressed.
w_right = w_comm == 2 & ~w_stop; % Decreasing w: d pressed.

figure;
subplot(2,1,1);
plot(t, v, 'Color', [0.6 0.6 0.6]);
hold on;
stairs(t, v_quantized, 'b-', 'LineWidth', 1.2);
plot(t(v_comm == 1 & ~v_stop), v_quantized(v_comm == 1 & ~v_stop), 'r.', 'MarkerSize', 16);
plot(t(v_comm == 2 & ~v_stop), v_quantized(v_comm == 2 & ~v_stop), 'g.', 'MarkerSize', 16);
plot(t(v_stop), v_quantized(v_stop), 'ms', 'MarkerFaceColor', 'm');
for k = 1:length(t)
    if v_stop(k)
        text(t(k), v_quantized(k), ' Stop (s/Space)', 'Color', 'm', ...
            'VerticalAlignment', 'bottom');
    end
end
xlabel('Time (s)');
ylabel('v (m/s)');
title('quantized linear speed');
legend('v', 'v command', 'w pressed', 'x pressed', ...
    's/Space pressed', 'Location', 'best');
grid on;
hold off;

subplot(2,1,2);
plot(t, w, 'Color', [0.6 0.6 0.6]);
hold on;
stairs(t, w_quantized, 'b-', 'LineWidth', 1.2);
plot(t(w_left), w_quantized(w_left), 'r.', 'MarkerSize', 16);
plot(t(w_right), w_quantized(w_right), 'g.', 'MarkerSize', 16);
plot(t(w_stop), w_quantized(w_stop), 'ms', 'MarkerFaceColor', 'm');
for k = 1:length(t)
    if w_stop(k)
        text(t(k), w_quantized(k), ' Stop (s/Space)', 'Color', 'm', ...
            'VerticalAlignment', 'bottom');
    end
end
xlabel('Time (s)');
ylabel('\w (rad/s)');
title('quantized angular velocity');
legend('w', 'w command', 'a pressed (left)', 'd pressed (right)', ...
    's/Space pressed', 'Location', 'best');
grid on;
hold off;

% Save estimated velocity commands and their elapsed times for another MATLAB file.
% t: seconds; v_quantized: m/s; w_quantized: rad/s; *_comm: change flags (1 or 2).
save('estimated_commands.mat', 't', 'v_quantized', 'w_quantized', 'v_comm', 'w_comm');
% In the other file, load them with: commands = load('estimated_commands.mat');

