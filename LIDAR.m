close all;
clc;
clear all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%****Bag provided: topics include /odom, /scan, /tf, /tf_static, /clock (no/cmd_vel).*****
bagFolder ="tb3_nocmd"; % folder with metadata.yaml + *.mcap
bag = ros2bagreader(bagFolder);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scanBag = select(bag, "Topic", "/scan");
msgs_scan = readMessages(scanBag); % Read all laser scan messages
msgs_scan = msgs_scan(:);
odomBag = select(bag, "Topic", "/odom");
msgs_odom = readMessages(odomBag); % Read all odometry messages

% | Topic | Frequency | Time between messages |
% |---|---:|---:|
% | `/scan` | **5 Hz** | **0.2 s** |
% | `/odom` | **50 Hz** | **0.02 s** |

% Get the robot position and heading from odometry.
msgs_odom = msgs_odom(:);
x_odom = cellfun(@(odom) odom.pose.pose.position.x, msgs_odom);
y_odom = cellfun(@(odom) odom.pose.pose.position.y, msgs_odom);
qx = cellfun(@(odom) odom.pose.pose.orientation.x, msgs_odom);
qy = cellfun(@(odom) odom.pose.pose.orientation.y, msgs_odom);
qz = cellfun(@(odom) odom.pose.pose.orientation.z, msgs_odom);
qw = cellfun(@(odom) odom.pose.pose.orientation.w, msgs_odom);
theta = atan2(2.*(qw.*qz + qx.*qy), 1 - 2.*(qy.^2 + qz.^2));
theta = unwrap(theta);

% Use the same time origin for odometry and scans.
sec_odom = cellfun(@(odom) double(odom.header.stamp.sec), msgs_odom);
nanosec_odom = cellfun(@(odom) double(odom.header.stamp.nanosec), msgs_odom);
t_odom = (sec_odom - sec_odom(1)) + (nanosec_odom - nanosec_odom(1))*1e-9;

sec_scan = cellfun(@(scan) double(scan.header.stamp.sec), msgs_scan);
nanosec_scan = cellfun(@(scan) double(scan.header.stamp.nanosec), msgs_scan);
t_scan = (sec_scan - sec_odom(1)) + (nanosec_scan - nanosec_odom(1))*1e-9;

%% Average each group of 10 odometry readings
samples_per_average = 10; % /odom is 50 Hz and /scan is 5 Hz in this bag.
num_averaged = floor(length(msgs_odom)/samples_per_average);
odom_averaged = zeros(num_averaged, 3); % Columns are [x (m), y (m), theta (rad)].
t_averaged = zeros(num_averaged, 1);
scan_indices = zeros(num_averaged, 1);

% Use complete groups; ignore any remaining readings at the end.
for i = 1:num_averaged
    idx = (i-1)*samples_per_average + (1:samples_per_average);
    odom_averaged(i,:) = [mean(x_odom(idx)), mean(y_odom(idx)), mean(theta(idx))];
    t_averaged(i) = mean(t_odom(idx));

    % Match the averaged timestamp to the closest scan timestamp.
    [~, scan_indices(i)] = min(abs(t_scan - t_averaged(i)));
end

% LiDAR offset from /tf_static in this bag.
% base_footprint and base_link have the same planar position and heading.
lidar_x = -0.032; % LiDAR is 3.2 cm behind base_link (m).
lidar_y = 0;
lidar_theta = 0; % LiDAR has the same heading as base_link (rad).

obstacle_points = zeros(0, 2); % Each row is an obstacle position [x, y] in /odom.
obstacle_tol = 0.0002; % Points within 0.2 mm of a stored point are ignored (m).

for k = 1:num_averaged
    % Get the averaged robot pose and its corresponding scan.
    x_ref = odom_averaged(k,1);
    y_ref = odom_averaged(k,2);
    theta_ref = odom_averaged(k,3);
    if ~isfinite(x_ref) || ~isfinite(y_ref) || ~isfinite(theta_ref)
        continue;
    end

    scan = msgs_scan{scan_indices(k)};
    ranges = double(scan.ranges(:));
    angles = double(scan.angle_min) + (0:length(ranges)-1)'*double(scan.angle_increment);
    valid = isfinite(ranges) & ranges >= double(scan.range_min) & ...
        ranges <= double(scan.range_max);
    ranges = ranges(valid);
    angles = angles(valid);

    % Obstacle positions relative to the robot.
    x_robot = lidar_x + ranges.*cos(angles + lidar_theta);
    y_robot = lidar_y + ranges.*sin(angles + lidar_theta);

    % Obstacle positions in the fixed odometry frame.
    x_map = x_ref + x_robot*cos(theta_ref) - y_robot*sin(theta_ref);
    y_map = y_ref + x_robot*sin(theta_ref) + y_robot*cos(theta_ref);

    % Add the point only if no stored point is within the distance tolerance.
    for j = 1:length(x_map)
        distance = hypot(obstacle_points(:,1) - x_map(j),obstacle_points(:,2) - y_map(j));
        point_exists = any(distance <= obstacle_tol);
        if ~point_exists
            obstacle_points(end+1,:) = [x_map(j), y_map(j)];
        end
    end
end

figure;
plot(obstacle_points(:,1), obstacle_points(:,2), 'b.', 'MarkerSize', 3);
hold on;
plot(odom_averaged(:,1), odom_averaged(:,2), 'r-', 'LineWidth', 1.5);
axis equal;
grid on;
xlabel('x (m)');
ylabel('y (m)');
title('Obstacle points using averaged odometry');
legend('Obstacle points', 'Averaged robot trajectory', 'Location', 'best');
hold off;
