close all;
clc;
clear all;

%% Load the saved commands
commands = load('estimated_commands.mat');
t = commands.t;                     % Elapsed command times (s).
v_quantized = commands.v_quantized; % Estimated linear velocity commands (m/s).
w_quantized = commands.w_quantized; % Estimated angular velocity commands (rad/s).
v_comm = commands.v_comm;           % 1 = increase v, 2 = decrease v, 0 = no change.
w_comm = commands.w_comm;           % 1 = increase w, 2 = decrease w, 0 = no change.

%% Create a ROS 2 node and velocity publisher
domainID = 30; % Must match ROS_DOMAIN_ID in the terminal running Gazebo.
node = ros2node('/matlab_command_publisher', domainID); % MATLAB's node on the ROS 2 network.
velPub = ros2publisher(node, '/cmd_vel', 'geometry_msgs/TwistStamped'); % Publisher for your TB3 Jazzy bridge.
velMsg = ros2message(velPub); % Empty message; velocities go in velMsg.twist.linear.x and .angular.z.

%% Create a ROS 2 bag and start recording
bagFolder = "tb3_replay_" + string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS')); % New folder for each run.
bagRecorder = ros2bagrecorder(bagFolder, domainID, ...
    StorageFormat="mcap", UseSimTime=true); % Record using Gazebo's /clock timestamps.
startRecording(bagRecorder, IncludeTopics=["/odom", "/cmd_vel", "/clock"]);
% Recording continues in the background while MATLAB publishes commands.

%% Publish the commands at their saved times
publishCommands(node, velPub, velMsg, bagRecorder, t, v_quantized, w_quantized, v_comm, w_comm);

%% Local functions
function publishCommands(node, velPub, velMsg, bagRecorder,t, v_quantized, w_quantized, v_comm, w_comm)
% Use a function so cleanup runs when replay finishes, errors, or is interrupted.
clockSub = ros2subscriber(node, '/clock', 'rosgraph_msgs/Clock','Reliability', 'besteffort', 'Depth', 1);
cleanup = onCleanup(@() finishReplay(velPub, clockSub, bagRecorder));
pause(2); % Allow ROS discovery before starting the replay timer.

clockMsg = receive(clockSub, 10); % Wait for an unpaused Gazebo simulation.
startStamp = clockMsg.clock;     % Simulation time when replay begins.
previousElapsed = 0;
velMsg.twist.linear.x = v_quantized(1);  % Value to retain when only w changes.
velMsg.twist.angular.z = w_quantized(1); % Value to retain when only v changes.

for i = 1:length(t)
    % Wait until this sample's time, measured using Gazebo's clock.
    targetTime = t(i) - t(1);
    while true
        elapsed = (double(clockMsg.clock.sec) - double(startStamp.sec)) + (double(clockMsg.clock.nanosec) - double(startStamp.nanosec))*1e-9;
        if elapsed < previousElapsed
            error('Gazebo time moved backward. Restart replay after resetting the simulation.');
        end
        previousElapsed = elapsed;
        if elapsed >= targetTime
            break;
        end
        clockMsg = receive(clockSub, 5);
    end

    v_changed = v_comm(i) == 1 || v_comm(i) == 2;
    w_changed = w_comm(i) == 1 || w_comm(i) == 2;
    if v_changed
        velMsg.twist.linear.x = v_quantized(i); % New v; retain the current w.
    end
    if w_changed
        velMsg.twist.angular.z = w_quantized(i); % New w; retain the current v.
    end
    if v_changed || w_changed
        velMsg.header.stamp = clockMsg.clock;
        send(velPub, velMsg); % Publish once if either or both velocities changed.
    end
end
end

function finishReplay(velPub, clockSub, bagRecorder)
% Stop the robot, then close the recording and save the bag metadata.
try
    stopMsg = ros2message(velPub); % All velocity components start at zero.
    latestClock = clockSub.LatestMessage;
    if ~isempty(latestClock)
        stopMsg.header.stamp = latestClock.clock;
    end
    send(velPub, stopMsg);
    pause(0.1); % Allow the stop message to reach the robot and recorder.
catch stopError
    warning('Could not publish the final stop: %s', stopError.message);
end
stopRecording(bagRecorder);
end
