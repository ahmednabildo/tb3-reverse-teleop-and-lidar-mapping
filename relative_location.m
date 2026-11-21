close all;
clc;
clear all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%****Bag provided: topics include /odom, /scan, /tf, /tf_static, /clock (no/cmd_vel).*****
bagFolder ="tb3_nocmd"; % folder with metadata.yaml + *.mcap
bag = ros2bagreader(bagFolder);

tfBag = select(bag, "Topic", "/tf_static");
msgs_tf = readMessages(tfBag);
msgs_tf{1}.transforms

transforms = msgs_tf{1}.transforms;

for i = 1:length(transforms)
    fprintf('Parent: %s\n', transforms(i).header.frame_id);
    fprintf('Child: %s\n', transforms(i).child_frame_id);
    disp(transforms(i).transform.translation);
    disp(transforms(i).transform.rotation);
end


% Parent: /base_footprint
% Child: /base_link
%     MessageType: 'geometry_msgs/Vector3'
%               x: 0
%               y: 0
%               z: 0.0100

%     MessageType: 'geometry_msgs/Quaternion'
%               x: 0
%               y: 0
%               z: 0
%               w: 1


% Parent: /base_link
% Child: /base_scan
%     MessageType: 'geometry_msgs/Vector3'
%               x: -0.0320
%               y: 0
%               z: 0.1720

%     MessageType: 'geometry_msgs/Quaternion'
%               x: 0
%               y: 0
%               z: 0
%               w: 1
