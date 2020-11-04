import gtsam.*

addpath(genpath('../'));

gt = readtable('../../ground_truth_00.txt', 'Delimiter', ' ');
gt = table2array(gt);

% traj = readtable('test.txt', 'Delimiter', ' ');
% traj = readtable('test2.txt', 'Delimiter', ' ');
traj = readtable('../../results/1750_1_poses.txt', 'Delimiter', ' ');
traj = table2array(traj);

graph = poseGraph;

for i = 1:size(traj, 1)
%     x = traj(i,10);
%     y = traj(i,11);
%     z = traj(i,12);
%     rot = traj(i,1:9);
%     rot = reshape(rot, [3,3]);
%     quat = rotm2quat(rot);

    x = traj(i,1);
    z = traj(i,2);
    theta = traj(i,3);
    addRelativePose(graph, [x z theta]);
end

figure(1)
% axis equal
graph
show(graph, 'IDs', 'off'); hold on
% axis equal
figure(2)
% plot(arr(:,1),arr(:,2)); hold on
plot(gt(1:1750,4),gt(1:1750,12));