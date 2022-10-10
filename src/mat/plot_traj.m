import gtsam.*

addpath(genpath('../'));

gt = readtable('../../ground_truth_00.txt', 'Delimiter', ' ');
gt = table2array(gt);
poses_init = readtable('../../results/1750_1_poses_initial.txt', 'Delimiter', ' ');
poses_init = table2array(poses_init);
poses_opt = readtable('../../results/1750_1_poses_optimized.txt', 'Delimiter', ' ');
poses_opt = table2array(poses_opt);

graph_init = poseGraph3D;
graph_opt = poseGraph3D;

for i = 1:size(poses_init, 1)
    x = poses_init(i,10);
    y = poses_init(i,11);
    z = poses_init(i,12);
    rot = poses_init(i,1:9);
    rot = reshape(rot, [3,3]);
    quat = rotm2quat(rot);
    addRelativePose(graph_init, [x y z quat]);
    
    x = poses_opt(i,10);
    y = poses_opt(i,11);
    z = poses_opt(i,12);
    rot = poses_opt(i,1:9);
    rot = reshape(rot, [3,3]);
    quat = rotm2quat(rot);
    addRelativePose(graph_opt, [x y z quat]);
end

figure(1)
show(graph_init, 'IDs', 'off'); hold on
axis equal

figure(2)
show(graph_opt, 'IDs', 'off'); hold on
axis equal

figure(3)
% plot(arr(:,1),arr(:,2)); hold on
plot3(gt(1:1750,4),gt(1:1750,4),gt(1:1750,12));