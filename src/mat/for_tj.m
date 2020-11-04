matlabrc; clc; close all;

import gtsam.*

addpath(genpath('tools')) % these are things you actually want
addpath(genpath('misc')) % these are things just for generating fake data
addpath(genpath('../'));

% monovo_results = readtable('MonoVO_00.txt', 'Delimiter', ' ');
gt = readtable('ground_truth_00.txt', 'Delimiter', ' ');
gt = table2array(gt);

traj = readtable('MonoVO_00_1000_scaling.txt', 'Delimiter', ' ');
traj = table2array(traj);

% plot2DTrajectory(traj, 'r~'); axis equal

% lim = size(mv,1)
% gt = gt(1:lim,:)

%% Generate a random trajectory:
% This part is just to generate random data to plot.  Its assumed you have
% this done already...

% Generate some random true trajectory:
% dt = 10; %(sec)
% tspan = (0:dt:30*60)'; % time array, Nx1 
% true_traj = gen_traj(tspan); % Nx12 array (each row is 9 rotation + 3 translation)

% Add noise for "estimated" trajectory (for demonstration purposes)
% sigang = 5; % standard deviation of euler angle estimate
% sigpos = 5; % standard deviation of position
% est_traj = zeros(size(true_traj));
% for ii = 1:size(est_traj,1)
%     est_rot = reshape(true_traj(ii,1:9),3,3)*e2a(sigang*randn,sigang*randn,sigang*randn, true);
%     est_pos = true_traj(ii,10:end) + sigpos*randn(1,3);
%     est_traj(ii,:) = [reshape(est_rot,1,[]), est_pos];
% end

%% Just plot of trajectory:
figure()
% plot3(gt(:,4),gt(:,8),gt(:,12)); hold on
%plot3(traj(:,10),traj(:,11),traj(:,12))
plot(gt(1:1600,4),gt(1:1600,12)); hold on
plot(traj(:,10),traj(:,12))
axis equal

%% Animation of just trajectory:
% figure()
% plot3(gt(:,4),gt(:,8),gt(:,12)); hold on
% plot3(mv(:,4),mv(:,8),mv(:,12))
% true_pt = plot3(gt(1,4),gt(1,8),gt(1,12),'.g','MarkerSize',20);
% est_pt = plot3(mv(1,4),mv(1,8),mv(1,12),'xb','MarkerSize',20);
% axis equal; grid on
% for ii = 1:size(mv,1)
%     % Update the points:
%     set(true_pt,'XData',gt(ii,4),'YData',gt(ii,8),'ZData',gt(ii,12));
%     set(est_pt,'XData',mv(ii,4),'YData',mv(ii,8),'ZData',mv(ii,12));
%     drawnow
%     pause(.05)
% end

%% Animation of Everything:
% Initialize:
% figure()
% plot3(true_traj(:,10),true_traj(:,11),true_traj(:,12)); hold on
% plot3(est_traj(:,10),est_traj(:,11),est_traj(:,12))
% scale = 50;
% LW = 3;
% true_att.x = plot3(true_traj(1,10)+[0 scale*true_traj(1,1)],...
%                    true_traj(1,11)+[0 scale*true_traj(1,2)],...
%                    true_traj(1,12)+[0 scale*true_traj(1,3)],'r','LineWidth',LW);
% true_att.y = plot3(true_traj(1,10)+[0 scale*true_traj(1,4)],...
%                    true_traj(1,11)+[0 scale*true_traj(1,5)],...
%                    true_traj(1,12)+[0 scale*true_traj(1,6)],'g','LineWidth',LW);
% true_att.z = plot3(true_traj(1,10)+[0 scale*true_traj(1,7)],...
%                    true_traj(1,11)+[0 scale*true_traj(1,8)],...
%                    true_traj(1,12)+[0 scale*true_traj(1,9)],'b','LineWidth',LW);
% est_att.x = plot3(est_traj(1,10)+[0 scale*est_traj(1,1)],...
%                   est_traj(1,11)+[0 scale*est_traj(1,2)],...
%                   est_traj(1,12)+[0 scale*est_traj(1,3)],':r','LineWidth',LW);
% est_att.y = plot3(est_traj(1,10)+[0 scale*est_traj(1,4)],...
%                   est_traj(1,11)+[0 scale*est_traj(1,5)],...
%                   est_traj(1,12)+[0 scale*est_traj(1,6)],':g','LineWidth',LW);
% est_att.z = plot3(est_traj(1,10)+[0 scale*est_traj(1,7)],...
%                   est_traj(1,11)+[0 scale*est_traj(1,8)],...
%                   est_traj(1,12)+[0 scale*est_traj(1,9)],':b','LineWidth',LW);
% axis equal; grid on
% zlims = zlim();
% zlim(1.1*zlims);
% for ii = 1:size(est_traj,1)
%     % Update the poses:
%     set(true_att.x,'XData',true_traj(ii,10)+[0 scale*true_traj(ii,1)],...
%                    'YData',true_traj(ii,11)+[0 scale*true_traj(ii,2)],...
%                    'ZData',true_traj(ii,12)+[0 scale*true_traj(ii,3)])
%     set(true_att.y,'XData',true_traj(ii,10)+[0 scale*true_traj(ii,4)],...
%                    'YData',true_traj(ii,11)+[0 scale*true_traj(ii,5)],...
%                    'ZData',true_traj(ii,12)+[0 scale*true_traj(ii,6)])
%     set(true_att.z,'XData',true_traj(ii,10)+[0 scale*true_traj(ii,7)],...
%                    'YData',true_traj(ii,11)+[0 scale*true_traj(ii,8)],...
%                    'ZData',true_traj(ii,12)+[0 scale*true_traj(ii,9)])
%                
%     set(est_att.x,'XData',est_traj(ii,10)+[0 scale*est_traj(ii,1)],...
%                   'YData',est_traj(ii,11)+[0 scale*est_traj(ii,2)],...
%                   'ZData',est_traj(ii,12)+[0 scale*est_traj(ii,3)])
%     set(est_att.y,'XData',est_traj(ii,10)+[0 scale*est_traj(ii,4)],...
%                   'YData',est_traj(ii,11)+[0 scale*est_traj(ii,5)],...
%                   'ZData',est_traj(ii,12)+[0 scale*est_traj(ii,6)])
%     set(est_att.z,'XData',est_traj(ii,10)+[0 scale*est_traj(ii,7)],...
%                   'YData',est_traj(ii,11)+[0 scale*est_traj(ii,8)],...
%                   'ZData',est_traj(ii,12)+[0 scale*est_traj(ii,9)])
%     
%     % Update the attitudes:
%     
%     drawnow
%     pause(.1)
% end