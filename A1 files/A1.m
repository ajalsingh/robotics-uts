%% Setup environment
clc;
clear;
[ur3, ur5, bricks] = worldsetup();
bricks = bricks';
%% 
drop = bricks(5,:);
drop(1,2) = drop(1,2)+0.2;
retrieveanddrop(ur5.model,bricks(3,:),[0.1,-0.2,0]); 
% retrieveanddrop(ur3.model,bricks(4,:),[0.1,-0.2,0]);