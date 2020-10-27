function  main
%% load robot and drum
clear;
close all;
clc;
set(0,'DefaultFigureWindowStyle','docked')
view (3)
camlight

% load puma560
mdl_puma560;

%set base
robot_base = [1.262, 1.189, 1];
p560.base = transl(robot_base);

% set tool
tool_offset = 0.2*tan(pi/4);
p560.tool = transl([0,0,tool_offset]);

% plot robot
p560.plot([0 0 -pi/2 0 0 0])
xlim([-0.5,2.5]);
ylim([0,2]);
zlim([0,2]);
hold on; 

% Load Drum
% Place drum
[f,v,data] = plyread('Drum.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

xoffset = 0.5;
yoffset = 1;
drum_transform = transl([xoffset, yoffset, 0]);

% Then plot the trisurf
trisurf(f,v(:,1)+xoffset,v(:,2)+yoffset, v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

m = Manipulator;
%% Blast window
p560.base;
drum_transform;

x = p560.base(1,4) - drum_transform(1,4);
y = drum_transform(2,4) - p560.base(2,4);
z = drum_transform(3,4) - p560.base(3,4);

pose_1 = [0.8 1 0.6];
% pose_1 = [y x z];

m.MoveSingleArm(p560, pose_1)
% q = p560.ikcon(transl(pose_1), p560.getpos);
% p560.animate(q)
% p560.fkine(p560.getpos)
end

