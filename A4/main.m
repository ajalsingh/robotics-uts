function  main
%% load robot and set base pos
clear;
close all;
clc;
set(0,'DefaultFigureWindowStyle','docked')
view (3)
camlight

mdl_puma560;
robot_base = [1.262, 1.189, 1];
p560.base = transl(robot_base);
p560.plot(zeros(1,6))
xlim([-0.5,2.5]);
ylim([0,2]);
zlim([0,2]);
hold on; 

%% Load Drum
% Place drum
[f,v,data] = plyread('Drum.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

xoffset = 0.5;
yoffset = 1;
drum_transform = transl([xoffset, yoffset, 0])

% Then plot the trisurf
trisurf(f,v(:,1)+xoffset,v(:,2)+yoffset, v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
end

