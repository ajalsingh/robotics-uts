function [ur3,ur5] = worldsetup(ur3pos, ur5pos, bricks)
%WORLDSETUP Summary of this function goes here
%   Detailed explanation goes here
%% Setup environment and place robots

addpath('UR3/');

ur5 = LinearUR5(false);
ur3 = UR3();
%adjust robot base transform
ur5.model.base = transl(ur3pos) * trotx(pi/2);
ur3.model.base = transl(ur5pos);
ur3.PlotAndColourRobot();
ur5.PlotAndColourRobot();
% animate(ur5.model, [0 0 0 pi/2 0 0 0]);
hold on;


% Conveyor file download: https://grabcad.com/library/conveyor-food-processing-roller-system-with-tripod-supports
[f,v,data] = plyread('conveyor.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
trisurf(f,v(:,1)+1,v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
trisurf(f,v(:,1)-1,v(:,2), v(:,3)-0.2 ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

% Setup bricks
[f,v,data] = plyread('Brick.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;


for rows=1:size(bricks,1)
    trisurf(f,v(:,1)+bricks(rows,1),v(:,2)+bricks(rows,2), v(:,3)+bricks(rows,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
end
end

