function [  ] = Lab6Q1() 
clc;
clear;
close all;
%% 1.1  make robot
robot = SchunkUTSv2_0();
q = [0,pi/2,0,0,0,0];
robot.plot3d(q);
view(3);
camlight;
hold on;

%% 1.2 point laser
tr = robot.fkine(q)
startP = tr(1:3,4);
endP = tr(1:3,4) + 1.9594 * tr(1:3,3);
line1_h = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r');
plot3(endP(1),endP(2),endP(3),'r*');
axis equal;

v1 = endP;

%% 1.3a point laser again
q = [ pi/10, pi/2, 0, 0, 0, 0 ] ;
animate(robot,q);
tr = robot.fkine(q)
startP = tr(1:3,4);
endP = tr(1:3,4) + 2.4861 * tr(1:3,3);
line1_h = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r');
plot3(endP(1),endP(2),endP(3),'r*');
axis equal;

v2 = endP;

%% 1.3b point the laser one last time
q = [ -pi/10, 5*pi/12, 0, 0, 0, 0 ];
animate(robot,q);
tr = robot.fkine(q)
startP = tr(1:3,4);
endP = tr(1:3,4) + 1.9132 * tr(1:3,3);
line1_h = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r');
plot3(endP(1),endP(2),endP(3),'r*');
axis equal;

v3 = endP;

%% 1.4 Mesh
triangleNormal = unit(cross((v1-v2),(v2-v3)));

% Make a plane at the orgin, to rotate later
basePlaneNormal = [-1,0,0];
[Y,Z] = meshgrid(-2:0.1:2,-2:0.1:2  );
sizeMat = size(Y);
X = repmat(0,sizeMat(1),sizeMat(2));

% Rotation axis: to rotate the base plane around
rotationAxis = cross(triangleNormal,basePlaneNormal);
rotationAxis = rotationAxis / norm(rotationAxis);

% Rotation angle: how much to rotate base plane to make it match triangle plane
rotationRadians = acos(dot(triangleNormal,basePlaneNormal));

% Make a transform to do that rotation
tr = makehgtform('axisrotate',rotationAxis,rotationRadians);

% Find a central point of the triangle
verts = [v1' ; v2';v3'];
trianglePoint = sum(verts)/3;

% Plot the point/normal of the triangle
plot3(trianglePoint(1),trianglePoint(2),trianglePoint(3),'g*');
plot3([trianglePoint(1),trianglePoint(1)+triangleNormal(1)] ...
     ,[trianglePoint(2),trianglePoint(2)+triangleNormal(2)] ...
     ,[trianglePoint(3),trianglePoint(3)+triangleNormal(3)],'b');
 drawnow();
 pause(1);
 
% Transform the points on the default plane, to matches the actual triangle
points = [X(:),Y(:),Z(:)] * tr(1:3,1:3) + repmat(trianglePoint,prod(sizeMat),1);
X = reshape(points(:,1),sizeMat(1),sizeMat(2));
Y = reshape(points(:,2),sizeMat(1),sizeMat(2));
Z = reshape(points(:,3),sizeMat(1),sizeMat(2));

% Make points where Z<0 to be = zero
Z(Z<0)= 0;
surf(X,Y,Z);
pause(1);

%% 1.5 
q = [0,pi/2,0,0,0,0];
robot.animate(q);

tr = robot.fkine(q);


%% Q2 
%% 2.1 Create vertices that represent an ellipsoid with radii (rx=3,ry=2,rz=1) centered at [xc,yc,zc] = [0,0,0]. 
clf;
centerPoint = [0,0,0];
radii = [3,2,1]
[X,Y,Z] = ellipsoid(centerPoint (1),centerPoint (2),centerPoint(3),radii(1),radii(2),radii(3));

%% 2.2 plot it
surf(X,Y,Z)

%% 2.3 Cube 1.5m sides 
[Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
X = repmat(0.75,size(Y,1),size(Y,2));
end

