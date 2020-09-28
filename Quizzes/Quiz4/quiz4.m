%% Theory Questions

% Manipulability is the measure of how spherical an ellipsoid is
% Manipulability, m = sqrt(det(J*J^T)),     p179 of textbook

%% XDOF Planar, collision checks
clear
close all
clc

% Given a 4 DOF planar robot with the base at joint 1 and the end Effector
% 1 m from the last joint and 40 triangles in an environment, how many line
% to triangle collison checks are required?


Dof = 4;
distance = 1;
triangles = 40;
checks =Dof*triangles

%% Get rectangularPrism and IsCollision
clear
close all
clc
set(0,'DefaultFigureWindowStyle','docked')
view (3)
axis equal
camlight

mdl_planar3;

%%%%%%%%%%% input required below
[v,f,fn] = RectangularPrism([2,-1,-1],[3,1,1]);
steps = 50;
q1 = [pi/3 0 0];
q2 = [-pi/3 0 0];
%%%%%%%%%%%

qMatrix = jtraj(q1,q2,steps);

for i = 1: steps
    p3.plot(qMatrix(i,:))
    result = IsCollision(p3,qMatrix(i,:),f,v,fn);
    if result == 1
        break;
    end
end

qMatrix(i,:)

%% Collision detection ellipsoid
clear 
close all
clc
set(0,'DefaultFigureWindowStyle','docked')
view (3)

%%%%%% input
centre = [3 2 1];
radii = [1 2 3];
[X Y] = meshgrid(-10:1:10, -10:1:10);
%%%%%%

Z = X;
[x,y,z]= ellipsoid(centre(1), centre(2), centre(3), radii(1), radii(2), radii(3));
ellipsoid_h = surf(x,y,z);
hold on
surf(X,Y,Z)
alpha(0.5);
cubePoints = [X(:),Y(:),Z(:)];
algebraicDist = ((cubePoints(:,1)-centre(1))/radii(1)).^2 ...%.^2 every element power of 2 in matrices
              + ((cubePoints(:,2)-centre(2))/radii(2)).^2 ...
              + ((cubePoints(:,3)-centre(3))/radii(3)).^2;
pointsInside = find(algebraicDist < 1);
display(['There are ', num2str(size(pointsInside,1)),...
    ' points inside']);

%% Load a model of a 3 Link planar robot with planar. Which of these poses is closest to a singularity 
clear 
close all
clc
set(0,'DefaultFigureWindowStyle','docked')
view (3)

mdl_planar3;

q(1,:) = [0 1.5708 -1.5708];
q(2,:) = [0.5 0.5 0.5];
q(3,:) = [0 -0.7854 -0.7854];
q(4,:) = [0.7854 0.7854 0.7854];

index=0;
prev=10;
for i=1:size(q,1)
    p3.plot(q(i,:));

    jacobian = p3.jacob0(q(i,:));

    p3.vellipse(q(i,:));
    p3.maniplty(q(i,:), 'yoshikawa');
    measureOfManip = sqrt(det(jacobian(1:2,:)*jacobian(1:2,:)'))
    axis equal
    
    if prev > measureOfManip
        index = i;
        prev = measureOfManip;
    end
end

index
%% Load model of puma 560. which of the poses is closes to singularity.
clear;
close all
clc
set(0,'DefaultFigureWindowStyle','docked')
view (3)

mdl_puma560;

q(1,:) = [0 pi/2 -pi 0 0 0];
q(2,:) = [0 0.01 0 0 0 0];
q(3,:) = [0 2.1677 -2.7832 0 -0.9425 0];
q(4,:) = [0 0.7854 pi 0 0.7854 0];

index=0;
prev=10;
for i=1:size(q,1)
    p560.plot(q(i,:));

    jacobian = p560.jacob0(q(i,:));
    inv_jacobian = inv(jacobian);

    p560.vellipse(q(i,:));
    measureOfManip = p560.maniplty(q(i,:), 'yoshikawa')
    limit = p560.islimit(q(i,:));
    axis equal
    
    if prev > measureOfManip
        index = i;
        prev = measureOfManip;
    end
end
index


%% Camera
clear
close all
clc
set(0,'DefaultFigureWindowStyle','docked')
view (3)

%%%%%% input
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, 'resolution', [1024,1024], 'centre', [512, 512]);
q = [1.6; -1; 1.2; -0.5; 0; 0];
pStar = [600 300 300 600; 300 300 600 600];
P = [2,2,2,2; -0.3,0.3,0.3,-0.3; 1.3,1.3,0.7,0.7]; 
%%%%%%

robot = UR10;
robot.model.animate(q');
plot_sphere(P, 0.05, 'g')
cam.T = robot.model.fkine(q);
cam.plot_camera('Tcam',cam.T, 'label','scale',0.15)
%Project points to the image
p = cam.plot(P, 'Tcam', cam.T);
uv = cam.plot(P);
e = pStar-uv
