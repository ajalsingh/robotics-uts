%%
clear all
close all
clc

% Given a 4 DOF planar robot with the base at joint 1 and the end Effector
% 1 m from the last joint and 40 triangles in an environment, how many line
% to triangle colliso checks are required?


Dof = 3;
distance =1;
triangles = 50;
checks =Dof*triangles

%%
clear all
close all
clc
set(0,'DefaultFigureWindowStyle','docked')
view (3)
axis equal
camlight

% Get rectangular prism and isCollision. Create a mdl_planar3 and a
% rectangular prim  with [v,f,fn] = rebtangular prism([..]). Create a 50
% step trajectory with jtraj from q1 to q2  and use "result " = iscollisoin
% .. determine poses in the trajectory that is in collision.

mdl_planar3;
% [v,f,fn] = RectangularPrism([2,-1,-1],[3,1,1]);

q1 = [pi/3,0,0];
q2 = [-pi/3,0,0];
steps = 50;
q = jtraj(q1,q2,steps);
result = false;
for i=1:numel(q)
    p3.plot(q(i,:));
    
    %[result,qIntersect] = IsCollision(p3,q,f,v,fn)
    J = p3.jacob0(p3.getpos);
    measureOfManip = sqrt(det(J(1:2,:)*J(1:2,:)'));
    [result,qIntersect] = IsCollision(p3,q,f,v,fn);
    if(q(i,:) ==qIntersect)
        disp('There is an collision at : ')
        disp(qIntersect)
        p3.plot(qIntersect)
        break
    end
end

%% Given a collision detection ellipsoid for a robot centered at ...
% and the radii ... and the surface with points as as follows [x,y]= mesh
% grid ... Z = x how many points are inside the ellipsoid

clear al
close all
clc
set(0,'DefaultFigureWindowStyle','docked')
view (3)
centerPoint = [3,2,1];
radii = [1,2,3];
[X,Y] = meshgrid(-10:1:10,-10:1:10);
Z = X;
[x,y,z]= ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
ellipsoid_h = surf(x,y,z)
hold on
surf(X,Y,X)
cubePoints = [X(:),Y(:),Z(:)];
algebraicDist = ((cubePoints(:,1)-centerPoint(1))/radii(1)).^2 ...%.^2 every element power of 2 in matrices
              + ((cubePoints(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((cubePoints(:,3)-centerPoint(3))/radii(3)).^2;
pointsInside = find(algebraicDist < 1);
display(['There are ', num2str(size(pointsInside,1)),...
    ' points inside']);
%% Load a model of a 3 Link planar robot with planar. Which of these poses 
% is closest to a singularity 



clear al
close all
%clc


set(0,'DefaultFigureWindowStyle','docked')
view (3)
mdl_planar3;
%q = [0 1.5708 -1.5708];
%q = [0.7854 0.1745 -0.1745];
%q = [0.7854 -0.7854 0.7854];
%q = [0 -0.7854 -0.7854];
p3.plot(q);

jacobian = p3.jacob0(q);

p3.vellipse(q);
p3.maniplty(q, 'yoshikawa');
measureOfManip = sqrt(det(jacobian(1:2,:)*jacobian(1:2,:)'))
axis equal

%% Load model of puma 560. which of the poses is closes to singularity.

% checkkk

clear;
close all
clc
set(0,'DefaultFigureWindowStyle','docked')
view (3)

mdl_puma560;

% q = [0 1.5708 -3.1416 0 0 0];
% q = [0 0.01 0 0 0 0];
% q = [0 2.1677 -2.7332 0 -0.9425 0];
q = [0 0.7854 3.1416 0 0.7854 0];
p560.plot(q);

jacobian = p560.jacob0(q);
inv_jacobian = inv(jacobian);

p560.vellipse(q);
measureOfManip=p560.maniplty(q, 'yoshikawa')
%measureOfManip = sqrt(det(jacobian(1:2,:)*jacobian(1:2,:)'))
limit = p560.islimit(q);
axis equal

%% Create camera on UR10

clear all
close all
clc
set(0,'DefaultFigureWindowStyle','docked')
view (3)
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5,'resolution', [1024 1024], 'centre', [512 512],'name', 'UR10camera');
pStar = [600 300 300 600; 300 300 600 600];
q0 =  [1.6; -1; -1.2; -0.5; 0; 0];
robot = UR10();
P=[2,2,2,2; -0.3,0.3,0.3,-0.3; 1.3,1.3,0.7,0.7]; 
plot_sphere(P, 0.05, 'g')

lighting gouraud
light
transform = robot.model.fkine(q0);
robot.model.animate(q0');
drawnow();

cam.T = transform;
cam.plot_camera('Tcam',transform, 'label','scale',0.15)
%Project points to the image
p = cam.plot(P, 'Tcam', transform);
uv = cam.plot(P);
e = pStar-uv