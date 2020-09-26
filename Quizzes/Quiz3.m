%% 5DOF Planar
clear;
clc;
close all;

L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);    
L4 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L5 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);

robot = SerialLink([L1 L2 L3 L4 L5],'name','myRobot');                     
q = deg2rad([30 -60 45 -30 0]);                                                     % Create a vector of initial joint angles        

scale = 0.5;
workspace = [-2 2 -2 2 -0.05 2];                                       % Set the size of the workspace when drawing the robot
robot.fkine(q)
%% PUMA distance to point
clear;
clc;
close all;

mdl_puma560;

% point provided
otherPoint = [0.6304 -2.149 0.6223];

% adjust joint states
q = deg2rad([0, 45, -80, 0, 45, 0]);
T = p560.fkine(q);
distance = norm(T(1:3,4)' - otherPoint)
%%
p560.plot(q);
view(3);
hold on;

r = 1 ;
[x,y,z] = sphere;
surf(r*x+T(1,4),r*y+T(2,4),r*z+T(1,4));
axis equal

for i=-2.5:0.01:2.5
    scatter3(1,i,1,'filled')
end
%%
for i=-2.5:0.00001:2
    distance = norm(T(1:3,4)' - [1 i 1]);
    if round(distance,3) == r
        ans = i
        break;
    end
end

%% A1 Trajectory
clear;
clc;
close all;

q1= [pi/10, pi/7, pi/5, pi/3, pi/4, pi/6];
q2 = [-pi/6, -pi/3, -pi/4, -pi/8, -pi/7, -pi/10];

steps = 35;

interpolation = 2;

switch interpolation
    case 1
        qMatrix = jtraj(q1,q2,steps);
    case 2
        s = lspb(0,1,steps);                                             	% First, create the scalar function
        qMatrix = nan(steps,6);                                             % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;                   	% Generate interpolated joint angles
            end
    otherwise
        error('interpolation = 1 for Quintic Polynomial, or 2 for Trapezoidal Velocity')
end
        
velocity = zeros(steps,6);
acceleration  = zeros(steps,6);
for i = 2:steps
    velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);                          % Evaluate relative joint velocity
    acceleration(i,:) = velocity(i,:) - velocity(i-1,:);                    % Evaluate relative acceleration
end
velocity
max(abs(velocity(:)))
%% Puma560 point in end-effector coordinate frame
clear;
clc;
close all;

mdl_puma560;


% chnage the following:
p560.base = transl(0,0,0.0);
q = deg2rad([90, 30, -80, 0, 45, 0]);
ball = transl(0.4,-0.2,0.7);               %target location in global frame
%

T = p560.fkine(q);

% ball position with respect to end-effector
Te = inv(T) * ball
%%
ball = transl(-0.4611, 0.6501 ,0.2822);
Tr = inv(Te)/ball
% inv(Tr)

%% PUMA560 ikine
clear;
clc;
close all;

mdl_puma560;
T = transl([0.6 0.1 0.1]);
p560.ikine(T,qn, [1 1 1 0 0 0])


%% PUMA560 bolted to floor, find distance from end-effector to floor(z axis)
clear;
clc;
close all;

mdl_puma560;

p560.base = transl(0,0,0.0);
q =  [pi/12,0,-pi/2,0,0,0];

fkine = p560.fkine(q);
floorDist = fkine(3,4)

%% Sawyer robot
clear;
clc;
close all;

% https://sdk.rethinkrobotics.com/intera/Hardware_Components#Sawyer_Arm_Specifications
L1 = Link('d',0.317,    'a',-0.081, 'alpha',-pi/2,  'qlim',[-(pi-pi/36) (pi-pi/36)],    'offset', 0);
L2 = Link('d',0.1925,   'a',0,      'alpha',pi/2,   'qlim',[-(pi-pi/36) (pi-pi/36)],    'offset', pi/2);
L3 = Link('d',0.4,      'a',0,      'alpha',pi/2,   'qlim',[-(pi-pi/36) (pi-pi/36)],    'offset', 0);    
L4 = Link('d',0.1685,  'a',0,      'alpha',-pi/2,   'qlim',[-(pi-pi/36) (pi-pi/36)],    'offset', 0);
L5 = Link('d',0.4,      'a',0,      'alpha',-pi/2,  'qlim',[-(pi-pi/36) (pi-pi/36)],    'offset', 0);
L6 = Link('d',0.1363,   'a',0,      'alpha',pi/2,   'qlim',[-(pi-pi/36) (pi-pi/36)],    'offset', 0);
L7 = Link('d',0.13375,  'a',0,      'alpha',0,      'qlim',[-(1.5*pi) (1.5*pi)],        'offset', 0);

robot = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name','myRobot');                     
q = zeros(1,7);                                                     % Create a vector of initial joint angles        

scale = 0.5;
workspace = [-1.5 1.5 -1.5 1.5 -0.05 1.5];                                       % Set the size of the workspace when drawing the robot
robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot

%% PUMA line plane

%Create a puma 560.
mdl_puma560;
%The robot is at q = [pi/20,0,-pi/2,0,0,0].
q = [pi/12,0,-pi/2,0,0,0]%Q INPUT

%Determine where a ray cast from the Z axis (the approach vector) of the end effector intersects with a planar wall. (i.e. normal = [-1 0 0], point = [1.2 0 0]).
endEffectorTr = p560.fkine(q)
p560.teach(q)

hold on

normal = [-1,0,0] ;%Q INPUT
point= [1.8,0,0] ;%Q INPUT
planeXntersect = 1.8;%Q INPUT

planeBounds = [planeXntersect-eps,planeXntersect+eps,-2,2,-2,2]; 
[Y,Z] = meshgrid(planeBounds(3):0.1:planeBounds(4),planeBounds(5):0.1:planeBounds(6));
X = repmat(planeXntersect,size(Y,1),size(Y,2));
surf(X,Y,Z);

rayEndTr = endEffectorTr * transl(0,0,10);
point1OnLine = endEffectorTr(1:3,4)';
point2OnLine = rayEndTr(1:3,4)';

[intersectionPoint,check] = LinePlaneIntersection(normal,point,point1OnLine,point2OnLine)
%% 3 link plane
mdl_3link3d;
q = [-pi/6,0,0];
normal = [3.1,0,0];
% [intersectP,check] = LinePlaneIntersection(normal,point,tr(1:3,4,i)',tr(1:3,4,i+1)');
R3.plot(q)
