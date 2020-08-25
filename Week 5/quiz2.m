%% Hyper2d self collision
clear
clc
close all

%options
q1= [pi pi/2 -pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2 pi/3 -pi/3];
q2= [pi pi/2 -pi/2 pi/2 -pi/2 pi/2 -pi/2 -pi/2 -pi/3 -pi/3];
q3= [-pi pi/2 pi/2 pi/2 -pi/2 pi/2 -pi/2 -pi/3 pi/3 pi/3];
q4= [-pi/2 pi/2 pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2 pi/3 -pi/3];

mdl_hyper2d;

view(3);
h2d.plot(q4);

%% Puma560 ball pose with respect to end-effector coordinate frame
clear;
clc;
close all;

mdl_puma560;

p560.base = transl(0,0,0.8);
q = deg2rad([45 45 45 0 45 0]);

epose = p560.fkine(q);
ball = transl(0.7,0,0.4) * trotx(pi/2)

% ball position with respect to end-effector
pose = ball - epose

%% PUMA560 jacobian
clear;
clc;
close all;

mdl_puma560;

q = p560.qlim(:,1)';

p560.teach;
p560.jacob0(q)

%% PUMA560 bolted to floor, find distance from end-effector to floor
clear;
clc;
close all;

mdl_puma560;

p560.base = transl(0,0,0.8);
q = [0 0 pi/10 0 0 0];

fkine = p560.fkine(q);
fkine(3,4)

%% PUMA560 will the follwoing be true
clear;
clc;
close all;

mdl_baxter;

p1a = 0
p1b = 0
p2a = 0
p2b = 0
%% Baxter distance between left and right
clear;
clc;
close all;

mdl_baxter;

config = 2; % 2 for both end-effectors, 1 for the other

if config==1
    qRight = [-9*pi/10 0 0 4*pi/9 0 0 0];

    right_fkine = right.fkine(qRight);
    distance = norm(right_fkine(1:3,4) - left.base(1:3,4))
else
    qLeft = [pi/10 0 0 0 0 0 pi/10];
    qRight = [-pi/10 0 0 0 0 0 -pi/10];
    
    left_fkine = left.fkine(qLeft);
    right_fkine = right.fkine(qRight);
    distance = norm(right_fkine(1:3,4) - left_fkine(1:3,4))
end

%% Baxter ikine
clear;
clc;
close all;

mdl_baxter;
q = zeros(1,7);
qr = left.ikine(transl(-0.0,-0.0,0),q,[1,1,1,0,0,0]);
tr = left.fkine(qr)
newqr = left.ikine(tr);
left.fkine(newqr)