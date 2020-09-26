%% Hyper2d self collision
clear
clc
close all

%options
q1= [pi, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, pi/3, -pi/3];
q2= [pi, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, -pi/2, -pi/3, -pi/3];
q3= [-pi, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, -pi/3, pi/3, -pi/3];
q4= [-pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, pi/3, -pi/3];

mdl_hyper2d;

view(3);

%cycle from q1-q4 to check for collision
h2d.plot(q3);

%% Puma560 ball pose with respect to end-effector coordinate frame
clear;
clc;
close all;

mdl_puma560;


% chnage the llowing:
p560.base = transl(0,0,0.8);
q = deg2rad([-90, 45, 45, 0, 45, 0]);
ball = transl(0.5,0,0.6);
%

epose = p560.fkine(q);

% ball position with respect to end-effector
pose = inv(epose) * ball

%% PUMA560 jacobian
clear;
clc;
close all;

mdl_puma560;

q = pi/18*ones(1,6);

p560.teach;
p560.jacob0(q)

%% PUMA560 bolted to floor, find distance from end-effector to floor
clear;
clc;
close all;

mdl_puma560;

p560.base = transl(0,0,0.8);
q = [0,0,pi/10,0,0,0];

fkine = p560.fkine(q);
fkine(3,4)

%% PUMA560 will the follwoing be true
clear;
clc;
close all;

mdl_puma560;

p1a = fkine(p560,ikine(p560,fkine(p560,[pi/4,0,0,0,0,0])));
p1b = fkine(p560,[pi/4,0,0,0,0,0]);
p2a = fkine(p560,ikine(p560,fkine(p560,[pi/4,0,0,0,0,0]),[pi/4,0,0,0,0,0]));
p2b = fkine(p560,[pi/4,0,0,0,0,0]);

if p1a==p1b
    resa = 1
else
    res = 0
end

if p2a ==p2b
    resb=1
else
    resb=0
end
%% Baxter distance between left and right
clear;
clc;
close all;

mdl_baxter;

config = 1; % 2 for both end-effectors, 1 for the other

if config==1
    %change below
    qLeft = [3*pi/10,0,0,0,0,0,2*pi/10];
    %
    
    left_fkine = left.fkine(qLeft);
    distance = norm(left_fkine(1:3,4) - right.base(1:3,4))
else
    %change below
    qLeft = [pi/10,0,0,0,0,0,pi/10];
    qRight = [-pi/10,0,0,0,0,0,-pi/10];
    %
    
    left_fkine = left.fkine(qLeft);
    right_fkine = right.fkine(qRight);
    distance = norm(right_fkine(1:3,4) - left_fkine(1:3,4))
end

%% Baxter ikine - dont know what to do
clear;
clc;
close all;

mdl_baxter;
q = zeros(1,7);
qr = left.ikine(transl(-0.55,-0.4,0.1),q,[1,1,1,0,0,0])
left.plot(qr)
tr = left.fkine(qr);
newqr = left.ikine(tr,q,[1,1,1,0,0,0])