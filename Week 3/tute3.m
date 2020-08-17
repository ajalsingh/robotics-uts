% L1 = Link('d',___,'a',___,'alpha',___,'offset',___,'qlim', [__,__]);



L1 = Link('d',0.475,'a',0.18,'alpha',-pi/2,'offset',0,'qlim', deg2rad(170))

L2 = Link('d',0,'a',0.385,'alpha',0,'offset',-pi/2,'qlim', deg2rad(135))

L3 = Link('d',0,'a',-0.1,'alpha',pi/2,'offset',pi/2,'qlim', deg2rad(165))

L4 = Link('d',0.445,'a',0,'alpha',-pi/2,'offset',0,'qlim', deg2rad(185))

L5 = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim', deg2rad(120))

L6 = Link('d',0.09,'a',0,'alpha',0,'offset',0,'qlim', deg2rad(360))

robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','DensoVM6083');
%% 
% create 1x6 vector of joingt angles set at 0
q = zeros(1,6)

% create robot model
figure(2)
robot.plot(q)

robot.teach();

q = robot.getpos()
%%
T = robot.fkine(q)
q = robot.ikine(T); % N.B. DOES NOT WORK FOR 3DOF MANIPULATORS
J = robot.jacob0(q);
J = J(1:3,1:3); % For the 3-Link robots, we only need the first 3 rows. Ignore this line for 6DOF robots.
inv(J)
q = zeros(1,6)
J = robot.jacob0(q)
inv(J)
robot.vellipse(q);