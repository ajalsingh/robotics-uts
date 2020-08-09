%% Section 1
%1.1 move up
tranimate(eye(4),transl([0,0,10]),'fps',25)

%1.2 roll -10 deg x dir
trStart = transl([0,0,10])
trEnd = transl([0,0,10]) * trotx(-10 * pi/180)
tranimate(trStart,trEnd,'fps',25);

%1.3 translate
trStart = trEnd;
trEnd = transl([0,2,10])* trotx(-10 * pi/180);
tranimate(trStart,trEnd,'fps',25)

%1.4 rollx dir even out
trStart = trEnd;
trEnd = trStart * trotx(10 * pi/180);
tranimate(trStart,trEnd,'fps',25);

%1.5 rolly 20 deg
trStart = trEnd;
trEnd = trStart * troty(20 * pi/180);
tranimate(trStart,trEnd,'fps',25);

%1.6 translate
trStart = trEnd;
trEnd = transl(2,2,10)* troty(20 * pi/180);
tranimate(trStart,trEnd,'fps',25);

%1.7 rolly even out
trStart = trEnd;
trEnd = trStart * troty(-20 * pi/180);
tranimate(trStart,trEnd,'fps',25);

%1.8
trStart = trEnd;
trEnd = transl(2,2,0);
tranimate(trStart,trEnd,'fps',25);

%1.9

%1.10

%% Section 2

% Create and plot an instance of RobotCows using: 
cowHerd = RobotCows();

% You can check the default cow count with: 
cowHerd.cowCount

% 2.5 And plot the random walk movement of them with: 
cowHerd.PlotSingleRandomStep();

% 2.6 Increase the number of cows, by first 
clf; 
clear all; 

% then recreate a herd by passing in a new cowCount as follows: 
cowHerd = RobotCows(10);

% 2.7 Test many random steps
numSteps=100;
delay=0.01;
cowHerd.TestPlotManyStep(numSteps,delay);

% 2.8 Query the location of the 2nd cow with: 
cowHerd.cow{2}.base

%% Section 3
clf;
% 3.1 Create a cow herd with more than 2 cows.
cowHerd = RobotCows(3);

% 3.2 Plot the transform of the UAV starting at the origin
uavTR{1} = eye(4);

% 3.3 Determine the transform between the UAV and each of the cows 
for cowIndex = 1:cowHerd.cowCount
    display(['At trajectoryStep ',num2str(1),' the UAV TR to cow ',num2str(cowIndex),' is ']);
    display(num2str(inv(uavTR{1}) * cowHerd.cow{cowIndex}.base));
end  
cowHerd.PlotSingleRandomStep();

%3.5
uavTR{2} = transl([0,0,10]);
uavTR{3} = transl([0,0,10]) * trotx(-10 * pi/180);
uavTR{4} = transl([0,2,10]) * trotx(-10 * pi/180);
uavTR{5} = transl([0,2,10]);
uavTR{6} = transl([0,2,10]) * troty(20 * pi/180);
uavTR{7} = transl([2,2,10]) * troty(20 * pi/180);
uavTR{8} = transl([2,2,10]);
uavTR{9} = transl([2,2,0]);

for trajectoryStep = 1:size(uavTR,2)-1
    rpy = tr2rpy(uavTR{trajectoryStep+1});
    
    tranimate(uavTR{trajectoryStep},uavTR{trajectoryStep+1},'fps',25)
    
    try delete(text_h);end;
    message = "rpy: " + sprintf([num2str(round(rpy(1,:),2,'significant')),'\n' ])... 
                + "qua: " + sprintf([num2str(round(uavTR{trajectoryStep+1}(1,:),2,'significant')),'\n' ]);
    text_h = text(2, 23, message, 'FontSize', 10, 'Color', [.6 .2 .6]);
      
    %3.4
    cowHerd.PlotSingleRandomStep();
    
    for cowIndex = 1:cowHerd.cowCount
        display(['At trajectoryStep ',num2str(trajectoryStep+1),' the UAV TR to cow ',num2str(cowIndex),' is ']);
        display(num2str(inv(uavTR{trajectoryStep+1}) * cowHerd.cow{cowIndex}.base));
    end    
end
%% 3.6 hover UAV above fat cow
clf;
clear all;
clc;
cowHerd = RobotCows(1);
uavTRStart = eye(4);
uavTRGoal = transl([0,0,5]);
tranimate(uavTRStart,uavTRGoal,'fps',25)

for i = 1:10
    cowHerd.PlotSingleRandomStep();
    uavTRStart = uavTRGoal;
    uavTRGoal = cowHerd.cow{1}.base * transl(0,0,5);
    tranimate(uavTRStart,uavTRGoal,'fps',100);
end

%% Section 4
%4.1
clf;
clear all;
clc
% 4.1 and 4.2: Define the DH Parameters to create the Kinematic model
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
robot = SerialLink([L1 L2 L3],'name','myRobot')                     % Generate the model

workspace = [-4 4 -4 4 -4 4];                                       % Set the size of the workspace when drawing the robot

scale = 0.5;

q = zeros(1,3);                                                     % Create a vector of initial joint angles

robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot

% 4.3 Manually play around with the robot
robot.teach;                                                        % Open a menu to move the robot manually

% 4.4 Get the current joint angles based on the position in the model
q = robot.getpos();  

% 4.5 Get the joint limits
robot.qlim 