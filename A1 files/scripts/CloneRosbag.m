classdef CloneRosbag
    %CLONEROSBAG Match movements of recorded rosbag
    %   The rosbag video on utsonline was used to estimate poses
    
    properties
        robot;  % robot
        steps;  % number of steps in each translation
    end
    
    methods
        %%
        function self = CloneRosbag()
            %CLONEROSBAG Construct an instance of this class
            %   Create robot and set start joint angles
            close all;
            clc;

            ur3 = UR3;
            ur3.PlotAndColourRobot;

            self.robot = ur3.model;
            self.steps = 50;

            qStart= [-1.2217   -0.7854    1.9199    0.5236    1.5708         0];
            animate(self.robot, qStart);
            
            self.simulate;
        end
        %%
        function simulate(self)
            %METHOD1 Simulate poses 
            
            %   The following poses where estimated using the rosbag video
            %   on utsonline and the teach method
            q1= [-1.3963   -0.8727    2.0944    0.3491    1.5708         0];
            q2= [-pi/6      pi/9      pi*7/18         0     pi/2         0];
            q3= [-0.6981    0.5236    0.9599    0.1745    1.5708         0];
            q4= [-1.8326    0.1745    2.3562   -0.9599    1.5708         0];
            q5= [-1.3963         0    1.5708    0.4363    0.8727         0];
            q6= [-3.4907   -0.1745    2.4050   -0.4363    2.0071         0];
            q7= [-5.4978         0    1.2217    1.9199    1.5708         0];
            q = [q1;q2;q3;q4;q5;q6;q7];
            
            % simulate
            for i=1:length(q)
                qMatrix = jtraj(self.robot.getpos,q(i,:),self.steps); 
                for i=1:size(qMatrix,1)
                    animate(self.robot,qMatrix(i,:));
                    drawnow();
                end
            end
        end
        
    end
end

