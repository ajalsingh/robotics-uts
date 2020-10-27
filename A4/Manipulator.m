classdef Manipulator
    %MANIPULATOR Summary of this class goes here
    %   This class handles all manipulator related tasks 
    %   such as moving and calculating max reach and volume
    %   Some code in this class has been taken from lab material
    
    properties

    end
    
    methods
        %%
        function self = Manipulator()
            %MANIPULATOR Construct an instance of this class
            
            
        end
        %%
        function MoveSingleArm(self,robot,goal)
            % Move a single arm to desired goal
            % input1: robot
            % input2: goal position
            % input3: brick Mesh object (optional)
            
            clc;
            % State 0 = retrieve brick
            % State 1 = place brick on wall
            % State 2 = invalid number of arguments
            if nargin ==3
                state = 0;
            elseif nargin==4
                state = 1;
            else
                state = 2;
            end

            if state==0
                goal(1,3) = goal(1,3) + 0.07;
            end
            
            if state ==2
                disp('Too few or too many input arguments');
            else
                steps = 50;

                % Determine joint angles to goal
                if length(goal) == 7 || length(goal) == 6
                    q1 = robot.getpos;
                    q2 = goal;
                else
                    q1 = robot.getpos;                                                        % Derive joint angles for required end-effector transformation
                    T2 = transl(goal);                                                   % Define a translation matrix            
                    q2 = robot.ikcon(T2,q1);
                end
                

                qMatrix = jtraj(q1,q2,steps);    

                % Obtain brick mesh properties
%                 if state ==1
%                     [f,v,data] = plyread('ply/Brick.ply','tri');
%                     % Get vertex count
%                     brickVertexCount = size(v,1);
% 
%                     % Move center point to origin
%                     midPoint = sum(v)/brickVertexCount;
%                     brickVerts = v - repmat(midPoint,brickVertexCount,1);
%                 end

                % Plot the results
                for i=1:size(qMatrix,1)
                    animate(robot,qMatrix(i,:));
                    if state==1
                        endP = robot.fkine(qMatrix(i,:));
                        endP = transl(endP(1:3,4)');
                        updatedPoints = [endP * [brickVerts,ones(brickVertexCount,1)]']';
%                         brick.Vertices = updatedPoints(:,1:3);
                    end
                    drawnow();
                end
            end
        end
        %%
        function MoveArms(self,robot1, robot2,goal1,goal2,brick1,brick2)
            % Move a two arms to desired respective goals
            % input1: robot 1
            % input2: robot 2
            % input3: goal position 1
            % input4: goal position 1
            % input5: brick Mesh object 1 (optional)
            % input6: brick Mesh object 2 (optional)

            clc;
            % State 0 = retrieve brick
            % State 1 = place brick on wall
            % State 2 = invalid number of arguments
            if nargin ==5
                state = 0;      
            elseif nargin==7
                state = 1;
            else
                state = 2;
            end

            if state==0
                goal1(1,3) = goal1(1,3) + 0.07;
                goal2(1,3) = goal2(1,3) + 0.07;
            end

            if state ==2
                disp('Too few or too many input arguments');
            else
                % Determine joint angles to goals
                steps = 50;
                
                % Check whether position or joint states are passed 
                if length(goal1) == 6
                    robot1q1 = robot1.getpos;
                    robot1q2 = goal1;
                else
                    robot1q1 = robot1.getpos;                                                        % Derive joint angles for required end-effector transformation
                    robot1T = transl(goal1);                                                   % Define a translation matrix            
                    robot1q2 = robot1.ikcon(robot1T,robot1q1);
                end

                if length(goal2) == 7
                    robot2q1 = robot2.getpos;
                    robot2q2 = goal2;
                else
                    robot2q1 = robot2.getpos;                                                        % Derive joint angles for required end-effector transformation
                    robot2T = transl(goal2);                                                   % Define a translation matrix            
                    robot2q2 = robot2.ikine(robot2T,robot2q1);
                end
                

                % Determine joint states of motion to goal
                robot1qMatrix = jtraj(robot1q1,robot1q2,steps);
                robot2qMatrix = jtraj(robot2q1,robot2q2,steps);

                %get brick object properties
                if state ==1
                    [f,v,data] = plyread('ply/Brick.ply','tri');
                    % Get vertex count
                    brickVertexCount = size(v,1);

                    % Move center point to origin
                    midPoint = sum(v)/brickVertexCount;
                    brickVerts = v - repmat(midPoint,brickVertexCount,1);
                end

                % Plot the results
                for i=1:steps
                    animate(robot1,robot1qMatrix(i,:));

                    if state==1
                        endP1 = robot1.fkine(robot1qMatrix(i,:));
                        endP1 = transl(endP1(1:3,4)');
                        updatedPoints1 = [endP1 * [brickVerts,ones(brickVertexCount,1)]']';
                        brick1.Vertices = updatedPoints1(:,1:3);
                    end

                    animate(robot2,robot2qMatrix(i,:));
                    if state==1
                        endP2 = robot2.fkine(robot2qMatrix(i,:));
                        endP2 = transl(endP2(1:3,4)');
                        updatedPoints2 = [endP2 * [brickVerts,ones(brickVertexCount,1)]']';
                        brick2.Vertices = updatedPoints2(:,1:3);
                    end
                    drawnow();
                end
            end

        end
        %%
        function values = RobotPointCloud(self,robot1, robot2)
            %   Creates a point cloud of robot arms (taken from lab
            %   solutions)
            %   Method used to determine max reach and volume
            %   input1: robot1
            %   input2: robot2
            %   output: values where,
            %                   values(1) = robot1 max side reach
            %                   values(2) = robot1 max top reach
            %                   values(3) = robot1 max volume of full sphere
            %                   values(4) = robot2 max side reach
            %                   values(5) = robot2 max top reach
            %                   values(6) = robot2 max volume of full sphere
            
            robots = [robot1 robot2];
            disp('Calculating... ');
            values= [];
            
            for i=1:length(robots)
                stepRads = deg2rad(70);
                qlim = robots(i).qlim;
                % Don't need to worry about joint 6
                PointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
                PointCloud = zeros(PointCloudeSize,3);
                counter = 1;
                tic
                for q1 = qlim(1,1):stepRads:qlim(1,2)
                    for q2 = qlim(2,1):stepRads:qlim(2,2)
                        for q3 = qlim(3,1):stepRads:qlim(3,2)
                            for q4 = qlim(4,1):stepRads:qlim(4,2)
                                for q5 = qlim(5,1):stepRads:qlim(5,2)
                                    if length(qlim)==6
                                        q6 = 0;
                                            q = [q1,q2,q3,q4,q5,q6];
                                            tr = robots(i).fkine(q);                        
                                            PointCloud(counter,:) = tr(1:3,4)';
                                            counter = counter + 1; 
                                    else
                                        for q6 = qlim(6,1):stepRads:qlim(6,2)
                                            q7 =0;
                                            q = [q1,q2,q3,q4,q5,q6,q7];
                                            tr = robots(i).fkine(q);                        
                                            PointCloud(counter,:) = tr(1:3,4)';
                                            counter = counter + 1; 
                                        end
                                    end
                                end
                            end
                        end
                    end
                end

                % plot possible end effector positions
                plot3(PointCloud(:,1),PointCloud(:,2),PointCloud(:,3),'r.');
                
                side = max([PointCloud(:,1); PointCloud(:,2)]);
                top = max(PointCloud(:,3));
                values(end+1) = side;
                values(end+1) = top;
                values(end+1) = 4/3*pi*(max(side,top)^2);
            end
        end
    end
end
