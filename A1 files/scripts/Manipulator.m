classdef Manipulator
    %MANIPULATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties

    end
    
    methods
        function self = Manipulator()
            %MANIPULATOR Construct an instance of this class
            %   Detailed explanation goes here
            
        end
        
        function MoveSingleArm(self,robot,goal,brick)
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

            % if state==0
            %     goal(1,3) = goal(1,3) + 0.07;
            % end
            
            if state ==2
                disp('Too few or too many input arguments');
            else
                steps = 50;

                q1 = robot.getpos;                                                        % Derive joint angles for required end-effector transformation
                T2 = transl(goal);                                                   % Define a translation matrix            
                q2 = robot.ikine(T2,q1, [1 1 1 0 0 0]);

                qMatrix = jtraj(q1,q2,steps);    

                %%
                if state ==1
                    [f,v,data] = plyread('ply/Brick.ply','tri');
                    % Get vertex count
                    brickVertexCount = size(v,1);

                    % Move center point to origin
                    midPoint = sum(v)/brickVertexCount;
                    brickVerts = v - repmat(midPoint,brickVertexCount,1);
                end

                %% Plot the results
                % ur5.plot(,'trail','r-')
                for i=1:size(qMatrix,1)
                    animate(robot,qMatrix(i,:));
                    if state==1
                        endP = robot.fkine(qMatrix(i,:));
                        endP = transl(endP(1:3,4)');
                        updatedPoints = [endP * [brickVerts,ones(brickVertexCount,1)]']';
                        brick.Vertices = updatedPoints(:,1:3);
                    end
                    drawnow();
                end
            end
        end
        
        function MoveArms(self,robot1, robot2,goal1,goal2,brick1,brick2)
            %RETRIEVE Summary of this function goes here
            %   Detailed explanation goes here

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

            % if state==0
            %     goal1(1,3) = goal1(1,3) + 0.07;
            %     goal2(1,3) = goal2(1,3) + 0.07;
            % end

            if state ==2
                disp('Too few or too many input arguments');
            else
                %%
                steps = 50;

                robot1q1 = robot1.getpos;                                                        % Derive joint angles for required end-effector transformation
                robot1T = transl(goal1);                                                   % Define a translation matrix            
                robot1q2 = robot1.ikine(robot1T,robot1q1, [1 1 1 0 0 0]);

                robot1qMatrix = jtraj(robot1q1,robot1q2,steps);

                robot2q1 = robot2.getpos;                                                        % Derive joint angles for required end-effector transformation
                robot2T = transl(goal2);                                                   % Define a translation matrix            
                robot2q2 = robot2.ikine(robot2T,robot2q1, [1 1 1 0 0 0]);

                robot2qMatrix = jtraj(robot2q1,robot2q2,steps);

                %%
                if state ==1
                    [f,v,data] = plyread('ply/Brick.ply','tri');
                    % Get vertex count
                    brickVertexCount = size(v,1);

                    % Move center point to origin
                    midPoint = sum(v)/brickVertexCount;
                    brickVerts = v - repmat(midPoint,brickVertexCount,1);
                end

                %% Plot the results
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
                        brick2.Vertices = updatedPoints2(:,1:3)
                    end
                    drawnow();
                end
            end

        end
        
        function values = RobotPointCloud(self,robot1, robot2)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
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

                side = max([PointCloud(:,1); PointCloud(:,2)]);
                top = max(PointCloud(:,3));
                values(end+1) = side;
                values(end+1) = top;
                values(end+1) = 4/3*pi*(max(side,top)^2);
            end
        end
    end
end

