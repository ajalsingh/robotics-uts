classdef RobotClass
    %ROBOTCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot;
        home;
        tool;
        toolVertexCount;
        toolMidPoint;
        toolVerts;
    end
    
    methods
        function self = RobotClass(robot_base_transform, tool_offset)
            mdl_puma560;
            self.robot = p560;
            self.home = deg2rad([0 90 0 0 45 0]);
            self.initRobot(robot_base_transform, tool_offset)
            
            [f,v,data] = plyread('ply/tool.ply','tri');
            
            % Get vertex count
            self.toolVertexCount = size(v,1);

            % Move center point to origin
            self.toolMidPoint = sum(v)/self.toolVertexCount;
            self.toolMidPoint(3) = self.toolMidPoint(3)+0.07;
            self.toolMidPoint(1) = self.toolMidPoint(1);%-0.05;
            self.toolVerts = v - repmat(self.toolMidPoint,self.toolVertexCount,1);

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            T = self.robot.fkine(self.robot.getpos);
            offset = T(1:3,4)';
            
            % Then plot the trisurf
            self.tool = trisurf(f,v(:,1)+offset(1),v(:,2)+offset(2), v(:,3)+offset(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            T = T  * troty(deg2rad(180));
            updatedPoints = [T * [self.toolVerts,ones(self.toolVertexCount,1)]']';
            self.tool.Vertices = updatedPoints(:,1:3);
        end
        
        function initRobot(self,robot_base_transform, tool_offset)
            %set base
            self.robot.base = robot_base_transform;

            % set tool
            self.robot.tool = tool_offset;

            % plot robot
            self.robot.plot(self.home)
            xlim([-0.5,1.5]);
            ylim([0,2]);
            zlim([0,2]);
            hold on; 
            
            drawnow();
        end
        
        %% Move arm and plot results
        function moveArm(self, pose, time, plotON)
            if nargin == 3
                plotON = 0;
            end

            if plotON == 1
                disp('blasting')
            end

            q1 = self.robot.getpos;

            if length(pose) == 6
                q2 = pose;
            else                                                          
                T2 = transl(pose) * troty(pi);           
                q2 = self.robot.ikcon(T2,q1);
            end

            dt = 1/100;
            steps = time/dt;

            s = lspb(0, 1, steps); % First, create the scalar function
            qMatrix = nan(steps, 6); % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i, :) = (1 - s(i)) * q1 + s(i) * q2; % Generate interpolated joint angles
            end

            qd = zeros(steps,6);                                                        % Array of joint velocities
            qdd = nan(steps,6);                                                         % Array of joint accelerations
            tau = nan(steps,6);                                                         % Array of joint torques
            tau_max = [97.6 186.4 89.4 24.2 20.1 21.3]';
            qd_max = [8, 10, 10, 5, 5, 5]';
            qdd_max = [10, 12, 12, 8, 8, 8]';
            w = [0, 0, 209, 0, 0, 0]';
            
            mass = 2.09;                                                                  % Payload mass (kg)
            grav = 9.81;
            reaction = 209;
            total_force = mass*grav - reaction;
            total_mass = total_force/grav;
            self.robot.payload(total_mass,[0;0;0]); 
            
            for i = 1:steps-1
                qdd(i,:) = (1/dt)^2 * (qMatrix(i+1,:) - qMatrix(i,:) - dt*qd(i,:));                 % Calculate joint acceleration to get to next set of joint angles
                M = self.robot.inertia(qMatrix(i,:));                                               % Calculate inertia matrix at this pose
                C = self.robot.coriolis(qMatrix(i,:),qd(i,:));                                      % Calculate coriolis matrix at this pose
                J = self.robot.jacob0(qMatrix(i,:));                
                g = self.robot.gravload(qMatrix(i,:));                                              % Calculate gravity vector at this pose
                tau(i,:) = (M*qdd(i,:)' + C*qd(i,:)' + g' + J'*w)' ;                            % Calculate the joint torque needed
                for j = 1:6
                    if abs(tau(i,j)) > tau_max(j)                                       % Check if torque exceeds limits
                        tau(i,j) = sign(tau(i,j))*tau_max(j);                           % Cap joint torque if above limits
                    end
                    % Check within max velocities
                    if abs(qd(i,j)) > qd_max(j)                                       % Check if v exceeds limits
                        qd(i,j) = sign(qd(i,j))*qd_max(j);                           % Cap joint v if above limits
                    end

                    % Check within max accelerations
                    if abs(qdd(i,j)) > qdd_max(j)                                       % Check if a exceeds limits
                        qdd(i,j) = sign(qdd(i,j))*qdd_max(j);                           % Cap joint a if above limits
                    end
                end
                qdd(i,:) = (inv(M)*(tau(i,:)' - C*qd(i,:)' - g'))';                     % Re-calculate acceleration based on actual torque
                qMatrix(i+1,:) = qMatrix(i,:) + dt*qd(i,:) + dt^2*qdd(i,:);                         % Update joint angles based on actual acceleration
                qd(i+1,:) = qd(i,:) + dt*qdd(i,:);                                      % Update the velocity for the next pose

                % update robot joints
                self.robot.animate(qMatrix(i,:));

                % update end-effector pose
                endP = self.robot.fkine(qMatrix(i,:));
                endP = endP  * troty(deg2rad(180));
                updatedPoints = [endP * [self.toolVerts,ones(self.toolVertexCount,1)]']';
                self.tool.Vertices = updatedPoints(:,1:3);
                drawnow();

            end

            t = 0:dt:(steps-1)*dt; 

            if plotON == 1
                disp('preparing plots')

                % Plot joint angles
                figure(2)
                for j = 1:6
                    subplot(3,2,j)
                    plot(t,qMatrix(:,j)','k','LineWidth',1);
                    refline(0,self.robot.qlim(j,1));
                    refline(0,self.robot.qlim(j,2));
                    ylabel('Angle (rad)');
                    box off
                end

                % Plot joint velocities
                figure(3)
                for j = 1:6
                    subplot(3,2,j)
                    plot(t,qd(:,j),'k','LineWidth',1);
                    refline(0,qd_max(j));
                    refline(0,-qd_max(j));
                    ylabel('Velocity (rad/s)');
                    box off
                end

                % Plot joint acceleration
                figure(4)
                for j = 1:6
                    subplot(3,2,j)
                    plot(t,qdd(:,j),'k','LineWidth',1);
                    ylabel('rad/s/s');
                    refline(0,qdd_max(j));
                    refline(0,-qdd_max(j));
                    box off
                end

                % Plot joint torques
                figure(5)
                for j = 1:6
                    subplot(3,2,j)
                    plot(t,tau(:,j),'k','LineWidth',1);
                    refline(0,tau_max(j));
                    refline(0,-tau_max(j));
                    ylabel('Nm');
                    box off
                end
            end
        end 
        
        function updatePos(self, propTr) % move the prop to a given Tr
			for j = 1:self.toolVertexCount
				self.prop_h.Vertices(j,:) = transl(propTr*transl(self.points(j,:)))';
			end
        end
    end
end

