classdef Bricks
    %BRICKS Creates bricks at initial positions and determines drop
    %locations
    
    properties

    end
    
    methods
        %%
        function [self,mesh] = Bricks(bricksPoses)
            %   Create and place 9 bricks from ply file
            %   input:      array of brick poses
            %   output1:    class object
            %   output2:    mesh array of type patch
            
            [f,v,data] = plyread('ply/Brick.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            for i=1:size(bricksPoses,1)
                mesh(i) = trisurf(f,v(:,1)+bricksPoses(i,1),v(:,2)+bricksPoses(i,2), v(:,3)+bricksPoses(i,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            end
        end
        %%
        function drop = CalcDropPoses(self,d1)
            % Calculates drop positions based on first brick location
            % input: first drop location
            % output: matrix of drop locations (x y z)
            
            d2 = [d1(1),d1(2)+0.15, d1(3)]';
            d3 = [d1(1),d2(2)+0.15, d1(3)]';
            d4 = [d1(1),d1(2),      d1(3)+0.08]'; 
            d5 = [d1(1),d1(2)+0.15, d1(3)+0.08]';
            d6 = [d1(1),d2(2)+0.15, d1(3)+0.08]';
            d7 = [d1(1),d1(2),      d4(3)+0.08]'; 
            d8 = [d1(1),d1(2)+0.15, d4(3)+0.08]';
            d9 = [d1(1),d2(2)+0.15, d4(3)+0.08]';
            drop = [d1 d2 d3 d4 d5 d6 d7 d8 d9]';
        end
    end
end

