classdef Bricks
    %BRICKS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties

    end
    
    methods
        function [self,mesh] = Bricks(bricksPoses)
            %BRICKS Construct an instance of this class
            %   Detailed explanation goes here
            %% Place bricks
            [f,v,data] = plyread('ply/Brick.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            for i=1:size(bricksPoses,1)
                mesh(i) = trisurf(f,v(:,1)+bricksPoses(i,1),v(:,2)+bricksPoses(i,2), v(:,3)+bricksPoses(i,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            end
        end
        
        function drop = CalcDropPoses(self,d1)
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
        
        function modifyPoses(self,brickMesh, bricksPoses)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            [f,v,data] = plyread('ply/Brick.ply','tri');
            % Get vertex count
            brickVertexCount = size(v,1);

            % Move center point to origin
            midPoint = sum(v)/brickVertexCount;
            brickVerts = v - repmat(midPoint,brickVertexCount,1);
            
            for i=1:size(brickMesh,1)
                updatedPoints2 = [bricksPoses(i) * [brickVerts,ones(brickVertexCount,1)]']';
                brickMesh(i).Vertices = updatedPoints2(:,1:3);
            end

        end
    end
end

