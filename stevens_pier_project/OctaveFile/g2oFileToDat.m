## Copyright (C) 2016 Clarisse
## 
## This program is free software; you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
## 
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.

## -*- texinfo -*- 
## @deftypefn {Function File} g2oFileToDat (@var{file})
## Allow to read a g2o file and save it to dat file 
## @seealso{}
## @end deftypefn

## Author: Clarisse <clarisse@john-LIFEBOOK-E780>
## Created: 2016-07-01

function g2oFileToDat (file)

% G2O file :             
% 1:Vertex_SE2        | 2:id        | 3:x       | 4:y | 5:theta 
% 1:VERTEX_POINT_XY   | 2:id        | 3:x       | 4:y  
% 1:EDGE_SE2_POINT_XY | 2:poseSE2id | 3:ldkXYid | 4:0 | 5:r     | 6:phi 
% ....................................................................... 



poseVertex = [] ; % coordonates of the robot poses 
ldkVertex  = [] ; % coordonates of the landmark localisations 
ldkEdge    = [] ; % coordonates of the sensor measurements 

f = fopen(file);
while (!feof(f))
  line = fgets(f);
  [splitLine] = strread(line, ("%s"));  
  switch (splitLine{1,1}) 
  
    case "VERTEX_SE2"
      % add a robot pose [id,x,y,theta] to the vector poseVertex
      poseVertex(end + 1 , :) = zeros(1,4) ; 
      poseVertex(end,:) = [str2num(splitLine{2,1}) ,str2num(splitLine{3,1}) ,str2num(splitLine{4,1}) ,str2num(splitLine{5,1}) ];
          
    case "VERTEX_POINT_XY"
      % add a landmark localisation [id x,y] to the vector ldkVertex
      ldkVertex(end + 1 , :) = zeros(1,3) ; 
      ldkVertex(end,:) = [str2num(splitLine{2,1}) ,str2num(splitLine{3,1}) ,str2num(splitLine{4,1}) ];
      
    case "EDGE_SE2_POINT_XY"
      % add a landmark measurement [poseId, ldkId, r,phi] to the vector ldkEdge 
      ldkEdge(end + 1 , :) = zeros(1,4) ; 
      ldkEdge(end,:) = [str2num(splitLine{2,1}) ,str2num(splitLine{3,1}) ,str2num(splitLine{5,1}) ,str2num(splitLine{6,1}) ];
    
    %TODO : if you need the robot pose measurements you can uncomment the following and add your treatment
    % case "EDGE_SE2 "

    otherwise
     %disp ("Other \n");
        
  endswitch
endwhile
fclose(f);

% Save in .dat file
% Vertex_SE2          1:id     | 2:x     | 3:y | 4:theta     odom frame
% VERTEX_POINT_XY     1:id     | 2:x     | 3:y               odom frame
% EDGE_SE2_POINT_XY   1:poseid | 2:lmkid | 4:r | 5:phi       body frame
dlmwrite("_poseVertex.mat",poseVertex,"delimiter"," ","newline", "\n");
dlmwrite("_ldkVertex.mat",ldkVertex,"delimiter"," ","newline", "\n");
dlmwrite("_ldkEdge.mat",ldkEdge,"delimiter"," ","newline", "\n");

















%save _poseVertex.mat poseVertex 
%save _ldkVertex.mat ldkVertex 
%save _ldkEdge.mat ldkEdge

%
%
% %% ------------------- FILE 2 : AFTER OPTIM -------------------%%
%poseVertexAO = []; % coordonate of the robot poses in odom frame 
%ldkVertexAO = [] ; % coordonate of the landmark localisations 
%ldkEdgeA0 = [] ; % coordonate of the sensor measurements in body frame
%
%f = fopen(fileAO);
%while (!feof(f))
%  line = fgets(f);
%  [splitLineAO] = strread(line, ("%s"));  
%  switch (splitLineAO{1,1}) 
%    case "VERTEX_SE2"
%      % add a robot pose [id,x,y,theta] to the vector poseVertex
%      poseVertexAO(end + 1 , :) = zeros(1,4) ; 
%      poseVertexAO(end,:) = [str2num(splitLineAO{2,1}) ,str2num(splitLineAO{3,1}) ,str2num(splitLineAO{4,1}) ,str2num(splitLineAO{5,1}) ];
%          
%    case "VERTEX_POINT_XY"
%      % add a landmark localisation [id x,y] to the vector ldkVertex
%      ldkVertexAO(end + 1 , :) = zeros(1,3) ; 
%      ldkVertexAO(end,:) = [str2num(splitLineAO{2,1}) ,str2num(splitLineAO{3,1}) ,str2num(splitLineAO{4,1}) ];
%    case "EDGE_SE2_POINT_XY"
%      % add a landmark measurement [poseId, ldkId, r,phi] to the vector ldkMes 
%      ldkEdgeAO(end + 1 , :) = zeros(1,4) ; 
%      ldkEdgeAO(end,:) = [str2num(splitLineAO{2,1}) ,str2num(splitLineAO{3,1}) ,str2num(splitLineAO{5,1}) ,str2num(splitLineAO{6,1}) ];
%
%    otherwise
%     %disp ("Other \n");
%  endswitch
%endwhile
%fclose(f)
%
%% Save in .dat file
%save _poseVertexAO.mat poseVertexAO
%save _ldkVertexAO.mat ldkVertexAO
%save _ldkEdgeAO.mat ldkEdgeAO

endfunction
