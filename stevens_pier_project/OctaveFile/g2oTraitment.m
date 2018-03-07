## Copyright (C) 2016 Clarisse
## 
## This program is free software; you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Founmation; either version 3 of the License, or
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
## @deftypefn {Function File} g2oTraitment
## Transformation : ldkEdge : polar coordinates in body frame -> cartesian coordinates in odom frame
## @seealso{}
## @end deftypefn

## Author: Clarisse <clarisse@john-LIFEBOOK-E780>
## Created: 2016-07-01




function g2oTraitment()
% data file :
% Vertex_SE2          1:id     | 2:x     | 3:y | 4:theta     odom frame
% VERTEX_POINT_XY     1:id     | 2:x     | 3:y               odom frame
% EDGE_SE2_POINT_XY   1:poseid | 2:lmkid | 4:r | 5:phi       body frame
% ....................................................................... 





% load data file 
poseVertex = dlmread("_poseVertex.mat"); % Vertex_SE2        : odom frame [x,y,theta]
ldkEdge    = dlmread("_ldkEdge.mat");    % EDGE_SE2_POINT_XY : body frame [r,phi]

ldkEdgeOdom   = [] ; % cartesian coordonates of the sensor measurements in odom frame [x,y]


%% ...Transformation : ldkEdge : polar coordinates in body frame -> cartesian coordinates in odom frame
for i = 1:size(ldkEdge,1)
  % try to find the index of the pose from which the landmark i has been looked
  [ixPv]=find(poseVertex(:,1) == ldkEdge(i,1));
  
  if isempty(ixPv) 
    disp ("error id, id pose missing");
  else
  
    % generate homogeneous matrix to transforme body frame point in odom frame point
    % z rotation : cos(theta)  -sin((theta)  tx
    %              sin(theta)   cos((theta)  ty
    %                   0            0       1
      
    MATodomtobody = [ cos(poseVertex(ixPv,4)) , -sin(poseVertex(ixPv,4))  , poseVertex(ixPv,2) ; ...
                      sin(poseVertex(ixPv,4)) ,  cos(poseVertex(ixPv,4))  , poseVertex(ixPv,3) ; ...
                              0          ,          0           ,       1       ];
                                
    % generate cartesian coordonate (x,y) from polar coordonate (r,phi)
    % [x;y] = [ r * cos(phi) ; r * sin(phi)] 
    ldkEBodyFrame = [ldkEdge(i,3) * cos(ldkEdge(i,4)) ; ldkEdge(i,3) * sin(ldkEdge(i,4)) ; 1] ;
    
    % update ldkEdge with the cartesian coordonates of the landmark in the odom frame 
    ldkEOdomFrame = MATodomtobody * ldkEBodyFrame ;

    ldkEdgeOdom(end + 1 , :) = zeros(1,4) ; 
    ldkEdgeOdom(end,:) = [ldkEdge(i,1), ldkEdge(i,2), ldkEOdomFrame(1,1),ldkEOdomFrame(2,1) ];
  endif
endfor
% .............................................................................................



% Save in .dat file
% data file :
% Vertex_SE2          1:id     | 2:x     | 3:y | 4:theta     odom frame
% VERTEX_POINT_XY     1:id     | 2:x     | 3:y               odom frame
% EDGE_SE2_POINT_XY   1:poseid | 2:lmkid | 4:x | 5:y         odom frame
% ....................................................................... 
dlmwrite("_ldkEdgeOdom.mat",ldkEdgeOdom,"delimiter"," ","newline", "\n");


endfunction
