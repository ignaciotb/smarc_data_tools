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

## This script allow to display the map obtained with a graph-based SLAM
##

## Author: Clarisse <clarisse@john-LIFEBOOK-E780>
## Created: 2016-07-01


% ... TODO : if you want to display diferents results ....................
% To display your own results you have to change the following variables :
% - files
% - radius 
% - nfigure
% ........................................................................

close all;
clear all;


% files : contained the g2o file for the display
files     = [ "../mapping/savedData/2015-08-19-11-25-05/02/pillar02_OPTIM_A.g2o"];
% radius : contained the radius size corresponding to the g2o file : used to the legend
radius    = [ "0.2m " ];
% nFigure : previse the figure name for the display
nFigure   = [ "(a) " ];
precision = 5;     % The precision of the values
dispDA    = true;  % True to display the data association, false otherwise 

for f = 1:size(files,1)

  ## Read g2o file
  % G2O file :             
  % 1:Vertex_SE2        | 2:id        | 3:x       | 4:y | 5:theta 
  % 1:VERTEX_POINT_XY   | 2:id        | 3:x       | 4:y  
  % 1:EDGE_SE2_POINT_XY | 2:poseSE2id | 3:ldkXYid | 4:0 | 5:r     | 6:phi 
  ## ....................................................................... 
  g2oFileToDat(files(f,:));
  disp ( ["Read file " , num2str(f)," done \n"] );
  
  ## Traitement of landmark vertex and landmark edges 
  % data file :
  % Vertex_SE2          1:id     | 2:x     | 3:y | 4:theta     odom frame
  % VERTEX_POINT_XY     1:id     | 2:x     | 3:y               odom frame
  % EDGE_SE2_POINT_XY   1:poseid | 2:ldkid | 4:r | 5:phi       body frame
  # ....................................................................... 
  g2oTraitment(true);
  
  ## Data file :
  % Vertex_SE2          1:id     | 2:x     | 3:y | 4:theta     odom frame
  % VERTEX_POINT_XY     1:id     | 2:x     | 3:y               odom frame
  % EDGE_SE2_POINT_XY   1:poseid | 2:ldkid | 4:x | 5:y         odom frame
  # ....................................................................... 


  ## Load mat file 
    Pv = dlmread("_poseVertex.mat");  % cartesian coordonate [x,y,theta] of the robot poses in odom frame
    Lv = dlmread("_ldkVertex.mat");   % landmark localizations in odom frame
    Le = dlmread("_ldkEdgeOdom.mat"); % cartesian coordonate [x,y] of the sensor measurements in odom frame
  # .......................................................................


  ## Display 
    figure()
      % variables :
      % Pv   (nbrVertex,4)  1:id     | 2:x     | 3:y | 4:theta     odom frame
      % Lv   (nbrVertex,3)  1:id     | 2:x     | 3:y               odom frame
      % Le    (nbrEdge,4)   1:poseid | 2:ldkid | 4:x | 5:y         odom frame
      % ....
    
    % PLOT : robot path, landmark measurements and landmark vertex 
      plot  ( Pv(2:end,2),Pv(2:end,3),'k-',Le(:,3),Le(:,4),'m+');
      hold on;
      plot ( Lv(:,2),Lv(:,3),'k.',"markersize", 10);
      legend("Robot path", "sensor measurements",'landmark localizations','location', 'northeastoutside');
    
    % PLOT : landmark vertices position (x,y)
    for i=1:size(Lv,1)
      idd = find(Le(:,2) == Lv(i,1));
      text(Le(idd(end),3) + 0.1, Le(idd(end),4) + 0.1 ,["(",mat2str(Le(idd(end),3),precision),", ",mat2str(Le(idd(end),4),precision),")" ]);
      text(Lv(i,2) - 0.1, Lv(i,3) - 0.3 , mat2str(i)); 
    endfor
    
    % Display parameters
    axis ("equal");
    title( [nFigure(f,:),"Graph-based Slam : Pillar radius ",radius(f,:)] );
    xlabel(" x(m) ");
    ylabel(" y(m) ");

    % generate table with the landmark ids save in the graph
      % variables :
      % ldkId  (1,nbrldk)  1:ldkid1 | 2:ldkid2    ...
      % ....
    ldkId = zeros(1,size(Lv,1));
    for i=1:size(Lv,1)
      ldkId(i) = Lv(i,1);
    endfor
  # .......................................................................
  hold off;
  
  
  % Display the data association : one color is used for on unique landmark
  if (dispDA)
    figure()
    color = ['ko';'ro';'go'; 'bo';'mo';'co';'k+';'r+';'g+'; 'b+';'m+';'c+'; 'k.';'r.';'g.'; 'b.';'m.';'c.';'ks';'rs';'gs'; 'bs';'ms';'cs' ] ;
    hold on;
    for i=1:size(Lv,1)
      ldkId = Lv(i,1);
      idd = find(Le(:,2) == ldkId);
      for j=1:size(idd)
        plot( Le(idd(j),3), Le(idd(j),4) ,color(mod(i,24)+1,:))
        % Display parameters
        axis ("equal");
        title( "Data association : one color is used for on unique landmark"  ); 
        xlabel(" x(m) ");
        ylabel(" y(m) ");
        drawnow
      endfor 
    endfor

    hold off;
  endif 
endfor;

 waitforbuttonpress ();

