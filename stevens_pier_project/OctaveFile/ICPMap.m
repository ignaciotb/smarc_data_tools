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

## This script allow to display the map obtained with a scan matching SLAM
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
files       = [ "../mapping/savedData/2015-08-19-11-25-05/02/pillar02.txt";
                "../mapping/savedData/2015-08-19-11-25-05/03/pillar03.txt"];
% radius : contained the radius size corresponding to the g2o file : used to the legend
radius    = [ "0.2m " ;
              "0.3m " ];
% nFigure : previse the figure name for the display
nFigure   = [ "(a) " ;
              "(b) " ];
              
precision   = 5; % The precision of the values
color       = [ [187 140 198] ./ 255 ; [69 130 186] ./ 255 ; [214 50 78] ./ 255 ; ...
              [113 180 97] ./ 255 ; [246 134 194] ./ 255 ; [0,255,128] ./255 ; ...
              [102,102,255] ./255 ; [255,178,102] ./255];
color_black = [0,0,0] ./255 ;
feature     = cell();
map         = cell();



for f = 1:size(files,1)

  ## Display 
  figure();
  map = readPillars(files(f,:)); % read the map on the file f
  disp ( ["map " , num2str(f)," done \n"] );

  hold on;
  for i = 1:size(map,2)
    [r_ellipse, X0,  Y0] = plot95ErrEll (map{1,i}); % 95% error ellipse computation 
    % PLOT : plot the feaures
    plot(map{1,i}(:,1),map{1,i}(:,2),'ko','MarkerEdgeColor', color(mod(i,8)+1,:),'MarkerFaceColor', color(mod(i,8)+1,:));
    drawnow ;
    % PLOT : plot the ellipses
    plot(r_ellipse(:,1) + X0,r_ellipse(:,2) + Y0,'-')
    drawnow ;
    % PLOT : The map found with the scan matching SLAM 
    plot(map{1,i}(end,1),map{1,i}(end,2),'ko','MarkerEdgeColor', color_black,'MarkerFaceColor',color_black);
    text(map{1,i}(end,1) + 1,map{1,i}(end,2) - 1 ,["(",mat2str(map{1,i}(end,1),precision),", ",mat2str(map{1,i}(end,2),precision),")" ]);
    drawnow ;
  endfor
  hold off;
  axis("equal");
  title( [nFigure(f,:)," Scan matching slam : Pillar radius ",radius(f,:)] );
  xlabel ("x(m)");
  ylabel ("y(m)");
  
endfor;
