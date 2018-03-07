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
## @deftypefn {Function File} {@var{map} =} readPillars (@var{file})
##
## @seealso{}
## @end deftypefn

## Author: Clarisse <clarisse@clarisse-Ubuntu>
## Created: 2016-07-13

function [map] = readPillars (file)

pillars = [] ; % coordonates of the pillars localization
map = cell();

nbr_pillar = 0 ;
start = 0;

f = fopen(file);
while (!feof(f))
  line = fgets(f);
  [splitLine] = strread(line, ("%s"));  
  if (strcmp(splitLine{1,1},"Pillar:"))
    start = 1 ;
    nbr_pillar++;
    if( nbr_pillar > 1 )
      map(1,end+1) = pillars;
      pillars = [] ; 
    endif;
  

  elseif (strcmp(splitLine{1,1},"FEATURES."))
    map(1,end+1) = pillars;
    pillars = [] ; 
    break;
    
  elseif (start)
    % x,y in odom frame
    pillars(end + 1,:) = zeros(1,2) ; 
    x = str2num(splitLine{1,1}) ;
    y = str2num(splitLine{2,1}) ;
    pillars(end,:) =[x , y];
  endif
  
endwhile
fclose(f)


endfunction
