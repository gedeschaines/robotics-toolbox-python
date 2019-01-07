% Copyright (C) 1993-2013, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License
% as published by the Free Software Foundation, either version
% 3 of the License, or (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
% the GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Lesser General
% Public License along with RTB. If not, copies may be viewed
% and downloaded at <https://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

% This file contains scripting elements extracted from RTB for
% MATLAB v9.8 rvctools/robot/demos/traj.m script to illustrate
% the creation and plotting of frame transform sequences.

%%begin

echo off
more off

% We may wish to interpolate between poses. We will define an 
% initial and final pose as homogeneous transforms T0 and T1
% respectively,

T0 = transl(0.4, 0.2, 0) * trotx(pi);
T1 = transl(-0.4, -0.2, 0.3) * troty(pi/2) * trotz(-pi/2);

% and apply the ctraj() function to create the smooth sequence
% T between them in 50 steps as:

T = ctraj(T0, T1, 50);

printf("The first pose transform is:\n");
display(T(:,:,1));
printf("The tenth pose transform is:\n");
display(T(:,:,10));
printf("The final pose transform is:\n");
display(T(:,:,50));

% We can plot and record the motion of an XYZ Cartesian coordinate
% frame by using the tranimate() function.

set(gca,'NextPlot','ReplaceChildren');  % setting these appears to  
set(gcf, 'toolbar', 'None');            % ensure smooth image saves
axlims = [-1.5 1.5 -1.5 1.5 -1.5 1.5];
tranimate(T, 'movie', "./images/Octave", 'thick', 1.0, 'axis', axlims);

