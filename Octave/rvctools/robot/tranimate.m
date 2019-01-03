%TRANIMATE Animate a coordinate frame
%
% TRANIMATE(P1, P2, OPTIONS) animates a 3D coordinate frame moving from pose P1
% to pose P2.  Poses P1 and P2 can be represented by:
%   - homogeneous transformation matrices (4x4)
%   - orthonormal rotation matrices (3x3)
%   - Quaternion
%
% TRANIMATE(P, OPTIONS) animates a coordinate frame moving from the identity pose
% to the pose P represented by any of the types listed above.
%
% TRANIMATE(PSEQ, OPTIONS) animates a trajectory, where PSEQ is any of
%   - homogeneous transformation matrix sequence (4x4xN)
%   - orthonormal rotation matrix sequence (3x3xN)
%   - Quaternion vector (Nx1)
%
% Options::
%  'fps', fps    Number of frames per second to display (default 10)
%  'nsteps', n   The number of steps along the path (default 50)
%  'axis',A      Axis bounds [xmin, xmax, ymin, ymax, zmin, zmax]
%  'movie',M     Save frames as files in the folder M
%
% Notes::
% - The 'movie' options saves frames as files NNNN.png.
% - When using 'movie' option ensure that the window is fully visible.
% - To convert frames to a movie use a command like:
%        ffmpeg -r 10 -i %04d.png out.avi
%
% See also TRPLOT.

% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

% TODO
%  auto detect the axis scaling
function tranimate(P2, varargin)

    opt.fps = 10;
    opt.nsteps = 50;
    opt.axis = [];
    opt.movie = [];

    [opt, args] = tb_optparse(opt, varargin);
    
    % create movie images subdirectories if recording a movie
    FMT = "jpg";  % Set to "png" or "jpg"
    if ~isempty(opt.movie)
        if ~exist(opt.movie, 'dir')
            mkdir(opt.movie);
        end
        if isOctave
            sdir = [opt.movie, "/", FMT, "s"];
        else
            sdir = join([opt.movie, "/", FMT, "s"],"");
        end
        if ~exist(sdir, 'dir')
            mkdir(sdir);
        end
    end
 
    P1 = [];

    % convert quaternion and rotation matrix to hom transform
    if isa(P2, 'Quaternion')
        T2 = P2.T;   % convert quaternion to transform
        if ~isempty(args) && isa(args{1},'Quaternion')
            T1 = T2;
            Q2 = args{1};
            T2 = Q2.T;
            args = args(2:end);
        else
            T1 = eye(4,4);
        end
    elseif isrot(P2)
        T2 = r2t(P2);
        if ~isempty(args) && isrot(args{1})
            T1 = T2;
            T2 = r2t(args{1});
            args = args(2:end);
        else
            T1 = eye(4,4);
        end
    elseif ishomog(P2)
        T2 = P2;
        if ~isempty(args) && ishomog(args{1})
            T1 = T2;
            T2 = args{1};
            args = args(2:end);
        else
            T1 = eye(4,4);
        end
    end
    
    % at this point
    %   T1 is the initial pose
    %   T2 is the final pose
    %
    % T2 may be a sequence
        
    if size(T2,3) > 1
        % tranimate(Ts)
        % we were passed a homog sequence
        if ~isempty(P1)
            error('only 1 input argument if sequence specified');
        end
        Ttraj = T2;
    else
        % tranimate(P1, P2)
        % create a path between them
        Ttraj = ctraj(T1, T2, opt.nsteps);
    end
    
    if isempty(opt.axis)
        % create axis limits automatically based on motion of frame origin
        t = transl(Ttraj);
        mn = min(t) - 1.5;  % min value + length of axis + some
        mx = max(t) + 1.5;  % max value + length of axis + some
        axlim = [mn; mx];
        axlim = axlim(:)';
        args = [args 'axis' axlim];
    end
    
    hg = trplot(eye(4,4), args{:});  % create a frame at the origin
 
    if ~isempty(opt.movie)
        numF = 1;
        if isOctave
            fig1 = gcf();
            fig1pos = get(fig1, 'position');
            F(numF,:) = putimage(fig1, opt.movie, numF, FMT, fig1pos(3), fig1pos(4));
        else
            fig1 = gcf();
            fig1pos = get(fig1, 'innerposition');
            F(numF) = getframe(fig1, [0 0 fig1pos(3), fig1pos(4)]);
        end
    end

    % animate it for all poses in the sequence

    for i=1:size(Ttraj,3)
        T = Ttraj(:,:,i);
        trplot(hg, T);
        
        if ~isempty(opt.movie)
            if isOctave
                F(numF,:) = putimage(gcf(), opt.movie, numF, FMT, fig1pos(3), fig1pos(4));
            else
                F(numF) = getframe(gcf(), [0 0 fig1pos(3), fig1pos(4)]);
                imwrite(F(numF).cdata, ...
                            sprintf('%s/%ss/%04d.%s', opt.movie, FMT, numF, FMT));
            end
            numF = numF + 1;
        end
        
        pause(1/opt.fps);
    end

    % display recorded movie
    if ~isempty(opt.movie)
        fprintf('On the figure, use the mouse buttons:\n');
        fprintf('  + left to replay the movie\n');
        fprintf('  + right to exit the program\n');
        figM = figure('Name','RTB-tranimate - Recorded Frames', ...
                      'NumberTitle', 'Off', 'toolbar', 'none');
        fig1opos = get(fig1, 'outerposition');
        sfac = fig1opos(4)/fig1pos(4);
        if isOctave
           set(figM, 'position', [720 100 fig1opos(3)*sfac fig1opos(4)]);
        else
           set(figM, 'outerposition', [720 100 fig1opos(3)*sfac fig1opos(4)]);
        end
        grid off;
        button = 1;
        while button == 1
            if isOctave
                for k = 1:length(F)
                    img = getimage(F(k,1:end));
                    figMpos = get(figM, 'position');
                    image([0 fig1pos(3)], [0 fig1pos(4)], img, 'clipping', "on");
                    axis("off");
                    set(figM, 'position', [figMpos(1) figMpos(2) fig1opos(3)*sfac fig1opos(4)]);
                    pause(1.0/opt.fps);
                end
            else
                axis off
                movie(figM, F);
            end
            [Xm,Ym,button] = ginput(1);
        end
        filename = sprintf('RTB_%s.avi', "tranimate");
        filepath = sprintf('%s/%s', opt.movie, filename);
        fprintf('Recorded movie frames saved as file %s\n', filepath);
        if isOctave
            info = imfinfo(F(1,1:end));
            image2avi(FMT, opt.fps, info.Width, info.Height, opt.movie, filepath(1:end));
        else
            v = VideoWriter(filepath, 'Motion JPEG AVI');
            v.FrameRate = opt.fps;
            open(v);
            writeVideo(v,F);
            close(v);
        end
    end

end % tranimate
