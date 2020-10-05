%_________________________________________________________________________%
%  Motion-encoded Partical Swarm Optimization (MPSO) source codes demo 1.0%
%                                                                         %
%  Developed in MATLAB 2020a                                              %
%                                                                         %
%  Author and programmer: Manh Duong Phung                                %
%                                                                         %
%         e-Mail: duongpm@gmail.com                                       %
%                 duongpm@vnu.edu.vn                                      %
%                                                                         %
%       Homepage: https://uet.vnu.edu.vn/~duongpm/                        %
%                                                                         %
%   Main paper: Manh Duong Phung, Quang Phuc Ha                           %
%               "Motion-encoded particle swarm optimization for moving    %
%               target search using UAVs",                                %
%               Applied soft computing , in press,                        %
%               DOI: https://doi.org/10.1016/j.asoc.2020.106705           %
%                                                                         %
%_________________________________________________________________________%

% Convert the encoded motion from an array of 8 numbers to a move
% Eight numbers encode the magnitude of a vector with the angle of
% NW, N, NE, W, E, SW, S, SE
% 1   2   3  4  5   6  7  8
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function move = MotionDecode(motion)
    
    % Find the angle of the sum vector
    angle = atan2(motion(2), motion(1));
    
    % Map the angle to its corresponding octant (0, 45, 90, 135 ...)
    octant = mod(round( 8 * angle / (2*pi) + 8 ),8) + 1;
    moveArray = [1 0;
                 1 1;
                 0 1;
                 -1 1;
                 -1 0;
                 -1 -1;
                 0 -1;
                 1 -1];
     % Map the octant to a move
     move = moveArray(octant,:);
end