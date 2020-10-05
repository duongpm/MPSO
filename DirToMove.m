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

% Convert from the target direction to a corresponding move in the map 

function move = DirToMove(dir)
    switch dir
        case 'N'
            move = [1 0];
        case 'NE'
            move = [1 1];
        case 'E'
            move = [0 1];
        case 'SE'
            move = [-1 1];
        case 'S'
            move = [-1 0];
        case 'SW'
            move = [-1 -1];
        case 'W'
            move = [0 -1];
        case 'NW'
            move = [1 -1];
    end
end