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


% Update the belief map given the current position of UAV and its
% measurement

function [scaleFactor, newMap] = UpdateMap(currentStep,pathLength,totalMoves,dir, location, map)

    mapSize = size(map);
    mapSizeX = mapSize(1);
    mapSizeY = mapSize(2);
    
    % Target move direction (totalMoves in the total path)
    move = DirToMove(dir);
    
    if totalMoves ~= 0
        moveStep = pathLength/totalMoves;
        if mod(currentStep,moveStep) == 0   % Shift the belief map every N steps
            tmp_map = noncircshift(map, move);
            map = tmp_map ./sum(tmp_map(:));    % Scale it to 1
        end
    end
    
    pSensorNoDetection = ones(mapSizeX,mapSizeY); % Initialize the probability of no detection with ones
    pSensorNoDetection(location.y,location.x) = 0; % For binary sensor model, the probability of no detection at UAV location is zero
    newMap = pSensorNoDetection .* map;   % Update the belief map
    scaleFactor = sum(newMap(:));    % Calculate the scaling factor
    newMap = newMap ./ scaleFactor;  % Scale the updated belief map   
end