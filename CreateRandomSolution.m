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

%
% Create random paths (solutions)
% 

function position=CreateRandomSolution(model)

    n=model.n;  % Load the number of path nodes
    startNode = [model.xs model.ys];
    path = zeros(n,2);  % path initialization
    MaxIt = 100; % Maximum number of trial iterations before resetting the path

    motions = [1 0;
                 0.7071 0.7071;
                 0 1;
                 -0.7071 0.7071;
                 -1 0;
                 -0.7071 -0.7071;
                 0 -1;
                 0.7071 -0.7071];
             
    should_restart = true;
    
    % Repeat until generating a valid path
    while should_restart
        should_restart = false;
        for i = 1:n
            path(i,:) = startNode;
        end
        position = zeros(n,2); % motion initialisation
        currentNode = startNode;
        for i=1:n
            motion = motions(randi([1 length(motions)]),:);
            invalidFlag = true;
            it = 0;
            while (invalidFlag && it < MaxIt)
                nextMove = MotionDecode(motion);
                nextNode = currentNode + nextMove;
                invalidFlag = false;

                % Limit the path to be within the map
                % Out of x direction -> Move it back
                if nextNode(1) > model.xmax
                    motion = motions(5,:);
                    invalidFlag = true;
                    it = it + 1;
                elseif nextNode(1) < model.xmin
                    motion = motions(1,:);
                    invalidFlag = true;
                    it = it + 1;
                    
                % Out of y direction 
                elseif nextNode(2) > model.ymax
                    motion = motions(7,:);
                    invalidFlag = true;
                    it = it + 1;
                elseif nextNode(2) < model.ymin
                    motion = motions(3,:);
                    invalidFlag = true;
                    it = it + 1;
                else
                    % Check duplicate nodes within a path
                    for j=1:length(path)
                        if isequal(nextNode,path(j,:))
                            motion = motions(randi([1 length(motions)]),:);
                            invalidFlag = true;
                            it = it + 1;
                            break;
                        end
                    end 
                end
            end
               
            % Restart the whole path
            if (it >= MaxIt)
                should_restart = true;
                break;
            else    % Path ok
                path(i,:) = nextNode;
                currentNode = nextNode;
                position(i,:) = motion;
            end
        end
    end
end