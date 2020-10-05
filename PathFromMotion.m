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


% Create a search path from the encoded motions
%
function path=PathFromMotion(position,model)

    n=model.n;
    xs = model.xs;
    ys = model.ys;
    path = zeros(n,2);  % path initialisation
    currentNode = [xs ys];
    for i=1:n
        motion = position(i,:);
        nextMove = MotionDecode(motion);
        nextNode = currentNode + nextMove;
        
        % Limit the path to be within the map
        % x direction
        if nextNode(1) > model.xmax
            nextNode(1) = model.xmax;
        elseif nextNode(1) < model.xmin
            nextNode(1) = model.xmin;
        end
        % y direction  
        if nextNode(2) > model.ymax
            nextNode(2) = model.ymax;
        elseif nextNode(2) < model.ymin
            nextNode(2) = model.ymin;    
        end
        path(i,:) = currentNode;
        currentNode = nextNode;
    end
end