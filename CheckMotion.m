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

% Check if the encoded motion forms a valid path
%
function valid = CheckMotion(position, model)
    
    % Load model parameters
    n=model.n;
    xs = model.xs;
    ys = model.ys;
    path = zeros(n,2);  % The path consists of n nodes, each node is (x,y)
    currentNode = [xs ys];
    valid = true;
    
    % Decode from the motions to a path
    for i=1:n
        motion = position(i,:);
        nextMove = MotionDecode(motion);  
        nextNode = currentNode + nextMove;
        % Out of map boundary
        if nextNode(1) > model.xmax || nextNode(1) < model.xmin...
            || nextNode(2) > model.ymax || nextNode(2) < model.ymin
            valid = false;
            return
        end            
        path(i,:) = nextNode;
        currentNode = nextNode;
    end
   
    % Check duplicate rows
    [u,I,J] = unique(path, 'rows', 'first');
    if size(u,1) < size(path,1)
        valid = false;
        return
    end

end