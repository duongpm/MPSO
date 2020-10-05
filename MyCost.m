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

% Calculate the cost associated to a search path
% Return: costP - The cumulative probability of detection

function costP=MyCost(position,model)
    
    if ~CheckMotion(position, model)  % Invalid path
        costP = 0;                              % Punish invalid paths
        return;
    else
        % Input solution
        % Note: Position here is actually a search path consisting a number of
        % nodes
        path=PathFromMotion(position,model);
        x=path(:,1);
        y=path(:,2);

        % Input map
        Pmap=model.Pmap;
        N = model.n; % path length
        
        % Target movement
        targetMoves = model.targetMoves; % total moves of the target (zero means static)
        targetDir = model.targetDir;
        
        pNodetectionAll = 1; % Initialize the Probability of No detection at all
        pDetection = zeros(N); % Initialize the Probability of detection at each time step

        % Calculate the cost
        for i=1:N
            location.x = x(i) + model.xmax + 1;  % The location is shifted to the range of [1,MAPSIZE]
            location.y = y(i) + model.ymax + 1;
            [scaleFactor,Pmap] = UpdateMap(i,N,targetMoves,targetDir,location,Pmap); % Update the probability map

            pNoDetection = scaleFactor; % Probability of No Detection at time t is Exactly the scaling factor
            pDetection(i) = pNodetectionAll*(1 - pNoDetection); % Probability of Detection for the first time at time i
            pNodetectionAll = pNodetectionAll * pNoDetection;   
        end
             
        costP = 1 - pNodetectionAll;  % Return the Cumulative Probability of detection up to now (P = 1 - R)
    end
end