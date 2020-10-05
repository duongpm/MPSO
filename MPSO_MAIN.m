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


% Main program: The Motion-encoded Partical Swarm Optimization (MPSO)
%
% Find a path that maximizes the probability of finding object
% 

clc;
clear;
close all;

%% Create the search scenario

model = CreateModel(); % Create search map and parameters

CostFunction=@(x) MyCost(x,model);    % Cost Function

nVar = model.n;       % Number of Decision Variables = searching dimension of PSO = number of movements

VarSize=[nVar 2];   % Size of Decision Variables Matrix

VarMin=-model.MRANGE;           % Lower Bound of particles (Variables)
VarMax = model.MRANGE;          % Upper Bound of particles 

%% PSO Parameters

MaxIt=100;          % Maximum Number of Iterations

nPop=1000;           % Population Size (Swarm Size)

w=1;                % Inertia Weight
wdamp=0.98;         % Inertia Weight Damping Ratio
c1=2.5;             % Personal Learning Coefficient
c2=2.5;             % Global Learning Coefficient

alpha= 2;
VelMax=alpha*(VarMax-VarMin);    % Maximum Velocity
VelMin=-VelMax;                    % Minimum Velocity

%% Initialization

% Create an Empty Particle Structure
empty_particle.Position=[];
empty_particle.Velocity=[];
empty_particle.Cost=[];
empty_particle.Best.Position=[];
empty_particle.Best.Cost=[];

% Initialize Global Best
GlobalBest.Cost = -1; % Maximization problem

% Create an empty particle matrix, each particle is a solution (searching path)
particle=repmat(empty_particle,nPop,1);

% Initialization Loop
for i=1:nPop
    
    % Initialize Position
    particle(i).Position=CreateRandomSolution(model);
    
    % Initialize Velocity
    particle(i).Velocity=zeros(VarSize);
    
    % Evaluation
    costP = CostFunction(particle(i).Position);
    particle(i).Cost= costP;
    
    % Update Personal Best
    particle(i).Best.Position=particle(i).Position;
    particle(i).Best.Cost=particle(i).Cost;
    
    % Update Global Best
    if particle(i).Best.Cost>GlobalBest.Cost
        GlobalBest=particle(i).Best;
    end
    
end

% Array to Hold Best Cost Values at Each Iteration
BestCost=zeros(MaxIt,1);

%% PSO Main Loop
for it=1:MaxIt
    for i=1:nPop
                
        % Update Velocity
        particle(i).Velocity = w*particle(i).Velocity ...
            + c1*rand(VarSize).*(particle(i).Best.Position-particle(i).Position) ...
            + c2*rand(VarSize).*(GlobalBest.Position-particle(i).Position);
        
        % Update Velocity Bounds
        particle(i).Velocity = max(particle(i).Velocity,VelMin);
        particle(i).Velocity = min(particle(i).Velocity,VelMax);
        
        % Update Position
        particle(i).Position = particle(i).Position + particle(i).Velocity;
        
        % Update Position Bounds
        particle(i).Position = max(particle(i).Position,VarMin);
        particle(i).Position = min(particle(i).Position,VarMax);
        
        % Evaluation
        costP = CostFunction(particle(i).Position);
        particle(i).Cost = costP;

        % Update Personal Best
        if particle(i).Cost > particle(i).Best.Cost
            
            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;
            
            % Update Global Best
            if particle(i).Best.Cost > GlobalBest.Cost
                GlobalBest=particle(i).Best;
            end
            
        end
      
    end
    
    % Update Best Cost Ever Found
    BestCost(it)=GlobalBest.Cost;
    
    % Inertia Weight Damping
    w=w*wdamp;

    % Show Iteration Information
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
end

%% Results

% Updade Map in Accordance to the Target Moves
targetMoves = model.targetMoves; % Number of Target Moves (Zero means static)
moveDir = DirToMove(model.targetDir); % Direction of the Target Movement
moveArr = targetMoves*moveDir;
updatedMap = noncircshift(model.Pmap, moveArr);
newModel = model;
newModel.Pmap = updatedMap;

% Plot Solution
figure(1);
path=PathFromMotion(GlobalBest.Position,model); % Convert from Motion to Cartesian Space 
PlotSolution(path,newModel);  

% Plot Best Cost Over Iterations
figure(2);
plot(BestCost,'LineWidth',2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;
