%% Code to perform the collective navigation described in "Modelling collective
%% navigation via nonlocal communication" by Johnston and Painter. This is
%% highest level script for the idealised information fields. Note this file
%% requires the look-up table for the concentration parameter, which can found
%% at https://melbourne.figshare.com/articles/dataset/kappaCDFLookupTable_mat/14551614.

%% This version has individual trustworthiness parameters and individual navigation skill
%% (which for now only works with the fixe information field)

clear all
close all

% Loop over sensing ranges
for sensingRange = [0, 5, 10, 20, 50, 500]
%% User settings
nRepeats = 10;                                                  % Number of realisations of the model.
nSavePoints = 501;                                              % Number of time points to save model output.
load('kappaCDFLookupTable.mat');                                % Load the lookup table for estimating the vM concentration parameter.

% Path for output csv's. 
savePath = '../cooperative/g1k0.1n55_g1k7.5227n45/';

backgroundFieldType = 'Fixed';   % Choose type of background field, choice of 'Void', 'Fixed','Random','Void', 'Increasing', 'Decreasing', 'Brownian'.
noiseInfluence = 'Information'; % Choose type of noise influence either 'Information' or 'Range'. All results generated with 'Information' except for F9.
flowField = 0;                  % Flow field (unused).
flowDirection = 0;              % Flow direction (unused).
flowVelocity = 0;               % Flow velocity (unused).

totalStepCountLoop = 0;         % Number of reorientation events.
nHistDirection = 60;            % Number of points in histograms.
directionHist = zeros(nHistDirection-1,1); %Predefine direction histogram.
cbar = [linspace(40,115,20)',linspace(36,213,20)',linspace(108,236,20)']/255; %Define colormap.

domainWidth = 400;              % Width of the domain.
domainHeight = 300;             % Height of the domain.    
velocity = 1;                   % Speed of individuals.
runTime = 1;                    % Mean reorientation time.
tEnd = 1000;                    % End of simulation.

% Weightings between own information and observed neighbours
alpha = 10/20;                  % Weighting of observations for heading calculation.
beta = 10/20;                   % Weighting of observations for concentration calculation.

goalDistance = 10;              % Distance from goal to be counted as "arrived".
noiseWavelength = 6;            % Frequency of noise structure in the Brownian noise field only.
    
goalLocation = [0,0];           % Location of target.
holeLocation = [125,175];       % Location of information void.
    
navigationField = @(x,y) atan2(goalLocation(2)-y,goalLocation(1)-x) ;       % Direction of target.

cooperative = true;            % Controls whether arrived whales stay in simulation and signal location

%sensingRange = 500;             % Perceptual range of individuals.
backgroundStrength = 1;         % Background information level.
repulsionDistance = 0;          % Repulsion mechanism (unused).
alignDistance = sensingRange;   % Alignment distance (always = sensing range).
attractDistance = sensingRange; % Attraction mechanism (unused).


%% Set population structure
% Initially: Uniform trustworthiness, two classes of navigation skill

% New population structure
% [[class 1 ID, class 1 gamma, class 1 kappa, class 1 n]; 
% [class 2 ID, class 2 gamma, class 2 kappa, class 2 n]]

nIndividualsStart = 100;    % Total population size

% gamma = trustworthiness
% kappa = navigation skill
gamma_1 = 1;                % Trustworthiness of class 1. Always set to 1
kappa_1 = 0.1;              % Navigation skill of class 1.
n_1 = 55;                   % Number of individuals in class 1.

delta = n_1/nIndividualsStart; % Fraction of population in class 1.

% Solve for class 2 parameters
gamma_2 = 1;                % Uniform trustworthiness
n_2 = nIndividualsStart - n_1;      % Number of individuals in class 2


 % Solve for kappa_2, so that average individual velocity towards target is unchanged from
 % a uniform population with kappa = 1.
[kappa_2, err] = solveSkill(delta, kappa_1);
err
populationStructure = [[1, gamma_1, kappa_1, n_1]; [2, gamma_2, kappa_2, n_2]];

% Set up vectors to keep track of individual's class, trustworthiness, and
% individual skill during the runs.

classes = [];                        % Class.
gamma = [];                          % Trustworthiness values.
individual_kappas = [];              % Navigation skills.

for i = 1:size(populationStructure,1)
    classes = [classes; populationStructure(i,1) * ones(populationStructure(i,4),1)];
    gamma = [gamma; populationStructure(i,2) * ones(populationStructure(i,4),1)];
    individual_kappas = [individual_kappas; populationStructure(i,3) * ones(populationStructure(i,4),1)];
end

% Keep track of individual ID's during run. Needed for filling out the
% arrival time matrix.
individualIDs = [1:nIndividualsStart]';


% Check that I haven't done anything dumb here
assert(length(gamma) == nIndividualsStart)


% Add extra dimensions to the data saving arrays to store data for 
% each class of individual
numClasses = size(populationStructure, 1);



%% Set up matrices for saving data

finalTime = zeros(nRepeats,1);                                  % Time for all individuals to arrive at the goal.
xPosition = zeros(nSavePoints,nRepeats);                        % Mean position (x) of the population.
yPosition = zeros(nSavePoints,nRepeats);                        % Mean position (y) of the population.
clusterMeasure = zeros(nSavePoints,nRepeats);                   % Measure of clustering of the population.
meanNeighbours = zeros(nSavePoints,nRepeats);                   % Mean number of neighbours within perceptual range.
distanceToGoal = zeros(nSavePoints,nRepeats);                   % Mean distance to goal of the population.
meanDifferenceDirection = zeros(nSavePoints,nRepeats);          % Mean error in heading relative to target.
nIndividualsRemaining = zeros(nSavePoints,nRepeats);            % Number of individuals remaining in the simulation (i.e. yet to arrive at goal).
majorityGone = zeros(nRepeats,1);                               % Time for 90% of the individuals to arrive at the goal.
distanceToGoalAll = zeros(nSavePoints, nRepeats);               % Average distance to goal including those which have arrived at the target.
meanNeighboursIncArrived = zeros(nSavePoints, nRepeats);        % Mean neighbours of still navigating agents, where neighbours at the target are
                                                                % counted. Only relevant when cooperative == true.
xPositionAll = zeros(nSavePoints,nRepeats);                     % Mean position (x) of the population including arrived agents.
yPositionAll = zeros(nSavePoints,nRepeats);                     % Mean position (y) of the population including arrived agents.
concentrationParameters = zeros(nSavePoints, nRepeats);         % Store the average concentration parameter at each step
% Page 1 = All individuals, Page 2 = class 1, ..., Page N+1 = class N
for i = 2:numClasses+1
    finalTime(:,i) = 0;
    xPosition(:,:,i) = 0;
    yPosition(:,:, i) = 0;
    meanNeighbours(:,:, i) = 0;
    distanceToGoal(:,:, i) = 0;
    meanDifferenceDirection(:,:, i) = 0;
    nIndividualsRemaining(:,:, i) = 0;
    majorityGone(:, i) = 0;
    directionHist(:, i) = 0;
    concentrationParameters(:,:, i) = 0;
    distanceToGoalAll(:,:,i) = 0;              
    meanNeighboursIncArrived(:,:,i) = 0;
    xPositionAll(:,:,i) = 0;
    yPositionAll(:,:, i) = 0;
end

% Save the arrival time of each individual for each repeat + class info
arrivalTimes = NaN(nIndividualsStart, 3 + nRepeats);
% First colum = class
arrivalTimes(:, 1) = classes;
arrivalTimes(:, 2) = gamma;
arrivalTimes(:, 3) = individual_kappas;


%% Main body of simulation, loops over number of realisations.
for iRepeat = 1:nRepeats
    majorityCheck = zeros(1 + numClasses);                      % Check if 90% of population has arrived at the target.
    iRepeat                                 % Print the realisation.

    nIndividuals = nIndividualsStart;       % Number of indivuduals in the simulation.

    t = 0;                                  % Initialise time counter.

    defineBackgroundFields;                 % Define noise and background fields.
    
    initialPosition = zeros(nIndividuals,2);                            % Initial location of individuals.
    initialPosition(:,2) = -20+40*rand(nIndividuals,1);                 % Initial position (y) of individuals.
    initialPosition(:,1) = domainWidth-120+40*rand(nIndividuals,1);     % Initial position (x) of individuals.
    position = initialPosition;                                         % Position of individuals.                             
    pairDistanceVec = pdist(position);                                  % Calculate distances between all pairs of individuals.
    pairDistances = squareform(pairDistanceVec);                        % Pair distance matrix
    
    turningTime = exprnd(runTime,nIndividuals,1);                       % Calculate durations of run events.
    timeToUpdate = turningTime;                                         % Calculate time until reorientation events.
    
    heading = zeros(nIndividuals,1);                                    % Headings of individuals.
    
    concentrationIndividual = zeros(nIndividuals,1);                    % Concentration parameter of individuals. 
    
    runGamma = gamma;                                                   % Copy of individual weightings from which agents can be dropped.
    runKappa = individual_kappas;                                       % Navigation skill of individuals.
    runClass = classes;                                                 % Class (for each gamma and kappa pair) for each individual.
    runIDs = individualIDs;                                             % ID for each individual.
    
    arrivedPosition = [];                                               % Positions of agents which have reached target.
    arrivedGamma = [];                                                  % Weightings of individuals which have arrived.
    arrivedKappa = [];                                                  % Skill of individuals which have arrived.
    arrivedClass = [];                                                  % Class of individuals which have arrived.
    arrivedIDs = [];                                                    % IDs of individuals which have arrived.
    
    % Sample individual headings based on individual navigation skill
    for i = 1:nIndividuals
        heading(i) = circ_vmrnd(navigationField(position(i,1),position(i,2)), ...
            individual_kappas(i),1);
    end
    
    meanPosition = zeros(tEnd,2);                                       % Calculate mean position of the population - doesn't appear to be used
    tSave = linspace(0,tEnd,nSavePoints);                               % Time points where the data will be saved.   
    tSaveCount = 1;                                                     % Count of time points saved.
    totalStepCount = 0;                                                 % Number of steps taken.
    
    % Main loop of the individual simulations, run until end of simulation
    % or all individuals have arrived at the target.
    while t < tEnd && nIndividuals > 0
        
        totalStepCount = totalStepCount + 1;                            % Keep track of total steps taken.
        totalStepCountLoop = totalStepCountLoop + 1;                    % Keep track of overall total steps (i.e. over repeats)
        
%         SW: Don't know what this is doing, seem to run into errors on
%         long runs here as the variables aren't defined
%         % If sufficiently many steps taken, add extra preallocated vectors.
%         if mod(totalStepCountLoop,1e6) == 0
%             reorientation = [reorientation;zeros(1e6,1)];
%             navError = [navError;zeros(1e6,1)];
%             distToGoal = [distToGoal;zeros(1e6,1)];
%             neighboursOut = [neighboursOut;zeros(1e6,1)];
%         end

        [nextUpdate,nextAgent] = min(timeToUpdate);                     % Calculate next reorientation event time and agent.
        timeToUpdate = timeToUpdate - nextUpdate;                       % Update time to update for all individuals.
        timeElapsed = nextUpdate;                                       % Calculate time step length.
        t = t+nextUpdate;                                               % Update time.
        % Update the position of all individuals. Flow field is not used.
        position = position + velocity*timeElapsed*[cos(heading),sin(heading)] + flowField*flowVelocity*[cos(flowDirection),sin(flowDirection)];
        pairDistanceUpdate;                                             % Update pair distances for all pairs of individuals.
        pairDistances(1:nIndividuals+1:end) = 1e10;                     % Avoid influence of pairs of identical individuals.
        
        % Find individuals within the perceptual range of the individual
        % undergoing reorientation.
        neighbours = find(pairDistances(nextAgent,:)>0&pairDistances(nextAgent,:)<sensingRangeField(position(nextAgent,1),position(nextAgent,2)));
        nNeighbours = numel(neighbours);                                % Number of individuals within perceptual range.
        [minDistance,closestAgent] = min(pairDistances(nextAgent,:));   % Find closest agent.
        oldHeading = heading;                                           % Retain previous heading.
        
        
        % Find arrived individuals within the perceptual range.
        arrivedDistances = [];
        arrivedNeighbours = [];
        if ~isempty(arrivedPosition)
            arrivedDistances = pdist2(position(nextAgent,:), arrivedPosition);      % Distance to current agent
            arrivedNeighbours = find(arrivedDistances < sensingRangeField(position(nextAgent,1),position(nextAgent,2)));
        end
        
        nArrivedNeighbours = numel(arrivedNeighbours);
        
        nNeighbours = nNeighbours + nArrivedNeighbours;
        
        
        % Calculate sample heading based on inherent information/individual
        % skill only.
        potentialHeading = circ_vmrnd(navigationField(position(nextAgent,1),position(nextAgent,2)),...
            individual_kappas(nextAgent),1);
        
        % Update heading based on other observed individuals if number of
        % neighbours exceeds zero.
        if nNeighbours > 0 % && minDistance < sensingRangeField(position(nextAgent,1),position(nextAgent,2)) (redundant I think)
            % Repulsion mechanism unused.
            if minDistance < repulsionDistance
                heading(nextAgent) = atan2(-position(closestAgent,2)+position(nextAgent,2), ...
                    -position(closestAgent,1)+position(nextAgent,1));
            % Alignment mechanism.    
            elseif minDistance < alignDistance     
                
                % Case: cooperative navigation - use positions of arrived agents.
                if cooperative == true 
                    % Calculate angles from nextAgent to arrived neighbours.
                    anglesToArrived = [];
                    if nArrivedNeighbours > 0
                        arrivedNeighbourPositions = arrivedPosition(arrivedNeighbours, :);         % Positions of arrived neighbours.
                        anglesToArrived = zeros(nArrivedNeighbours,1);
                        for i = 1:nArrivedNeighbours
                            anglesToArrived(i) = atan2(arrivedNeighbourPositions(i,2) - position(nextAgent,2), ...
                                arrivedNeighbourPositions(i,1) - position(nextAgent,1));
                        end
                    end
                    allNeighbourHeadings = [heading(neighbours); anglesToArrived];          % Headings of navigating neighbours and angles to 
                                                                                            % arrived neighbours.                                                               
                    allNeighbourGammas = [runGamma(neighbours); arrivedGamma(arrivedNeighbours)];  % Weights for navigating and arrived neighbours.
                    
                    % Calculate reorientation parameters
                    weightedNeighbourHeading = circ_mean(allNeighbourHeadings, allNeighbourGammas);
                    bestGuessHeading = circ_mean([weightedNeighbourHeading;potentialHeading],[1-alpha;alpha]);   % MLE of heading.
                    w = [(1-beta)*allNeighbourGammas; beta*sum(allNeighbourGammas)];                     % Individual weightings for concentration parameter
                    alphaLookup = [allNeighbourHeadings; potentialHeading];                              % Set of observed headings.
                   
                % Case: Non-cooperative. Positions of arrived agents unused
                else                      
                    weightedNeighbourHeading = circ_mean(heading(neighbours), runGamma(neighbours));
                    bestGuessHeading = circ_mean([weightedNeighbourHeading;potentialHeading],[1-alpha;alpha]);     % MLE of heading.
                    w = [(1-beta)*runGamma(neighbours); beta*sum(runGamma(neighbours))];                           % Individual weightings for concentration parameter
                    alphaLookup = [heading(neighbours);potentialHeading];                                          % Set of observed headings.
                end
                
                circ_kappa_script;                                                                                  % Calculate estimate of concentration parameter.
                bestGuessStrength = kappa;                                                                          % Estimate of concentration parameter.
                heading(nextAgent) = circ_vmrnd(bestGuessHeading,bestGuessStrength,1);                              % Set new heading.
                
                % Store the agent's new concentration parameter
                concentrationIndividual(nextAgent) = kappa;
                
            % Attraction mechanism unused.
            elseif minDistance < attractDistance
                heading(nextAgent) = atan2(mean(position(neighbours,2))-position(nextAgent,2),...
                    mean(position(neighbours,1))-position(nextAgent,1));
            end
        else
            heading(nextAgent) = potentialHeading;
        end
         
        timeToUpdate(nextAgent) = exprnd(runTime,1);                    % New duration of run.
        pairDistances(1:nIndividuals+1:end) = 0;                        % Set pair distances to zeros for identical individuals.
        
        % Storage of data at specific time points
        if t > tSave(tSaveCount)
            saveData;
        end
        
        % Determine which individuals have arrived at the target and remove
        % from simulation.
        removal = [];
        for i = 1:nIndividuals
            if sqrt((position(i,1)-goalLocation(1))^2+(position(i,2)-goalLocation(2))^2) < goalDistance
                removal = [removal;i];
            end
        end
        
        % Store information on the removed individuals (do this before
        % removing them ...)
        arrivedPosition = [arrivedPosition; position(removal,:)];           % Positions of agents which have reached target.
        arrivedGamma = [arrivedGamma; runGamma(removal)];                   % Weightings of individuals which have arrived.
        arrivedKappa = [arrivedKappa; runKappa(removal)];                   % Skill of individuals which have arrived.
        arrivedClass = [arrivedClass; runClass(removal)];                   % Class of individuals which have arrived.
        arrivedIDs = [arrivedIDs; runIDs(removal)];                         % ID of individuals which have arrived.
        
        
        % Save the arrival time of the agents arriving at target. 
        arrivalTimes(runIDs(removal), iRepeat + 3) = t;
        
        % Remove arived agents from active navigation
        position(removal,:) = [];                                           % Remove individuals from position.
        heading(removal) = [];                                              % Remove individuals from heading.
        timeToUpdate(removal) = [];                                         % Remove individuals from reorientation.
        concentrationIndividual(removal) = [];                              % Remove individuals from concentration.
        runGamma(removal) = [];                                             % Remove individuals from weightings.
        runClass(removal) = [];                                             % Remove individuals from classes.
        runKappa(removal) = [];                                             % Remove individuals from navigation skills.
        runIDs(removal) = [];
        nIndividuals = nIndividuals - numel(removal);                       % Number of individuals remaining.
        
       
        
    end
    
    finalTime(iRepeat) = t;                                                 % Final time in the simulation.
    
end

% Calculate the means over the realisations for each variable.

% Non segmented variables
clusterMeasure = mean(clusterMeasure,2);                                    % Mean of clustering across realisation loop.

% Segmented variables, i.e. we have data separated for each class.
% First page = all agents, page 2 = class 1, ...
xPositionMean = squeeze(mean(xPosition,2));                                          % Mean of average x position across realisation loop.
distanceToGoal = squeeze(mean(distanceToGoal,2));                                    % Mean of average distance to goal across realisation loop.
meanNeighbours = squeeze(mean(meanNeighbours,2));                                    % Mean of average number of neighbours across realisation loop.
meanDifferenceDirection = squeeze(mean(meanDifferenceDirection,2));                  % Mean of difference between heading and target across realisation loop.
nIndividualsRemaining = squeeze(mean(nIndividualsRemaining,2));                      % Mean of number individuals remaining across realisation loop.
concentrationMean = squeeze(mean(concentrationParameters, 2));                       % Mean of the concentration parameters over realisation loop.
majorityGoneMean = mean(majorityGone, 1); 

% Add in first two rows stating the gamma and kappa values for each column.
row0 = [0, populationStructure(:,2)'];                                       % Gamma values
row1 = [0, populationStructure(:,3)'];                                       % Kappa values

xPositionMean = [row0; row1; xPositionMean];
distanceToGoal = [row0; row1; distanceToGoal];
meanNeighbours = [row0; row1; meanNeighbours];
meanDifferenceDirection = [row0; row1; meanDifferenceDirection];
nIndividualsRemaining = [row0; row1; nIndividualsRemaining];
concentrationMean = [row0; row1; concentrationMean];
majorityGoneMean = [row0; row1; majorityGoneMean];
directionHist = [row0; row1; directionHist];

% Save the data into tables then into CSV's.
% First get rid of singleton dimension, and turn into a table
xPositionMean = array2table(xPositionMean);
distanceToGoal = array2table(distanceToGoal);
meanNeighbours = array2table(meanNeighbours);
meanDifferenceDirection = array2table(meanDifferenceDirection);
nIndividualsRemaining = array2table(nIndividualsRemaining);
concentrationMean = array2table(concentrationMean);
directionHist = array2table(directionHist);
majorityGoneMean = array2table(majorityGoneMean);


% Add the column names - describe which agents each column refers to
colnames = cell(numClasses + 1,1);
colnames(1) = {char("all")};





for i = 1:numClasses
    page = i + 1;
    colnames(page) = {char(sprintf("class_%d",populationStructure(i, 1)))};
end

% Add the column names to the tables 

xPositionMean.Properties.VariableNames = colnames;
distanceToGoal.Properties.VariableNames = colnames; 
meanNeighbours.Properties.VariableNames = colnames;
meanDifferenceDirection.Properties.VariableNames = colnames; 
nIndividualsRemaining.Properties.VariableNames = colnames; 
concentrationMean.Properties.VariableNames = colnames;
directionHist.Properties.VariableNames = colnames;
majorityGoneMean.Properties.VariableNames = colnames;

% Save each variable to a csv
fileTail = sprintf('_range_%d_g%.2fk%.3fn%d_g%.2fk%.3fn%d.csv', ...
    sensingRange, populationStructure(1,2),populationStructure(1,3),populationStructure(1,4), ...
    populationStructure(2,2),populationStructure(2,3),populationStructure(2,4));   % SW: Keep track of range parameter and population structure for saved data
writetable(xPositionMean, strcat(savePath, 'xPosition', fileTail));  
csvwrite(strcat(savePath, 'clusterMeasure', fileTail), clusterMeasure);
writetable(distanceToGoal, strcat(savePath, 'distanceToGoal', fileTail));
writetable(meanNeighbours, strcat(savePath, 'meanNeighbours', fileTail));
writetable(meanDifferenceDirection, strcat(savePath, 'meanDifferenceDirection', fileTail));
writetable(nIndividualsRemaining, strcat(savePath, 'nIndividualsRemaining', fileTail));
writetable(concentrationMean, strcat(savePath, 'meanConcentration', fileTail));
writetable(directionHist, strcat(savePath, 'directionHist', fileTail));
writetable(majorityGoneMean, strcat(savePath, 'meanMajorityGone', fileTail));

% Lastly, save the arrival times of each individual
arrivalTimeColnames = cell(nRepeats+3, 1);
arrivalTimeColnames(1) = {char("Class")};
arrivalTimeColnames(2) = {char("gamma")};
arrivalTimeColnames(3) = {char("kappa")};
for i = 1:nRepeats
    arrivalTimeColnames(i + 3) = {char(sprintf("iRepeat_%d",i))};
end
arrivalTimes = array2table(arrivalTimes);
arrivalTimes.Properties.VariableNames = arrivalTimeColnames;
writetable(arrivalTimes, strcat(savePath, 'arrivalTimes', fileTail));




clear kappaCDF                                                              % Clear CDF to avoid saving over and over.

% Plot relevant results
plotResults;

end