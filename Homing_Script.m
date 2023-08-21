%% Code to perform the collective navigation described in "Modelling collective
%% navigation via nonlocal communication" by Johnston and Painter. This is
%% highest level script for the idealised information fields. Note this file
%% requires the look-up table for the concentration parameter, which can found
%% at https://melbourne.figshare.com/articles/dataset/kappaCDFLookupTable_mat/14551614.

%% This version has individual trustworthiness parameters and individual navigation skill
%% (which for now only works with the fixed information field)

clear all
close all

for slowClassGamma = [1]
slowClassGamma

% Loop over sensing ranges
for sensingRange = [0, 5, 10, 20, 50, 100, 200, 500]
sensingRange
%% User settings
nRepeats = 10;                                                  % Number of realisations of the model.
nSavePoints = 501;                                              % Number of time points to save model output.`
load('kappaCDFLookupTable.mat');                                % Load the lookup table for estimating the vM concentration parameter.

startDist = 300;                        % Initial distance from the target

% Path for output csv's. 
savePath = sprintf('/Users/boppin/Documents/work/Whales/collective-navigation-2/skill+weighting/outputs/g%.1fk0.5n50_g1k1.7408n50/', slowClassGamma);

backgroundFieldType = 'Fixed';   % Choose type of background field, choice of 'Void', 'Fixed','Random','Void', 'Increasing', 'Decreasing', 'Brownian'.
noiseInfluence = 'Information'; % Choose type of noise influence either 'Information' or 'Range'. All results generated with 'Information' except for F9.
flowField = 0;                  % Flow field (unused).
flowDirection = 0;              % Flow direction (unused).
flowVelocity = 0;               % Flow velocity (unused).

totalStepCountLoop = 0;         % Number of reorientation events.

cbar = [linspace(40,115,20)',linspace(36,213,20)',linspace(108,236,20)']/255; %Define colormap.

domainWidth = 400;              % Width of the domain. SW: Not used
domainHeight = 300;             % Height of the domain.    SW: Not used
velocity = 1;                   % Speed of individuals.
runTime = 1;                    % Mean reorientation time.
tChunkSize = 1000;                  % Size of chunks to break data into

limitRun = true;                % If true, force the run to stop once t > tEnd.
                                 % If false, run will continue until all
                                 % individuals arive (may take a long time
                                 % depending on population).

tEnd = 1000;                     % Only used if limitRun == true.

% Weightings between own information and observed neighbours
alpha = 10/20;                  % Weighting of observations for heading calculation.
beta = 10/20;                   % Weighting of observations for concentration calculation.

goalDistance = 10;              % Distance from goal to be counted as "arrived".
noiseWavelength = 6;            % Frequency of noise structure in the Brownian noise field only.
    
goalLocation = [0,0];           % Location of target.
holeLocation = [125,175];       % Location of information void.
    
navigationField = @(x,y) atan2(goalLocation(2)-y,goalLocation(1)-x) ;       % Direction of target.

cooperative = "target";    % Controls whether arrived whales stay in simulation and signal location.
                                % cooperative = "off":
                                %     individuals which arrive at target
                                %     are removed from the simulation.
                                %     They are ignored in the reorientation
                                %     calculations of remaining agents.
                                
                                % cooperative = "target":
                                %     Same as cooperative = "individuals",
                                %     however arrived agents' positions are
                                %     read as being exactly at the target
                                %     (as opposed to their final
                                %     positions).
                                

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
gamma_1 = slowClassGamma;                % Trustworthiness of class 1
kappa_1 = 0.5;              % Navigation skill of class 1.
n_1 = 50;                   % Number of individuals in class 1.
 
 delta = n_1/nIndividualsStart; % Fraction of population in class 1.

% Solve for class 2 parameters
gamma_2 = 1;                % Second class trustworthiness. Always have faster class with trustworthiness 1.
n_2 = nIndividualsStart - n_1;      % Number of individuals in class 2


 % Solve for kappa_2, so that average individual velocity towards target is unchanged from
 % a uniform population with kappa = 1.
[kappa_2, err] = solveSkill(delta, kappa_1);

% Print out the error between the effective velocity for the selected
% parameters and the effective velocity of the uniform population with
% kappa = 1.
kappa_2
err
populationStructure = [[1, gamma_1, kappa_1, n_1]; [2, gamma_2, kappa_2, n_2]];
% populationStructure = [[1, gamma_1, kappa_1, n_1]];

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

% These will grow as required if the runs take longer.
finalTime = zeros(nRepeats, numClasses + 1);                                  % Time for all individuals to arrive at the goal.
xPosition = zeros(nSavePoints,nRepeats, numClasses + 1);                        % Mean position (x) of the population.
yPosition = zeros(nSavePoints,nRepeats, numClasses + 1);                        % Mean position (y) of the population.
clusterMeasure = zeros(nSavePoints,nRepeats, numClasses + 1);                   % Measure of clustering of the population.
meanNeighbours = zeros(nSavePoints,nRepeats, numClasses + 1);                   % Mean number of neighbours within perceptual range.
distanceToGoal = zeros(nSavePoints,nRepeats, numClasses + 1);                   % Mean distance to goal of the population.
meanDifferenceDirection = zeros(nSavePoints,nRepeats, numClasses + 1);          % Mean error in heading relative to target.
nIndividualsRemaining = zeros(nSavePoints,nRepeats, numClasses + 1);            % Number of individuals remaining in the simulation (i.e. yet to arrive at goal).
majorityGone = zeros(nRepeats, numClasses + 1);                                 % Time for 90% of the individuals to arrive at the goal.
distanceToGoalAll = zeros(nSavePoints, nRepeats, numClasses + 1);               % Average distance to goal including those which have arrived at the target.
meanNeighboursIncArrived = zeros(nSavePoints, nRepeats, numClasses + 1);        % Mean neighbours of still navigating agents, where neighbours at the target are
                                                                                % counted. Only relevant when cooperative ~= off.                                                                
xPositionAll = zeros(nSavePoints,nRepeats, numClasses + 1);                     % Mean position (x) of the population including arrived agents.
yPositionAll = zeros(nSavePoints,nRepeats, numClasses + 1);                     % Mean position (y) of the population including arrived agents.
concentrationParameters = zeros(nSavePoints, nRepeats, numClasses + 1);         % Store the average concentration parameter at each step
effectiveVelocity = NaN(nSavePoints, nRepeats, numClasses + 1);                 % Average velocity in direction of target.

nHistDirection = 60;                                                            % Number of points in histograms.
directionHist = zeros(nHistDirection-1, numClasses + 1);                        % Predefine direction histogram.

nHistVelocity = 81;                                                             % Number of points in effective velocity histogram.
velocityHist = zeros(nSavePoints, nHistVelocity - 1, numClasses + 1);           % Predefine time varying effective velocity histogram.

% Save trajectories only for the first repeat
xPositionsIndividualsRep1 = zeros(nSavePoints, nIndividualsStart);  % Store positions of each whale for each timestep of repeat 1. 
yPositionsIndividualsRep1 = zeros(nSavePoints, nIndividualsStart);  % Store positions of each whale for each timestep of repeat 1.

% Number of neighbours of each class, for each class. Dim 1 = "whose
% neighbours". I.e. Dim 1 = 1 gives the neighbours for class 1, dim 1 = 2
% gives the neighbours for class 2. 

% Dim 4 selects the class of the neighbours counted. Dim 4 = 1 means it's
% counting the number of neighbours of any class, dim 4 = 2 means it counts
% the number of neighbours of class 1, dim 4 = 3 means it counts the number
% of neighbours of class 2 etc. 
classSpecificNeighbours = NaN(numClasses, nSavePoints, nRepeats, numClasses + 1);


% Save the arrival time of each individual for each repeat + class info.
% To be used for creating arrival time histograms.
arrivalTimes = NaN(nIndividualsStart, 3 + nRepeats);
% First colum = class
arrivalTimes(:, 1) = classes;
arrivalTimes(:, 2) = gamma;
arrivalTimes(:, 3) = individual_kappas;


%% Main body of simulation, loops over number of realisations.

tMax = tChunkSize;          % Keep track of max number of chunks for data saving. Update during each repeat.
tSave = linspace(0,tChunkSize,nSavePoints);    % Time points where the data will be saved. Updates during runs.


for iRepeat = 1:nRepeats
    majorityCheck = zeros(1 + numClasses);                      % Check if 90% of population has arrived at the target.
    iRepeat                                 % Print the realisation.

    nIndividuals = nIndividualsStart;       % Number of indivuduals in the simulation.

    t = 0;                                  % Initialise time counter.

    defineBackgroundFields;                 % Define noise and background fields.
    
    initialPosition = zeros(nIndividuals,2);                            % Initial location of individuals.
    initialPosition(:,2) = -20+40*rand(nIndividuals,1);                 % Initial position (y) of individuals.
    initialPosition(:,1) = startDist - 20 + 40*rand(nIndividuals,1);     % Initial position (x) of individuals.     % Initial position (x) of individuals.
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
    
    
    tSaveCount = 1;                                                     % Count of time points saved.
    totalStepCount = 0;                                                 % Number of steps taken.
    
    % Main loop of the individual simulations, run until end of simulation
    % or all individuals have arrived at the target.
    while nIndividuals > 0 &&  (t < tEnd || limitRun == false)
        
        
        
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
        
        if cooperative == "target" && ~isempty(arrivedPosition)  % All arrived agents treated as being precisely at target.
            agentDistToTarget = sqrt((position(nextAgent,1) - goalLocation(1))^2 + (position(nextAgent,2) - goalLocation(2))^2);
            if agentDistToTarget < sensingRange
                arrivedNeighbours = 1:length(arrivedIDs);            % If agent within sensing range of target, all arrived agents are neighbours
                minDistance = min([minDistance, agentDistToTarget]);  % Update min distance if close to goal. minDistance controls if cooperative navigation is used
                                                                      % Not including this line was the reason for the single agent remaining oscilation bug.
           end
        end
        
        % If cooperative == "off", the arrivedNeighbours is left as empty.
        
        
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

                if cooperative == "target"  % Cooperative navigation where arrived whales are viewed as being precisely at target
                    
                    if nArrivedNeighbours > 0
                        angleToTarget = atan2(goalLocation(2) - position(nextAgent,2), goalLocation(1) - position(nextAgent,1));
                        arrivedHeadings = angleToTarget*ones(nArrivedNeighbours, 1);
                        
                    else    % No agents arrived, or nextAgent outside of target range
                        arrivedHeadings = [];
                    end
                    
                    allNeighbourGammas = [runGamma(neighbours); arrivedGamma(arrivedNeighbours)]; % arrivedGamma(arrivedNeighbours) will be empty if nextAgent 
                                                                                                  % not in range of target, or no whales have arrived.
                    allNeighbourHeadings = [heading(neighbours); arrivedHeadings];
                   
                    % Calculate reorientation parameters
                    weightedNeighbourHeading = circ_mean(allNeighbourHeadings, allNeighbourGammas);
                    bestGuessHeading = circ_mean([weightedNeighbourHeading;potentialHeading],[1-alpha;alpha]);   % MLE of heading.
                    w = [(1-beta)*allNeighbourGammas; beta*sum(allNeighbourGammas)];                     % Individual weightings for concentration parameter
                    alphaLookup = [allNeighbourHeadings; potentialHeading];                              % Set of observed headings.
                
                    
                    
                else        % Non-cooperative. Arrived agents are ignored.            
                    weightedNeighbourHeading = circ_mean(heading(neighbours), runGamma(neighbours));
                    bestGuessHeading = circ_mean([weightedNeighbourHeading;potentialHeading],[1-alpha;alpha]);     % MLE of heading.
                    w = [(1-beta)*runGamma(neighbours); beta*sum(runGamma(neighbours))];                           % Individual weightings for concentration parameter
                    alphaLookup = [heading(neighbours);potentialHeading];                                          % Set of observed headings.
                end
                
                circ_kappa_script;                                                                                  % Calculate estimate of concentration parameter.
                bestGuessStrength = kappa;                                                                          % Estimate of concentration parameter.
                heading(nextAgent) = circ_vmrnd(bestGuessHeading,bestGuessStrength,1);                              % Set new heading.
                if nArrivedNeighbours > 0
                    %"kappa"
                    %bestGuessStrength
                end
                
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
        
        
        % Add a new chunk of space to data matrices if t exceeds their prealocated size
        if t >= tMax
            t
            tSave = [tSave, linspace(tMax + 2,tMax + tChunkSize,nSavePoints -1)];       % Sort of set up for the nSavePoints = 1000, tSave = 501 case...
            
            % Add new space to data saving matrices
            xPosition = [xPosition; zeros(nSavePoints - 1, nRepeats, numClasses + 1)];
            yPosition = [yPosition; zeros(nSavePoints - 1, nRepeats, numClasses + 1)];
            meanNeighbours = [meanNeighbours; zeros(nSavePoints - 1, nRepeats, numClasses + 1)];
            distanceToGoal = [distanceToGoal; zeros(nSavePoints - 1, nRepeats, numClasses + 1)];
            meanDifferenceDirection = [meanDifferenceDirection; zeros(nSavePoints - 1, nRepeats, numClasses + 1)];
            nIndividualsRemaining = [nIndividualsRemaining; zeros(nSavePoints - 1, nRepeats, numClasses + 1)];
            concentrationParameters = [concentrationParameters; zeros(nSavePoints - 1, nRepeats, numClasses + 1)];
            distanceToGoalAll = [distanceToGoalAll; zeros(nSavePoints - 1, nRepeats, numClasses + 1)];              
            meanNeighboursIncArrived = [meanNeighboursIncArrived; zeros(nSavePoints - 1, nRepeats, numClasses + 1)];
            clusterMeasure = [clusterMeasure; zeros(nSavePoints - 1, nRepeats, numClasses + 1)];
            xPositionAll = [xPositionAll; zeros(nSavePoints - 1, nRepeats, numClasses + 1)];
            yPositionAll = [yPositionAll; zeros(nSavePoints - 1, nRepeats, numClasses + 1)];
            
            xPositionsIndividualsRep1 = [xPositionsIndividualsRep1; zeros(nSavePoints, nIndividualsStart)];
            yPositionsIndividualsRep1 = [yPositionsIndividualsRep1; zeros(nSavePoints, nIndividualsStart)];
            
            effectiveVelocity = [effectiveVelocity; zeros(nSavePoints - 1, nRepeats, numClasses + 1)];
            velocityHist = [velocityHist; zeros(nSavePoints - 1, nHistVelocity - 1, numClasses + 1)]; 
            

            % Concatenate along time dimension
            classSpecificNeighbours = cat(2, classSpecificNeighbours, NaN(numClasses, nSavePoints - 1, nRepeats, numClasses + 1));
            
            
            % New time corresponding to end of data saving matrices
            tMax = tMax + tChunkSize;
            
            
        end
        
       
        
    end
    
    finalTime(iRepeat) = t;                                                 % Final time in the simulation.
    
end

% Calculate the means over the realisations for each variable.

% Non segmented variables
clusterMeasure = squeeze(mean(clusterMeasure,2));                                    % Mean of clustering across realisation loop.

% Segmented variables, i.e. we have data separated for each class.
% First page = all agents, page 2 = class 1, ...
xPositionMean = squeeze(mean(xPosition,2));                                          % Mean of average x position across realisation loop.
distanceToGoal = squeeze(mean(distanceToGoal,2));                                    % Mean of average distance to goal across realisation loop.
meanNeighbours = squeeze(mean(meanNeighbours,2));                                    % Mean of average number of neighbours across realisation loop.
meanDifferenceDirection = squeeze(mean(meanDifferenceDirection,2));                  % Mean of difference between heading and target across realisation loop.
nIndividualsRemaining = squeeze(mean(nIndividualsRemaining,2));                      % Mean of number individuals remaining across realisation loop.
concentrationMean = squeeze(mean(concentrationParameters, 2));                       % Mean of the concentration parameters over realisation loop.
majorityGoneMean = mean(majorityGone, 1); 
distanceToGoalAll = squeeze(mean(distanceToGoalAll, 2));
meanNeighboursIncArrived = squeeze(mean(meanNeighboursIncArrived, 2));
effectiveVelocity = squeeze(mean(effectiveVelocity, 2, 'omitnan'));




% Save each variable to a csv. Keep track of run and population data in the
% filename.
fileTailStart = sprintf('_distance_%d_range_%d', startDist, sensingRange);
fileTailEnd = "";
for classIdx =  1:numClasses
    fileTailEnd = fileTailEnd + sprintf("_g%.2fk%.3fn%d", populationStructure(classIdx, 2), populationStructure(classIdx, 3), populationStructure(classIdx, 4));
end
fileTail = fileTailStart + fileTailEnd + ".csv";

% Save each of the matrices that are in standard form
tableSaver(xPositionMean, 'xPosition', populationStructure, fileTail, savePath, numClasses);
tableSaver(distanceToGoal, 'distanceToGoal', populationStructure, fileTail, savePath, numClasses);
tableSaver(clusterMeasure, 'clusterMeasure', populationStructure, fileTail, savePath, numClasses);
tableSaver(meanNeighbours, 'meanNeighbours', populationStructure, fileTail, savePath, numClasses);
tableSaver(meanDifferenceDirection, 'meanDifferenceDirection', populationStructure, fileTail, savePath, numClasses);
tableSaver(nIndividualsRemaining, 'nIndividualsRemaining',populationStructure, fileTail, savePath, numClasses);
tableSaver(concentrationMean, 'meanConcentration',populationStructure, fileTail, savePath, numClasses);
tableSaver(majorityGoneMean, 'meanMajorityGone', populationStructure, fileTail, savePath, numClasses);
tableSaver(directionHist, 'directionHist', populationStructure, fileTail, savePath, numClasses);
tableSaver(distanceToGoalAll, 'distanceToGoalAll', populationStructure, fileTail, savePath, numClasses);
tableSaver(meanNeighboursIncArrived, 'meanNeighboursIncArrived', populationStructure, fileTail, savePath, numClasses);
tableSaver(effectiveVelocity, 'meanEffectiveVelocity', populationStructure, fileTail, savePath, numClasses);

% Now save all of the other matrices which aren't in the standard form.


% Save the class specific neighbours.
% Average over the repeats. dim 3 is the dimension for the repeats. 
classSpecificNeighbours = squeeze(mean(classSpecificNeighbours, 3, 'omitnan'));
% There seems to be a bug when there is only one class, is it averaging
% over the wrong dimension in that case?

for sensingClass = 1:numClasses
    currentClassNeighbours = squeeze(classSpecificNeighbours(sensingClass, :, :));
    varName = "class" + sensingClass + "Neighbours";
    tableSaver(currentClassNeighbours, varName, populationStructure, fileTail, savePath, numClasses);
end

% Save the arrival times of each individual
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

% Save the time varying velocity histograms.
% Combine the histograms over several timesteps.
nTimeStepCombine = 50; 
nTimeBins = ceil(size(velocityHist,1)/nTimeStepCombine);

for page = 1:numClasses+1
    if page == 1
        class = "All";
    else
        class = page - 1;
    end
    
    velocityHistogramCurrent = velocityHist(:,:,page);
    
    velocityHistogramCurrentBinned = zeros(nTimeBins, nHistVelocity - 1);

    for i = 1:nTimeBins
        binEnd = min([i*nTimeStepCombine, size(velocityHistogramCurrent, 1)]);
        velocityHistogramCurrentBinned(i,:) = sum(velocityHistogramCurrent((i-1)*nTimeStepCombine + 1: binEnd, :), 1);
    end
    
    csvwrite(strcat(savePath, "velocityHistogramClass" + class, sprintf('_nTstep%d',nTimeStepCombine), fileTail),velocityHistogramCurrentBinned);
end


% Save trajectories for repeat 1.
% Add metadata as first two rows.
posRow1 = individualIDs';
posRow2 = classes';
xPositionsIndividualsRep1 = [posRow1; posRow2; xPositionsIndividualsRep1];
yPositionsIndividualsRep1 = [posRow1; posRow2; yPositionsIndividualsRep1];
csvwrite(strcat(savePath, 'xPositionsIndividuals', fileTail), xPositionsIndividualsRep1);
csvwrite(strcat(savePath, 'yPositionsIndividuals', fileTail), yPositionsIndividualsRep1);



clear kappaCDF                                                              % Clear CDF to avoid saving over and over.

end





% Function to save tables of standard form, 
% (Standard form: nSavepoints x (numClasses + 1). Column 1 has data
% relating to the whole population, column 1 + n has data relating to class
% n.)

end

function tableSaver(arrayToSave, varName, populationStructure, fileTail, savePath, numClasses)
    % Replace NaN's with 0 (makes loading csv's much more straightforward)
    arrayToSave(isnan(arrayToSave)) = 0;
    
    % Add class properties (gamma and kappa) as the first two rows.
    row0 = [0, populationStructure(:,2)'];                                       % Gamma values
    row1 = [0, populationStructure(:,3)'];                                       % Kappa values
    arrayWithMetadata = [row0; row1; arrayToSave];
    
    % Convert to table
    table2Save = array2table(arrayWithMetadata);
    
    % Add the variable names - describe which agents each column refers to
    colnames = cell(numClasses + 1,1);
    colnames(1) = {char("all")};

    for i = 1:numClasses
        page = i + 1;
        colnames(page) = {char(sprintf("class_%d",populationStructure(i, 1)))};
    end
    table2Save.Properties.VariableNames = colnames;
    writetable(table2Save, strcat(savePath, varName, fileTail));
end