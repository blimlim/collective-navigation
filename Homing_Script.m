%% Code to perform the collective navigation described in "Modelling collective
%% navigation via nonlocal communication" by Johnston and Painter. This is
%% highest level script for the idealised information fields. Note this file
%% requires the look-up table for the concentration parameter, which can found
%% at https://melbourne.figshare.com/articles/dataset/kappaCDFLookupTable_mat/14551614.

%% This version has individual trustworthiness parameters and individual navigation skill
%% (which for now only works with the fixe information field)

clear all
close all

nRepeats = 10;                                                  % Number of realisations of the model.
%nSavePoints = 501;                                              % Number of time points to save model output.
% Double for edited concentration mechanism
nSavePoints = 501;
load('kappaCDFLookupTable.mat');                                % Load the lookup table for estimating the vM concentration parameter.

% Path for output csv's. 
savePath = '../skill_classes/g1k0.1n55_g1k7.5227n45/';

finalTime = zeros(nRepeats,1);                                  % Time for all individuals to arrive at the goal.
xPosition = zeros(nSavePoints,nRepeats);                        % Mean position (x) of the population.
yPosition = zeros(nSavePoints,nRepeats);                        % Mean position (y) of the population.
clusterMeasure = zeros(nSavePoints,nRepeats);                   % Measure of clustering of the population.
meanNeighbours = zeros(nSavePoints,nRepeats);                   % Mean number of neighbours within perceptual range.
distanceToGoal = zeros(nSavePoints,nRepeats);                   % Mean distance to goal of the population.
meanDifferenceDirection = zeros(nSavePoints,nRepeats);          % Mean error in heading relative to target.
nIndividualsRemaining = zeros(nSavePoints,nRepeats);            % Number of individuals remaining in the simulation (i.e. yet to arrive at goal).
majorityGone = zeros(nRepeats,1);                               % Time for 90% of the individuals to arrive at the goal.

concentrationParameters = zeros(nSavePoints, nRepeats);         % Store the average concentration parameter at each step
                                                                
                                                               

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

sensingRange = 500;             % Perceptual range of individuals.
backgroundStrength = 1;         % Background information level.
repulsionDistance = 0;          % Repulsion mechanism (unused).
alignDistance = sensingRange;   % Alignment distance (always = sensing range).
attractDistance = sensingRange; % Attraction mechanism (unused).

goalDistance = 10;              % Distance from goal to be counted as "arrived".
noiseWavelength = 6;            % Frequency of noise structure in the Brownian noise field only.
    
goalLocation = [0,0];           % Location of target.
holeLocation = [125,175];        % Location of information void.
    
navigationField = @(x,y) atan2(goalLocation(2)-y,goalLocation(1)-x) ;       % Direction of target.


% Trustworthyness parameters for each individual

% Start with two classes, trustworthy (t) and untrustworthy (u).
% Set gamma_t = 1, n_t = 100-n_u

% We then have two parameters gamma_u in [0,1] and n_u in [0,nIndividualsStart]



%% Initially: Uniform trustworthyness, two classes of navigation skill


% New population structure
% [[class 1 ID, class 1 gamma, class 1 kappa, class 1 n]; 
% [class 2 ID, class 2 gamma, class 2 kappa, class 2 n]]

gamma_1 = 1; % Don't change this 

kappa_1 = 0.1; % Modified inherent navigation skill

nIndividualsStart = 100;
n_1 = 55;       % Number of individuals in class 1

delta = n_1/nIndividualsStart; % Class 1 population fraction

% Solve for class 2 parameters
gamma_2 = 1; % Uniform trustworthiness
n_2 = nIndividualsStart - n_1;


[kappa_2, err] = solveSkill(delta, kappa_1); % Solve for kappa_2 which maintains average individual velocity towards target
err
populationStructure = [[1, gamma_1, kappa_1, n_1]; [2, gamma_2, kappa_2, n_2]];


% Set up vectors to keep track of individual's class, trustworthyness, and
% individual skill during the runs

classes = [];                                                               % Class.
gamma = [];                                                                 % Trustworthiness values.
individual_kappas = [];                                                     % Navigation skills.

for i = 1:size(populationStructure,1)
    classes = [classes; populationStructure(i,1) * ones(populationStructure(i,4),1)];
    gamma = [gamma; populationStructure(i,2) * ones(populationStructure(i,4),1)];
    individual_kappas = [individual_kappas; populationStructure(i,3) * ones(populationStructure(i,4),1)];
end

% Check that I haven't done anything dumb here
assert(length(gamma) == nIndividualsStart)


% Add extra dimensions to the data saving arrays to store data for 
% each class of individual
numClasses = size(populationStructure, 1);

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
end







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
    %pairDistances = zeros(nIndividuals);                                
    pairDistanceVec = pdist(position);                                  % Calculate distances between all pairs of individuals.
    pairDistances = squareform(pairDistanceVec);                        % Pair distance matrix
    
    turningTime = exprnd(runTime,nIndividuals,1);                       % Calculate durations of run events.
    timeToUpdate = turningTime;                                         % Calculate time until reorientation events.
    
    heading = zeros(nIndividuals,1);                                    % Headings of individuals.
    
    concentrationIndividual = zeros(nIndividuals,1);                    % Concentration parameter of individuals. 
    
    runGamma = gamma;                                                   % Copy of individual weightings from which agents can be dropped.
    runKappa = individual_kappas;                                       % Navigation skill of individuals.
    runClass = classes;                                                   % Class (gamma and kappa) for each individual.
    
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
        
        % Calculate sample heading based on inherent information/individual
        % skill only.
        potentialHeading = circ_vmrnd(navigationField(position(nextAgent,1),position(nextAgent,2)),...
            individual_kappas(nextAgent),1);
        
        % Update heading based on other observed individuals if number of
        % neighbours exceeds zero.
        if nNeighbours > 0 && minDistance < sensingRangeField(position(nextAgent,1),position(nextAgent,2))
            % Repulsion mechanism unused.
            if minDistance < repulsionDistance
                heading(nextAgent) = atan2(-position(closestAgent,2)+position(nextAgent,2), ...
                    -position(closestAgent,1)+position(nextAgent,1));
            % Alignment mechanism.    
            elseif minDistance < alignDistance                
                % Heading estimatio with trustworthyness weightings applied
                % to neighbours
                weightedNeighbourHeading = circ_mean(heading(neighbours), runGamma(neighbours));
                
                bestGuessHeading = circ_mean([weightedNeighbourHeading;potentialHeading],[1-alpha;alpha]);   % MLE of heading.
                
                % Individual weightings for concentration parameter
                w = [(1-beta)*runGamma(neighbours); beta*sum(runGamma(neighbours))];
                
                % Original weighting for concentration parameter
                %w = [(1-beta)*ones(size(neighbours'));beta*nNeighbours];                                            % Weighting of observed headings.
                
                % Concentration parameter using individual weightings
                alphaLookup = [heading(neighbours);potentialHeading];                                               % Set of observed headings.
                
                %w = [neighbourWeights; runGamma(nextAgent)];
                
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
        
        position(removal,:) = [];                                           % Remove individuals from position.
        heading(removal) = [];                                              % Remove individuals from heading.
        timeToUpdate(removal) = [];                                         % Remove individuals from reorientation.
        concentrationIndividual(removal) = [];                              % Remove individuals from concentration.
        runGamma(removal) = [];                                             % Remove individuals from weightings.
        runClass(removal) = [];                                             % Remove individuals from classes.
        runKappa(removal) = [];                                             % Remove individuals from navigation skills.
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




clear kappaCDF                                                              % Clear CDF to avoid saving over and over.

% Plot relevant results
plotResults;