%% Code to perform the collective navigation described in "Modelling collective
%% navigation via nonlocal communication" by Johnston and Painter. This is
%% highest level script for the idealised information fields. Note this file
%% requires the look-up table for the concentration parameter, which can found
%% at https://melbourne.figshare.com/articles/dataset/kappaCDFLookupTable_mat/14551614.

%% This version has individual trustworthiness parameters and individual navigation skill
%% (which for now only works with the fixed information field)

clear all
close all


% Debugging
rng(1)

% Loop over slower class trustworthiness parameters.
for slowClassGamma = [1]
slowClassGamma

% Loop over sensing ranges
for sensingRange = [50]
sensingRange

load('kappaCDFLookupTable.mat');                                % Load the lookup table for estimating the vM concentration parameter.

% #########################################################################
% Start of user settings
% #########################################################################


% Run setup
nRepeats = 1;              % Number of realisations of the model.
nSavePoints = 501;         % Number of time points to save model output.`
startDist = 1000;          % Initial distance from the target.
runTime = 1;               % Mean reorientation time.

% Timing settings
tChunkSize = 1000;         % Size of chunks to break data into.  There were a few simulations where we didn't know 
                           % how long they would run for. The code was adapted so that the predefined arrays for saving 
                           % simulation data could be expanded during simulations. 
                           % 
                           % Each time that a duration of tChunkSize (in simulation time) is covered,
                           % nSavePoints-1 rows are added to each of the data saving matrices.


limitRun = true;           % Stop the simulation after after a maximum time? 
                           % (Otherwise it will continue untill all arrived - may take a long time or never finish)
                           
tEnd = 5000;               % Max run time if limitRun == true.


% Path for output csv's. 
savePath = '/Users/boppin/Documents/work/Whales/collective-navigation-2/misc_experiments/waypoints/csvs/absoluteDistModulation/';

backgroundFieldType = 'Fixed';   % Choose type of background field, choice of 'Void', 'Fixed','Random','Void', 'Increasing', 'Decreasing', 'Brownian'.
noiseInfluence = 'Information'; % Choose type of noise influence either 'Information' or 'Range'. All results generated with 'Information' except for F9.


velocity = 1;                   % Default speed of individuals.
alpha = 10/20;                  % Weighting of observations for heading calculation.
beta = 10/20;                   % Weighting of observations for concentration calculation.


goalDistance = 10;              % Distance from goal to be counted as "arrived".
goalLocation = [0,0];           % Location of target.

backgroundStrength = 1;         % Background information level.
alignDistance = sensingRange;   % Alignment distance (always = sensing range).


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
                               


                                
% -------------------------------------------------------------------------
% Speed modulation settings
modulateSpeeds = "absoluteDistance";    % Reduce speeds of whales closer to the target than the mean of their neighbours positions,
                                        % via the modulatevelocity script.
                                        % Has no effect if sensingRange == 0
                                        % Options: "off" - do not modulate speeds
                                        %          "goalAxis" - modulate speeds based on distance from neighbours in direction of the goal
                                        %          "absoluteDistance" - modulate speeds based on absolute distance from neighbours, 
                                        %                               applies only when agent is in front of neighbour mean position.
                                
                                
if modulateSpeeds == "absoluteDistance"
    warning(char("modulateSpeeds == 'absoluteDistance' currently only works when whales navigate in the negative x direction. If this is not the case, model will run but incorrectly"));
end
                                
maxDist =  100;                 % Maximum distance from mean position (along axis towards goal). Has different function when noisyModulation
                                % is true or false. 
                                % If noisyModulation is false:
                                % Past this distance, the whales stop.
                                % If noisyModulation is true:
                                    % Mean of normal distribution in logitnormal will be -d/maxDist
                                    % where d is displacement along direction to goal of a given whale 
                                    % from its' neighbours' mean. 
                                
neighboursToConsider = "slow";   % Either "slow" or "all". Whether to consider position of only slower class neighbours, or all neighbours
                                 % in the velocity modulation. Does nothing if modulateSpeeds is false
                                
%% FOR THE CURRENT VECTORISED VERSION OF THE CODE, NOISY MODULATION HAS NOT BEEN IMPLEMENTED AND MUST BE SET TO OFF   

noisyModulation = "off";        % Use probabalistic method to modulate velocities (does nothing if modulateSpeeds is false):
                                   % Options:
                                        %off ? don't use noise in velocity modulation step.
                                        %simple ? linear modulation, then add normal noise and clip to [0,1]
                                        %logitnormal_1side ?  Sample scaling factors from a logitnormal distribution.
                                                            % For "1side" option, only do this when distance >0. This 
                                                            % option allows for a nonzero shift in the normal mean
                                                            % to counter sudden velocity reductions.
                                                            % When distance < 0, this option sets the scaling 
                                                            % factor to 1.
                                        %logitnormal_2side ? % For "2side" option, sample scaling factors regardless of 
                                                             % whether distance < 0 or not. 
                                                             % For this option, I've forced it to use a 0 shift on the mean,
                                                             % in spite of any provided value of normalMeanShift,
                                                             % though this could be changed. 
                                

noiseNormalSD = 0.5;            % Controls width of noise when modulating velocities. Does nothing if modulateSpeeds or noisyModulation are false

normalMeanShift = 0;          % Sorry for adding so many parameters... It's getting a bit messy.
                              % This shifts the normal distributions mean so that we don't get sudden decays 
                              % in class 2 whale's velocities as soon as they pull in front. 

% -------------------------------------------------------------------------

    
navigationFieldGoal = @(x,y) atan2(goalLocation(2)-y,goalLocation(1)-x) ;    % Direction of target.
navigationFieldWaypoint = @(whalex,whaley,waypointx,waypointy) atan2(waypointy - whaley, waypointx - whalex);   % Direction from specific whale to specific waypoint

totalStepCountLoop = 0;         % Number of reorientation events.


% -------------------------------------------------------------------------
% Population structure settings

% Population data held in matrix of form:
% [[class 1 ID, class 1 gamma, class 1 kappa, class 1 n]; 
% [class 2 ID, class 2 gamma, class 2 kappa, class 2 n]]

nIndividualsStart = 100;                            % Total population size

gamma_1 = slowClassGamma;                           % Trustworthiness of class 1
kappa_1 = 0.5;                                      % Navigation skill of class 1.
n_1 = 50;                                           % Number of individuals in class 1.
delta = n_1/nIndividualsStart;                      % Fraction of population in class 1.

gamma_2 = 1;                                        % Trustworthiness of class 2. Always have faster class with trustworthiness 1.
n_2 = nIndividualsStart - n_1;                      % Number of individuals in class 2.
[kappa_2, err] = solveSkill(delta, kappa_1);        % Solve for kappa_2 to mantain average individual velocity towards target.


kappa_2                                             % Print out the error between the effective velocity for the selected
err                                                 % parameters and the effective velocity of the uniform population with
                                                    % kappa = 1.

                                
populationStructure = [[1, gamma_1, kappa_1, n_1];  % Matrix to hold population information
                       [2, gamma_2, kappa_2, n_2]];

% Set up vectors to keep track of individual's class, trustworthiness, and
% individual skill during the runs.
classes = [];                        % Keep track of individual class during run
gamma = [];                          % Keep track of individual trustworthiness during run.
individual_kappas = [];              % Keep track of individual navigation skills during run.

for i = 1:size(populationStructure,1)
    classes = [classes; populationStructure(i,1) * ones(populationStructure(i,4),1)];
    gamma = [gamma; populationStructure(i,2) * ones(populationStructure(i,4),1)];
    individual_kappas = [individual_kappas; populationStructure(i,3) * ones(populationStructure(i,4),1)];
end

numClasses = size(populationStructure, 1);  % Used for saving class specific data.

min_kappa = min(individual_kappas);

individualIDs = [1:nIndividualsStart]';     % Keep track of individual ID's during run. Needed for the
                                            % arrival time matrix.

                                            
% -------------------------------------------------------------------------
% Waypoint settings

% For now, place waypoints in straight line at equal distances between goal and start position

proceedDistance = 20;                              % At what distance from a given waypoint should a whale change
                                                    % target to the next waypoint. Likely to be an important parameter.
                                                                                                        
nWaypoints = 10;                                    % Number of waypoints including final goal
startLocation = [startDist, 0];                     % CAREFUL: changing this has no impact on the actual start location. To fix...

wayPointDist = norm((goalLocation - startLocation)/nWaypoints); % Distance between successive waypoints

wayPoints = [1:nWaypoints]' * (goalLocation - startLocation)/nWaypoints + startLocation;    % Waypoint positions on successive rows



% Currently unused settings
cbar = [linspace(40,115,20)',linspace(36,213,20)',linspace(108,236,20)']/255; %Define colormap.
domainWidth = 400;              % Width of the domain. SW: Not used
domainHeight = 300;             % Height of the domain.    SW: Not used
noiseWavelength = 6;            % Frequency of noise structure in the Brownian noise field only.
repulsionDistance = 0;          % Repulsion mechanism (unused).
holeLocation = [125,175];       % Location of information void.
flowField = 0;                  % Flow field (unused).
flowDirection = 0;              % Flow direction (unused).
flowVelocity = 0;               % Flow velocity (unused).

% #########################################################################
% End of user settings
% #########################################################################


%% Set up matrices for saving data

% These will grow as required if the runs take longer.
finalTime = zeros(nRepeats, numClasses + 1);                                    % Time for all individuals to arrive at the goal.
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

classSpecificNeighbours = NaN(numClasses, nSavePoints, nRepeats, numClasses + 1);
                            % (sensing class, tSaveCount, repeat, sensedclass + 1 )
                            
                            % Number of neighbours of each class, for each class. Dim 1 = "whose
                            % neighbours". I.e. Dim 1 = 1 gives the neighbours for class 1, dim 1 = 2
                            % gives the neighbours for class 2. 

                            % Dim 4 selects the class of the neighbours counted. Dim 4 = 1 means it's
                            % counting the number of neighbours of any class, dim 4 = 2 means it counts
                            % the number of neighbours of class 1, dim 4 = 3 means it counts the number
                            % of neighbours of class 2 etc. 



arrivalTimes = NaN(nIndividualsStart, 3 + nRepeats);                 % Arrival time of each individual for each repeat + class info.
                                                                     % To be used for creating arrival time histograms.
arrivalTimes(:, 1) = classes;                                        % Metadata for the arrival time histograms.                                 
arrivalTimes(:, 2) = gamma;
arrivalTimes(:, 3) = individual_kappas;


if numClasses == 2 
    lastContact = zeros(1,nRepeats);                                  % Save time at which the two classes first lose contact.
end

% Save histogram for cluster measure for each class at specified timesteps
nHistClustermeasure = 101;          % 100 bins for histogram => 100 edges.
histClustermeasureSnapshots = [1,500,1000,1500,2000,2500,3000,3500,4000,4500,5000]; %Timesteps to take snapshots at. 
                                                                                    % Note these refer to the variable tSaveCount, 
                                                                                    % not the actual time, as the
                                                                                    % time iterates with random jumps.
clustermeasureHist = zeros(length(histClustermeasureSnapshots), nHistClustermeasure - 1, numClasses);
                                                                                    % Dim 1: Save 11 snapshots during the run
                                                                                    % Dim 3: Separate histogram for each class




%% Main body of simulation, loops over number of realisations.

tMax = tChunkSize;          % Keep track of max number of chunks for data saving. Update during each repeat.
tSave = linspace(0,tChunkSize,nSavePoints);    % Time points where the data will be saved. Updates during runs.


for iRepeat = 1:nRepeats
    
    % INITIALISATION
    % ---------------------------------------------------------------------
    majorityCheck = zeros(1 + numClasses);                                  % Check if 90% of population has arrived at the target.
    iRepeat                                                                 % Print the realisation.

    nIndividuals = nIndividualsStart;                                       % Number of indivuduals in the simulation.

    t = 0;                                                                  % Initialise time counter.

    defineBackgroundFields;                                                 % Define noise and background fields.
    
    initialPosition = zeros(nIndividuals,2);                                % Initial location of individuals.
    initialPosition(:,2) = -20+40*rand(nIndividuals,1);                     % Initial position (y) of individuals.
    initialPosition(:,1) = startDist - 20 + 40*rand(nIndividuals,1);        % Initial position (x) of individuals.     
    position = initialPosition;                                             % Position of individuals.                             
    pairDistanceVec = pdist(position);                                      % Calculate distances between all pairs of individuals.
    pairDistances = squareform(pairDistanceVec);                            % Pair distance matrix
    
    turningTime = exprnd(runTime,nIndividuals,1);                           % Calculate durations of run events.
    timeToUpdate = turningTime;                                             % Calculate time until reorientation events.
    
    heading = zeros(nIndividuals,1);                                        % Headings of individuals.
    
    concentrationIndividual = zeros(nIndividuals,1);                        % Concentration parameter of individuals. 
    
    runGamma = gamma;                                                       % Copy of individual weightings from which agents can be dropped.
    runKappa = individual_kappas;                                           % Navigation skill of individuals.
    runClass = classes;                                                     % Class (for each gamma and kappa pair) for each individual.
    runIDs = individualIDs;                                                 % ID for each individual.
    
    runTargets = ones(nIndividuals, 1);                                     % Index of waypoint that each individual is currently targeting.
    
    arrivedPosition = [];                                                   % Positions of agents which have reached target.
    arrivedGamma = [];                                                      % Weightings of individuals which have arrived.
    arrivedKappa = [];                                                      % Skill of individuals which have arrived.
    arrivedClass = [];                                                      % Class of individuals which have arrived.
    arrivedIDs = [];                                                        % IDs of individuals which have arrived.
    
    contactCheck = 1;                                                       % Keep track of whether the two classes have lost contact yet
    
                                                                            % Sample initial headings based on individual navigation skill
    for i = 1:nIndividuals
        heading(i) = circ_vmrnd(navigationFieldWaypoint(position(i,1),position(i,2), wayPoints(runTargets(i), 1),wayPoints(runTargets(i),2) ), ...
            individual_kappas(i),1);
    end
%     navigationFieldWaypoint = @(whalex,whaley,waypointx,waypointy)
    
    % MAIN SIMULATION LOOP:
    % run until end of simulation or all individuals have arrived at the target.
    % ---------------------------------------------------------------------
    tSaveCount = 1;                                                         % Count of time points saved.
    totalStepCount = 0;                                                     % Number of steps taken.
    while nIndividuals > 0 &&  (t < tEnd || limitRun == false)
        
        totalStepCount = totalStepCount + 1;                                % Keep track of total steps taken.
        totalStepCountLoop = totalStepCountLoop + 1;                        % Keep track of overall total steps (i.e. over repeats)
        
%         SW: Don't know what this is doing, seem to run into errors on
%         long runs here as the variables aren't defined
%         % If sufficiently many steps taken, add extra preallocated vectors.
%         if mod(totalStepCountLoop,1e6) == 0
%             reorientation = [reorientation;zeros(1e6,1)];
%             navError = [navError;zeros(1e6,1)];
%             distToGoal = [distToGoal;zeros(1e6,1)];
%             neighboursOut = [neighboursOut;zeros(1e6,1)];
%         end

        [nextUpdate,nextAgent] = min(timeToUpdate);                         % Calculate next reorientation event time and agent.
        timeToUpdate = timeToUpdate - nextUpdate;                           % Update time to update for all individuals.
        timeElapsed = nextUpdate;                                           % Calculate time step length.
        t = t+nextUpdate;                                                   % Update time.
       
       % POSITION UPDATE:  Update the position of all individuals. Flow field is not used.
       %                   Modulate the speeds of the better navigators if required.
       % -----------------------------------------------------------------
        if modulateSpeeds ~= "off" && sensingRange > 0
            
            % Unfortunately I had to vectorize the velocity modulation calculation
            % to make it faster to run. It makes it harder to read though.
            
            newSpeeds = velocity * ones(nIndividuals, 1);                       % Column vector of each agent's speeds.
                                                                                % Modulate velocity of each faster agent based on position
                                                                                % relative to slow class neighbours.
            indexList = 1:nIndividuals;
            
            fasterAgents = indexList(runKappa == max(individual_kappas));       % Faster agents are those with higher skill (kappa)
            
            if numel(fasterAgents) > 0
                sensingRangeMat = ones(numel(fasterAgents), nIndividuals);       % Matrix where ith row is the sensing range for the 
                                                                                 % ith fasterAgent. For our simulations this will be 
                                                                                 % constant everywhere, but code is set up to allow for 
                                                                                 % spatially varying sensign ranges.
                for i = 1:numel(fasterAgents)
                    sensingRangeMat(i,:) = sensingRangeField(position(fasterAgents(i),1),position(fasterAgents(i),2));
                end


                fastAgentPairDists = pairDistances(fasterAgents, :);            % ith row is distances of each whale from the ith fasterAgent

                slowclassMat = repmat(runKappa' == min(individual_kappas), numel(fasterAgents),1 );

                xPosMat = repmat(position(:,1)', numel(fasterAgents),1);
                yPosMat = repmat(position(:,2)', numel(fasterAgents),1);


                % Set up a matrix to specify which whales are neighbours of each of the fasterAgents.
                if neighboursToConsider == "all"
                    neighbourMatrix = (fastAgentPairDists < sensingRangeMat) & (fastAgentPairDists > 0);
                elseif neighboursToConsider == "slow"
                    neighbourMatrix = (fastAgentPairDists < sensingRangeMat) & (fastAgentPairDists > 0) & slowclassMat;
                end
                 



                neighbourxPosMat = xPosMat.*neighbourMatrix;
                neighbourxPosMat(neighbourxPosMat == 0) = NaN;
                neighbouryPosMat = yPosMat.*neighbourMatrix;
                neighbouryPosMat(neighbouryPosMat == 0) = NaN;

                neighbourMeanX = nanmean(neighbourxPosMat,2);
                neighbourMeanY = nanmean(neighbouryPosMat,2); % Row j contains mean position of the jth faster agent's neighbours. 

                % What happens to natural 0's though? hmmmm need to think.

                neighbourMeanXY = [neighbourMeanX, neighbourMeanY];
                
                % NaN's in neighbourMeanXY occur when a given faster agent has no neighbours. 
                % This is handled in vectorModulateVel, where the agent's newSpeed is set to 1. 


                fasterAgentPositions = position(fasterAgents, :);

                newSpeeds(fasterAgents) = vectorModulatevel(fasterAgentPositions, neighbourMeanXY, goalLocation, velocity, maxDist, modulateSpeeds);
            end
            
            newVelocity = newSpeeds.*[cos(heading),sin(heading)];           % Velocity of each agent after modulation.
                                                                            % Keep this, as we need it for the velocity saving.
            
            position = position + timeElapsed*newVelocity + flowField*flowVelocity*[cos(flowDirection),sin(flowDirection)]; %newVelocity incorporates heading
     
        else
            position = position + velocity*timeElapsed*[cos(heading),sin(heading)] + flowField*flowVelocity*[cos(flowDirection),sin(flowDirection)];
        end
        
        
        
        
        pairDistanceUpdate;                                                 % Update pair distances for all pairs of individuals.
        pairDistances(1:nIndividuals+1:end) = 1e10;                         % Avoid influence of pairs of identical individuals.
        
        % HEADING UPDATE: Update heading of agent currently reorienting
        % ---------------------------------------------------------------------
        
        % Update target of nextAgent if necessary. Targets only used during reorientation, so only need to update the target for nextAgent.
        
        %agentWaypointDist = norm(position(nextAgent,:) - wayPoints(runTargets(nextAgent),:));
        agentWaypointxDist = abs(position(nextAgent,1) - wayPoints(runTargets(nextAgent),1));
        if agentWaypointxDist < proceedDistance && runTargets(nextAgent) < nWaypoints
            runTargets(nextAgent) = runTargets(nextAgent) + 1;
        end
        
        neighbours = find(pairDistances(nextAgent,:)>0&pairDistances(nextAgent,:)<sensingRangeField(position(nextAgent,1),position(nextAgent,2)));
        nNeighbours = numel(neighbours);                                    % Number of individuals within perceptual range.
        [minDistance,closestAgent] = min(pairDistances(nextAgent,:));       % Find closest agent.
        oldHeading = heading;                                               % Retain previous heading.
        
        
                                                                            % Find arrived individuals within the perceptual range.
        arrivedDistances = [];
        arrivedNeighbours = [];
        
        if cooperative == "target" && ~isempty(arrivedPosition)             % All arrived agents treated as being precisely at target.
            agentDistToTarget = sqrt((position(nextAgent,1) - goalLocation(1))^2 + (position(nextAgent,2) - goalLocation(2))^2);
            if agentDistToTarget < sensingRange
                arrivedNeighbours = 1:length(arrivedIDs);                   % If agent within sensing range of target, all arrived agents are neighbours
                minDistance = min([minDistance, agentDistToTarget]);        % Update min distance if close to goal. minDistance controls if cooperative navigation is used
                                                                            % Not including this line was the reason for the single agent remaining oscilation bug.
           end
        end        
                                                                            
        
        nArrivedNeighbours = numel(arrivedNeighbours);                      % If cooperative == "off", the arrivedNeighbours is left as empty.
        
        nNeighbours = nNeighbours + nArrivedNeighbours;
        
                                                                            % Calculate sample heading based on inherent information/individual
                                                                            % skill only.
        potentialHeading = circ_vmrnd(navigationFieldWaypoint(position(nextAgent,1),position(nextAgent,2), wayPoints(runTargets(nextAgent), 1),wayPoints(runTargets(nextAgent),2) ),...
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

                if cooperative == "target"                                  % Cooperative navigation where arrived whales are viewed 
                                                                            % as being precisely at target.
                    
                    if nArrivedNeighbours > 0
                        angleToTarget = atan2(goalLocation(2) - position(nextAgent,2), goalLocation(1) - position(nextAgent,1));
                        arrivedHeadings = angleToTarget*ones(nArrivedNeighbours, 1);
                        
                    else                                                    % No agents arrived, or nextAgent outside of target range
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
                
                    
                    
                else                                                        % Non-cooperative. Arrived agents are ignored.            
                    weightedNeighbourHeading = circ_mean(heading(neighbours), runGamma(neighbours));
                    bestGuessHeading = circ_mean([weightedNeighbourHeading;potentialHeading],[1-alpha;alpha]);     % MLE of heading.
                    w = [(1-beta)*runGamma(neighbours); beta*sum(runGamma(neighbours))];                           % Individual weightings for concentration parameter
                    alphaLookup = [heading(neighbours);potentialHeading];                                          % Set of observed headings.
                end
                
                circ_kappa_script;                                                                                  % Calculate estimate of concentration parameter.
                bestGuessStrength = kappa;                                                                          % Estimate of concentration parameter.
                heading(nextAgent) = circ_vmrnd(bestGuessHeading,bestGuessStrength,1);                              % Set new heading.

                concentrationIndividual(nextAgent) = kappa;                  % Store the agent's new concentration parameter
                
                
%             % Attraction mechanism unused.
%             elseif minDistance < attractDistance
%                 heading(nextAgent) = atan2(mean(position(neighbours,2))-position(nextAgent,2),...
%                     mean(position(neighbours,1))-position(nextAgent,1));
             end
        else
            heading(nextAgent) = potentialHeading;
        end
         
        timeToUpdate(nextAgent) = exprnd(runTime,1);                        % New duration of run.
        pairDistances(1:nIndividuals+1:end) = 0;                            % Set pair distances to zeros for identical individuals.
        
        
        
        % MAINTNENCE:  Save data, remove arrived agents, and expand data saving arrays if necessary
        % -----------------------------------------------------------------
        
        if t > tSave(tSaveCount)                                            % Storage of data at specific time points.
            saveData;
        end
        
        
        removal = [];                                                       % Remove arrived agents from simulation.

        for i = 1:nIndividuals
            if sqrt((position(i,1)-goalLocation(1))^2+(position(i,2)-goalLocation(2))^2) < goalDistance
                removal = [removal;i];
            end
        end
        
                                                                            % Store information on the arrived individuals.
                                                                            % Used for cooperative navigation.
                                                                            
        arrivedPosition = [arrivedPosition; position(removal,:)];           % Positions of agents which have reached target.
        arrivedGamma = [arrivedGamma; runGamma(removal)];                   % Weightings of individuals which have arrived.
        arrivedKappa = [arrivedKappa; runKappa(removal)];                   % Skill of individuals which have arrived.
        arrivedClass = [arrivedClass; runClass(removal)];                   % Class of individuals which have arrived.
        arrivedIDs = [arrivedIDs; runIDs(removal)];                         % ID of individuals which have arrived.
        
        
        
        arrivalTimes(runIDs(removal), iRepeat + 3) = t;                     % Save the arrival time of the agents arriving at target. 
        
                                                                            % Remove arived agents from active navigation
        
        position(removal,:) = [];                                           % Remove individuals from position.
        heading(removal) = [];                                              % Remove individuals from heading.
        timeToUpdate(removal) = [];                                         % Remove individuals from reorientation.
        concentrationIndividual(removal) = [];                              % Remove individuals from concentration.
        runGamma(removal) = [];                                             % Remove individuals from weightings.
        runClass(removal) = [];                                             % Remove individuals from classes.
        runKappa(removal) = [];                                             % Remove individuals from navigation skills.
        runTargets(removal) = [];                                           % Remove targest from list
        runIDs(removal) = [];
        pairDistances(removal, :) = [];                                     % Remove individuals form pair distance matrix
        pairDistances(:, removal) = [];                                     %   - We need to do this in order to calculate neighbours
                                                                            %     for the velocity modulation at the start of the next 
                                                                            %     timestep.
        nIndividuals = nIndividuals - numel(removal);                       % Number of individuals remaining.
        
        
        % Add space to data saving matrices if 
        % t exceeds prealocated size.
        if t >= tMax                                                       
                                                                           
            t
            tSave = [tSave, linspace(tMax + 2,tMax + tChunkSize,nSavePoints -1)];   % Sort of set up for the nSavePoints = 1000, tSave = 501 case...
            
                                                                            
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
            

            
            classSpecificNeighbours = cat(2, classSpecificNeighbours,...    % Concatenate along time dimension
                NaN(numClasses, nSavePoints - 1, nRepeats, numClasses + 1));
            
            tMax = tMax + tChunkSize;                                       % Time corresponding to end of new data saving matrices.
            
            
        end
        
    end
    
    finalTime(iRepeat) = t;                                                 % Final time in the simulation.
    
end

%% Save outputs - calculate means over the realisations for each variable

                                                                            % Non segmented variables: i.e. data not separated for each class
clusterMeasure = squeeze(mean(clusterMeasure,2));                           % Mean of clustering across realisation loop.

                                                                            % Segmented variables, i.e. we have data separated for each class.
                                                                            % First page = all agents, page 2 = class 1, ...
                                                                            
xPositionMean = squeeze(mean(xPosition,2));                                 % Mean of average x position across realisation loop.
distanceToGoal = squeeze(mean(distanceToGoal,2));                           % Mean of average distance to goal across realisation loop.
meanNeighbours = squeeze(mean(meanNeighbours,2));                           % Mean of average number of neighbours across realisation loop.
meanDifferenceDirection = squeeze(mean(meanDifferenceDirection,2));         % Mean of difference between heading and target across realisation loop.
nIndividualsRemaining = squeeze(mean(nIndividualsRemaining,2));             % Mean of number individuals remaining across realisation loop.
concentrationMean = squeeze(mean(concentrationParameters, 2));              % Mean of the concentration parameters over realisation loop.
majorityGoneMean = mean(majorityGone, 1);                                   % Mean time for majority to reach target.
distanceToGoalAll = squeeze(mean(distanceToGoalAll, 2));                    % Mean distance to goal of all whales, inc. arrived.
meanNeighboursIncArrived = squeeze(mean(meanNeighboursIncArrived, 2));      % Mean number of neighbours, inc. arrived.
effectiveVelocity = squeeze(mean(effectiveVelocity, 2, 'omitnan'));         % Mean effective velocity in target direction.



fileTailStart = sprintf('_distance_%d_range_%d_nwaypoints_%d_proceeddist_%.2f', startDist, sensingRange, nWaypoints, proceedDistance);  % Save each variable to a csv. Keep track of run and 
                                                                            % population metadata in the filename.    
if modulateSpeeds ~= "off"
    fileTailStart = strcat(fileTailStart, sprintf('modulate_%s_m%.2f',modulateSpeeds, maxDist));
    if noisyModulation ~= "off"
        shiftsize = 0;
        if noisyModulation == "logitnormal_1side"
            shiftsize = normalMeanShift;
        end
        fileTailStart = strcat(fileTailStart, sprintf('_noise_%s_SD%.2f_shift%.2f', noisyModulation, noiseNormalSD, shiftsize));
    end
else
    fileTailStart = strcat(fileTailStart, 'modulatefalse');
end
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



classSpecificNeighbours = squeeze(mean(classSpecificNeighbours, 3, 'omitnan'));  % Save the class specific neighbours.
                                                                                 % Average over the repeats. dim 3 is the dimension for the repeats. 
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


% Save the clustermeasure (i.e. pairdistance) histograms for each class
for page = 1:numClasses
                                            % Only saving histogram for each class, none for whole population
    class = populationStructure(page, 1);   % In case there's only 1 class, which we are naming class 2
    
    clustermeasureHistogramCurrentClass = clustermeasureHist(:,:, page);
    % It would be good to keep track of the times for each of the histogram
    % snapshots.
    snapshotTimes = 2*histClustermeasureSnapshots';                         % Keep track of the time associated with each snapshot.
    histogramForSaving = [snapshotTimes, clustermeasureHistogramCurrentClass];

    % It would also be good to keep track of the bin edges for the
    % histogram. This is a bit messy. Save bin edges as first row...

    binEdges = linspace(0, 1000, nHistClustermeasure);
    histogramForSaving = [binEdges; histogramForSaving];
    
    csvwrite(strcat(savePath, "pairdistanceHistogramClass" + class, fileTail), histogramForSaving);

end

% Save trajectories for repeat 1.
% Add metadata as first two rows.
posRow1 = individualIDs';
posRow2 = classes';
xPositionsIndividualsRep1 = [posRow1; posRow2; xPositionsIndividualsRep1];
yPositionsIndividualsRep1 = [posRow1; posRow2; yPositionsIndividualsRep1];
csvwrite(strcat(savePath, 'xPositionsIndividuals', fileTail), xPositionsIndividualsRep1);
csvwrite(strcat(savePath, 'yPositionsIndividuals', fileTail), yPositionsIndividualsRep1);


% Save the array of loss of contact times
if numClasses == 2
    csvwrite(strcat(savePath, 'lossOfContactTime', fileTail), lastContact)
end




% Save the data
fileTail = sprintf('_range_%d.csv', sensingRange);                          % SW: Keep track of range parameter for saved data
savePath = '/Users/boppin/Documents/work/Whales/collective-navigation-2/misc/sanity_check/originalcode/';
csvwrite(strcat(savePath, 'xPosition', fileTail), xPositionMean);                     % SW: Save the above matrices for combined plots
csvwrite(strcat(savePath, 'clusterMeasure', fileTail), clusterMeasure);
csvwrite(strcat(savePath, 'distanceToGoal', fileTail), distanceToGoal);
csvwrite(strcat(savePath, 'meanNeighbours', fileTail), meanNeighbours);
csvwrite(strcat(savePath, 'meanDifferenceDirection', fileTail), meanDifferenceDirection);
csvwrite(strcat(savePath, 'nIndividualsRemaining', fileTail), nIndividualsRemaining);
csvwrite(strcat(savePath, 'direction_Histogram', fileTail), directionHist);



clear kappaCDF                                                              % Clear CDF to avoid saving over and over.

end






end




function tableSaver(arrayToSave, varName, populationStructure, fileTail, savePath, numClasses)
    % Function to save tables of standard form, 
    % (Standard form: nSavepoints x (numClasses + 1). Column 1 has data
    % relating to the whole population, column 1 + n has data relating to class n.)

    
    arrayToSave(isnan(arrayToSave)) = 0;                                   % Replace NaN's with 0 
                                                                           % (makes loading csv's much more straightforward)
   
                                                                           % Add class properties (gamma and kappa) as the first two rows.
    row0 = [0, populationStructure(:,2)'];                                 % Gamma values
    row1 = [0, populationStructure(:,3)'];                                 % Kappa values
    arrayWithMetadata = [row0; row1; arrayToSave];
    
    table2Save = array2table(arrayWithMetadata);                           % Convert to table
    
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



