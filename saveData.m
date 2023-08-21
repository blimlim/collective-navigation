%% Code for Johnston and Painter to save relevant data at specific time points.
%% Called by Homing_Script.m

% Save global data, i.e. averages over all individuals.
% Recall: % Page 1 = All individuals, Page 2 = class 1, ..., Page N+1 = class N
xPosition(tSaveCount,iRepeat, 1) = mean(position(:,1));                        % Mean position (x) of the population.
yPosition(tSaveCount,iRepeat, 1) = mean(position(:,2));                        % Mean position (y) of the population.

distanceToGoal(tSaveCount,iRepeat, 1) = sqrt((xPosition(tSaveCount,iRepeat, 1)-goalLocation(1))^2+(yPosition(tSaveCount,iRepeat, 1)-goalLocation(2))^2);     % Mean distance of the population to the goal.


% Distance to goal including those arrived at target:
allPosition = [position; arrivedPosition];
xPositionAll(tSaveCount, iRepeat, 1) = mean(allPosition(:,1));
yPositionAll(tSaveCount, iRepeat, 1) = mean(allPosition(:,2));
distanceToGoalAll(tSaveCount, iRepeat, 1) = sqrt((xPositionAll(tSaveCount,iRepeat, 1)-goalLocation(1))^2+(yPositionAll(tSaveCount,iRepeat, 1)-goalLocation(2))^2);    

clusterMeasure(tSaveCount,iRepeat, 1) = 2*sum(pairDistanceVec)/max((nIndividuals*(nIndividuals-1)),1); % Measure of population clustering. 
                                                                                                    % SW: Added in factor of 2 to make it average distance per pairing
nNeighbours = zeros(nIndividuals,numClasses + 1);                                % Current number of observed neighbours. (i+1)th column = # neighbours of class i
                                                                                 % First col is total number of neighbours

nNeighboursIncArrived = zeros(nIndividuals, numClasses + 1);                                % Number of observed neighbours, including those at target
diffDirection = zeros(nIndividuals,1);                                      % Difference in direction between heading and target.

concentrationParameters(tSaveCount,iRepeat, 1) = mean(concentrationIndividual); % Average concentration parameter at last reorientation.


% Average velocity in direction of target
heading;
anglesToTarget = navigationField(position(:,1), position(:,2));             % Angles from each whale to the target.
vectorsToTarget = [cos(anglesToTarget), sin(anglesToTarget)];               % Unit vectors towards target from whale locations.
velocityVectors = velocity*[cos(heading), sin(heading)];                    % Velocity vectors for individuals.
velocityToTarget = dot(velocityVectors, vectorsToTarget, 2);                % Component of velocity in direction of target

effectiveVelocity(tSaveCount, iRepeat, 1) = mean(velocityToTarget);         % Save mean velocity towards target

% Velocity histogram 
velocityHist(tSaveCount, :, 1) = velocityHist(tSaveCount, :, 1) + histcounts(velocityToTarget, linspace(-1, 1, nHistVelocity));


% Loop over individuals to calculate the number of neighbours, and the
% differences in heading from the target.
for i = 1:nIndividuals
    
    % Neighbours
    neighbours = find(pairDistances(i,:)>0&pairDistances(i,:)<sensingRangeField(position(i,1),position(i,2)));
    nNeighbours(i,1) = numel(neighbours);                                     % Number of observed neighbours.
    for classIdx = 1:numClasses
        neighboursInClass = find(runClass(neighbours) == classIdx);          % which neighbours are of class classIdx
        nNeighbours(i, classIdx + 1)  = numel(neighboursInClass);
    end
    
    
   
    % If cooperative == "off", only count actively navigating neighbours,
    % as only they are used during reorientation.
    nNeighboursIncArrived(i,:) = nNeighbours(i,:); 
    
    % If cooperative is not off, count arrived whales within range as
    % neighbours, as their information is used for calculating new headings
  
    if cooperative == "target" && ~(isempty(arrivedPosition))
        if sqrt((position(i,1) - goalLocation(1))^2 + (position(i,2) - goalLocation(2))^2) < sensingRange  % If whale i in range of goal, all
                                                                                                           % arrived are neighbours
            nNeighboursIncArrived(i,1) = numel(arrivedIDs) + nNeighbours(i,1);  % Total number of neighbours for whale i.
            % Arrived neighbours of each class
            for classIdx = 1:numClasses
                arrivedNeighboursInClass = find(arrivedClass == classIdx);          % which arrived neighbours are of class classIdx
                nNeighboursIncArrived(i, classIdx + 1) = nNeighbours(i, classIdx + 1) + numel(arrivedNeighboursInClass);
            end
            
            
        end
    end
    
    
    % Direction between heading and target.
    diffDirection(i) = abs(heading(i)+pi - mod(navigationField(position(i,1),position(i,2))+pi,2*pi));
    if diffDirection(i) > pi
        diffDirection(i) = pi - mod(diffDirection(i),pi);
    end
end

meanNeighbours(tSaveCount,iRepeat, 1) = mean(nNeighbours(:,1));                     % Average number of actively navigating neighbours across all individuals
meanNeighboursIncArrived(tSaveCount, iRepeat, 1) = mean(nNeighboursIncArrived(:,1)); % Average number of neighbours including arrived, across all individuals

for sensingClass = 1:numClasses
    classSpecificNeighbours(sensingClass, tSaveCount, iRepeat, 1) =  mean(nNeighboursIncArrived(find(runClass == sensingClass), 1)); % average total neighbours for individuals in class sensingClass.
    for sensedClass = 1:numClasses
        classSpecificNeighbours(sensingClass, tSaveCount, iRepeat, sensedClass + 1) = mean(nNeighboursIncArrived(find(runClass == sensingClass), sensedClass+1));     % average number of neighbours of class sensedClass, for individuals in class sensingClass
    end
end


meanDifferenceDirection(tSaveCount,iRepeat, 1) = mean(diffDirection);          % Average difference between heading and target.
nIndividualsRemaining(tSaveCount,iRepeat, 1) = nIndividuals;                   % Number of individuals yet to arrive at target.
directionHist(:,1) = directionHist(:,1) + histcounts(mod(heading,2*pi),linspace(0,2*pi,nHistDirection))'; % Generate histogram of headings.






% Check if 90% of individals have arrived at target and if so, store time.
if nIndividuals/nIndividualsStart <= 0.1 && majorityCheck(1) == 0              
    majorityGone(iRepeat, 1) = t;
    majorityCheck(1) = 1;
end

% Save data for each class, e.g. averages calculated for only trustworthy
% individuals
% Recall: % Page 1 = All individuals, Page 2 = class 1, ..., Page N+1 = class N

for idx = 1:numClasses
    page = idx + 1;
    classIdx = populationStructure(idx, 1);         % Used in case there is only 1 class navigating which we want named 'class 2'
    
    agentsInClass = find(runClass == classIdx);                           % Indices of remaining agents within the current class.
    numInClass = numel(agentsInClass);
    
    
    % Check if 90% of individals in the current class have arrived at target and if so, store time.
    if numInClass/populationStructure(idx, 4) <= 0.1 && majorityCheck(page) == 0              
        majorityGone(iRepeat, page) = t;
        majorityCheck(page) = 1;
    end
    
    % If numInClass == 0, don't try to update data, it will just give NaNs.
    % Not updating the corresponding statistics for the 
    % given timestep as 0, which will bleed into the average over the
    % repeats. Keep this in mind, so probably don't trust the data towards
    % the end of the run anyway.
    
    arrivedAgentsInClass = find(arrivedClass == classIdx);
   
    
    if numInClass ~= 0                                                 % Only do calculations if there are agents in class still navigating
        classPositions = position(agentsInClass,:);
        classXPosition = mean(classPositions(:,1));
        classYPosition = mean(classPositions(:,2));
        classDistanceToGoal = sqrt((classXPosition-goalLocation(1))^2+(classYPosition-goalLocation(2))^2);     % Mean distance of the population to the goal.
        classVelocityToTarget = mean(velocityToTarget(agentsInClass));
        
        classPositionsAll = [position(agentsInClass,:); arrivedPosition(arrivedAgentsInClass,:)];                 % Position including arrived agents.
        classXPositionAll = mean(classPositionsAll(:,1));
        classYPositionAll = mean(classPositionsAll(:,2));
        classDistanceToGoalAll = sqrt((classXPositionAll-goalLocation(1))^2+(classYPositionAll-goalLocation(2))^2);
        
        % The number of neighbours and difference in direction have already
        % been calculated for each individual.
        classnNeighbours = mean(nNeighbours(agentsInClass, 1));
        classDiffDirection = mean(diffDirection(agentsInClass));
        classnNeighboursIncArrived = mean(nNeighboursIncArrived(agentsInClass, 1));
    
        classConcentrationParameters = mean(concentrationIndividual(agentsInClass)); % Average concentration parameter at last reorientation.
        
        

        % Cluster measure for class (average pairdistance within the class)
        classPairDistVec = pdist(classPositions);
        if numInClass == 0
            classPairDistVec = [0];
        end
        classClusterMeasure = 2*sum(classPairDistVec)/max((numInClass*(numInClass-1)),1);   % Average distance between pairs within class. 
        
        % Save the data for the current class and timestep to the overall
        % storage arrays
        distanceToGoal(tSaveCount, iRepeat, page) = classDistanceToGoal;
        meanNeighbours(tSaveCount, iRepeat, page) = classnNeighbours;
        meanDifferenceDirection(tSaveCount, iRepeat, page) = classDiffDirection;
        nIndividualsRemaining(tSaveCount, iRepeat, page) = numInClass;
        concentrationParameters(tSaveCount, iRepeat, page) = classConcentrationParameters;
        directionHist(:,page) = directionHist(:,page) + histcounts(mod(heading(agentsInClass),2*pi),linspace(0,2*pi,nHistDirection))'; % Generate histogram of headings.
        velocityHist(tSaveCount, :, page) = velocityHist(tSaveCount, :, page) + histcounts(velocityToTarget(agentsInClass), linspace(-1, 1, nHistVelocity)); % Velocity histogram 
        distanceToGoalAll(tSaveCount, iRepeat, page) = classDistanceToGoalAll;
        meanNeighboursIncArrived(tSaveCount, iRepeat, page) = classnNeighboursIncArrived;
        clusterMeasure(tSaveCount, iRepeat, page) = classClusterMeasure;
        effectiveVelocity(tSaveCount, iRepeat, page) = classVelocityToTarget;
        
        
    end
    
    
    
end



% Save individual positions for repeat 1
if iRepeat == 1
    remainingIndividuals = runIDs;
    arrivedIndividuals = arrivedIDs;
    xPositionsIndividualsRep1(tSaveCount, runIDs) = position(:,1)';
    yPositionsIndividualsRep1(tSaveCount, runIDs) = position(:,2)';
    if ~isempty(arrivedIDs)
        xPositionsIndividualsRep1(tSaveCount, arrivedIDs) = arrivedPosition(:,1)';
        yPositionsIndividualsRep1(tSaveCount, arrivedIDs) = arrivedPosition(:,2)';
    end
end


tSaveCount = tSaveCount + 1;                                                % Increase counter of number of saved data points.
