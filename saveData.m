%% Code for Johnston and Painter to save relevant data at specific time points.
%% Called by Homing_Script.m

% Save global data, i.e. averages over all individuals.
% -------------------------------------------------------------------------
% Recall: % Page 1 = All individuals, Page 2 = class 1, ..., Page N+1 = class N

                                                                                    % Distance to goal of active navigators
xPosition(tSaveCount, iRepeat, 1) = mean(position(:,1));                        
yPosition(tSaveCount, iRepeat, 1) = mean(position(:,2));                        
distanceToGoal(tSaveCount,iRepeat, 1) = sqrt((xPosition(tSaveCount, iRepeat, 1)-goalLocation(1))^2 ...
                                             +(yPosition(tSaveCount, iRepeat, 1)-goalLocation(2))^2);     
                                                                                    % Distance to goal including those already at target
allPosition = [position; arrivedPosition];                                                                                   
xPositionAll(tSaveCount, iRepeat, 1) = mean(allPosition(:,1));
yPositionAll(tSaveCount, iRepeat, 1) = mean(allPosition(:,2));
distanceToGoalAll(tSaveCount, iRepeat, 1) = sqrt((xPositionAll(tSaveCount, iRepeat, 1)-goalLocation(1))^2 ... 
                                                 + (yPositionAll(tSaveCount, iRepeat, 1)-goalLocation(2))^2);    
                                             
                                                                                    % Measure of population clustering: Mean distance per pair. 
                                                                                    %SW: Added in factor of 2 to make it average distance per pairing.
clusterMeasure(tSaveCount, iRepeat, 1) = 2*sum(pairDistanceVec)/max((nIndividuals*(nIndividuals-1)),1); 
nNeighboursIncArrived = zeros(nIndividuals, numClasses + 1);                        % Number of observed neighbours, including those at target.
diffDirection = zeros(nIndividuals,1);                                              % Difference in direction between heading and target.  
concentrationParameters(tSaveCount,iRepeat, 1) = mean(concentrationIndividual);     % Average concentration parameter at last reorientation.


                                                                                    % Average effective velocity in direction of target:
anglesToTarget = navigationFieldGoal(position(:,1), position(:,2));                                            
vectorsToTarget = [cos(anglesToTarget), sin(anglesToTarget)];                  
if modulateSpeeds ~= "off"
    velocityVectors = newVelocity;
else
    velocityVectors = velocity*[cos(heading), sin(heading)];                   
end
velocityToTarget = dot(velocityVectors, vectorsToTarget, 2);                    
effectiveVelocity(tSaveCount, iRepeat, 1) = mean(velocityToTarget);             
                                                                                    % Velocity histogram 
velocityHist(tSaveCount, :, 1) = velocityHist(tSaveCount, :, 1) + histcounts(velocityToTarget, linspace(-1, 1, nHistVelocity)); 

nNeighbours = zeros(nIndividuals,numClasses + 1);                                   % Number of neighbours for each individual (to be filled in).
                                                                                    % First col is neighbours of all classes, 
                                                                                    % second coll is neighbours of class 1, etc.                          
                                                                                                                                                                   
for i = 1:nIndividuals                                                              % Loop over individuals to calculate the number of neighbours, 
                                                                                    % and the differences in heading from the target.
    neighbours = find(pairDistances(i,:)>0&pairDistances(i,:)<sensingRangeField(position(i,1),position(i,2)));
    nNeighbours(i,1) = numel(neighbours);
    
    for classIdx = 1:numClasses
        neighboursInClass = find(runClass(neighbours) == classIdx);                 % How many neighbours in each class.
        nNeighbours(i, classIdx + 1)  = numel(neighboursInClass);
    end
   
    
    nNeighboursIncArrived(i,:) = nNeighbours(i,:);                                  % If cooperative == "off", only count actively navigating neighbours,
                                                                                    % as only they are used during reorientation.
    
                                                                                    % If cooperative is on, count nearby arrived whales as their information
                                                                                    % is used in the heading calculation.
    if cooperative == "target" && ~(isempty(arrivedPosition))
        if sqrt((position(i,1) - goalLocation(1))^2 + (position(i,2) - goalLocation(2))^2) < sensingRange       % Cooperative navigation kicks in when agent near the target
            nNeighboursIncArrived(i,1) = numel(arrivedIDs) + nNeighbours(i,1);                                  % Total number of neighbours for whale i.
            for classIdx = 1:numClasses
                arrivedNeighboursInClass = find(arrivedClass == classIdx);                                       % which arrived neighbours are of class classIdx
                nNeighboursIncArrived(i, classIdx + 1) = nNeighbours(i, classIdx + 1) + numel(arrivedNeighboursInClass);
            end
        end
    end
    
    diffDirection(i) = abs(heading(i)+pi - mod(navigationFieldGoal(position(i,1),position(i,2))+pi,2*pi));           % Direction between heading and target.
    if diffDirection(i) > pi
        diffDirection(i) = pi - mod(diffDirection(i),pi);
    end
end

meanNeighbours(tSaveCount,iRepeat, 1) = mean(nNeighbours(:,1));                             % Average number of actively navigating neighbours across all individuals
meanNeighboursIncArrived(tSaveCount, iRepeat, 1) = mean(nNeighboursIncArrived(:,1));        % Average number of neighbours including arrived, across all individuals






if



meanDifferenceDirection(tSaveCount,iRepeat, 1) = mean(diffDirection);          % Average difference between heading and target.
nIndividualsRemaining(tSaveCount,iRepeat, 1) = nIndividuals;                   % Number of individuals yet to arrive at target.
directionHist(:,1) = directionHist(:,1) + histcounts(mod(heading,2*pi),linspace(0,2*pi,nHistDirection))'; % Generate histogram of headings.

if nIndividuals/nIndividualsStart <= 0.1 && majorityCheck(1) == 0              % Time when 90% of individals have arrived at target.
    majorityGone(iRepeat, 1) = t;
    majorityCheck(1) = 1;
end



% Save class specific data, e.g. averages calculated for only class 1
% -------------------------------------------------------------------------
% Recall: % Page 1 = All individuals, Page 2 = class 1, ..., Page N+1 = class N


for idx = 1:numClasses
    page = idx + 1;
    classIdx = populationStructure(idx, 1);                                 % Used in case there is only 1 class navigating which we want named 'class 2'
    
    agentsInClass = find(runClass == classIdx);                             % Indices of remaining agents within the current class.
    numInClass = numel(agentsInClass);
    
    if numInClass/populationStructure(idx, 4) <= 0.1 && majorityCheck(page) == 0    % Store time when 90% of current class has reached target.          
        majorityGone(iRepeat, page) = t;
        majorityCheck(page) = 1;
    end

                                                                            % If numInClass == 0, don't try to update data, it will just give NaNs.
                                                                            % Not updating the corresponding statistics for the given timestep 
                                                                            % leaves the value as zero. This will bleed into the results when averages
                                                                            % are taken over the repeats. It's probably better to initialise 
                                                                            % fields as NaNs rather than 0.
    
    arrivedAgentsInClass = find(arrivedClass == classIdx);                  % Agents in the current class already at the target.
   
    
    if numInClass ~= 0                                                      % Only do calculations if there are agents in class still navigating.
        
                                                                            % Mean distance from target of active navigators.
        classPositions = position(agentsInClass,:);
        classXPosition = mean(classPositions(:,1));
        classYPosition = mean(classPositions(:,2));
        classDistanceToGoal = sqrt((classXPosition-goalLocation(1))^2+(classYPosition-goalLocation(2))^2);     
        classVelocityToTarget = mean(velocityToTarget(agentsInClass));
        
                                                                            % Mean distance from target including arrived whales.
        classPositionsAll = [position(agentsInClass,:); arrivedPosition(arrivedAgentsInClass,:)];                
        classXPositionAll = mean(classPositionsAll(:,1));
        classYPositionAll = mean(classPositionsAll(:,2));
        classDistanceToGoalAll = sqrt((classXPositionAll-goalLocation(1))^2+(classYPositionAll-goalLocation(2))^2);
        
                                                                            % Number of neighbours.
        classnNeighbours = mean(nNeighbours(agentsInClass, 1));
        classnNeighboursIncArrived = mean(nNeighboursIncArrived(agentsInClass, 1));
        
                                                                            % Difference in direction from target direction.
        classDiffDirection = mean(diffDirection(agentsInClass));
        
                                                                            % concentration parameters.
        classConcentrationParameters = mean(concentrationIndividual(agentsInClass)); 
        
                                                                            % Cluster measure (average pairdistance within the class).
        classPairDistVec = pdist(classPositions);
        if numInClass == 0
            classPairDistVec = [0];
        end
        classClusterMeasure = 2*sum(classPairDistVec)/max((numInClass*(numInClass-1)),1);   
        
                                                                            % Cluster measure histogram at specified times.
        if ismember(tSaveCount, histClustermeasureSnapshots)
            clustermeasureHist(histClustermeasureSnapshots == tSaveCount, :, classIdx) = clustermeasureHist(histClustermeasureSnapshots == tSaveCount, :, classIdx)...
                                                        + histcounts(classPairDistVec, linspace(0, 1000, nHistClustermeasure));
        end

        
                                                                            % Save the data for the current class and timestep to the overall
                                                                            % storage arrays.
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


                                                                            % Average number of neighbours for each class. 
                                                                            % Note that this field has different page structure:
                                                                            % Page 1 = class 1, Page 2 = class 2 ...
for sensingClass = 1:numClasses
    classSpecificNeighbours(sensingClass, tSaveCount, iRepeat, 1) =  mean(nNeighboursIncArrived(find(runClass == sensingClass), 1)); 
    for sensedClass = 1:numClasses
        classSpecificNeighbours(sensingClass, tSaveCount, iRepeat, sensedClass + 1) = mean(nNeighboursIncArrived(find(runClass == sensingClass), sensedClass+1));     
    end
end


                                                                            
if iRepeat == 1                                                             % Trajectories: Save all individuals positions for repeat 1.
    remainingIndividuals = runIDs;
    arrivedIndividuals = arrivedIDs;
    xPositionsIndividualsRep1(tSaveCount, runIDs) = position(:,1)';
    yPositionsIndividualsRep1(tSaveCount, runIDs) = position(:,2)';
    if ~isempty(arrivedIDs)
        xPositionsIndividualsRep1(tSaveCount, arrivedIDs) = arrivedPosition(:,1)';
        yPositionsIndividualsRep1(tSaveCount, arrivedIDs) = arrivedPosition(:,2)';
    end
end



if numClasses == 2 && contactCheck == 1                                        % If two classes, save the time at which they first lose contact.
    if classSpecificNeighbours(1, tSaveCount, iRepeat, 3) == 0             
        lastContact(iRepeat) = t;
        contactCheck = 0;                                       
    end 
end


tSaveCount = tSaveCount + 1;                                                                                       
