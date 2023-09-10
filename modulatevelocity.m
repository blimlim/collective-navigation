function newvels = modulatevelocity(position, kappas, goalPosition, velocity, maxDist)
    meanPosition = mean(position, 1);                              % Average position of the population
    
    goalUnitVec = (goalPosition - meanPosition)/norm(goalPosition - meanPosition);  % Unit vector from average position towards target 
    
    dispFromMean = position - meanPosition;                           % Displacement of each agent from mean position
    
    compToGoal = dispFromMean * (goalUnitVec');                                     % Distance of each agent from mean position along the 
                                                                                    % axis towards the goal from the mean position
                                                                                    
    scaleFactor = linearScalefactor(compToGoal, maxDist);                           % Calculate velocity reduction factor based on distance from mean
    scaleFactor(kappas < max(kappas)) = 1;                                          % Only scale down the faster class ? check if this makes any 
                                                                                    % improvements.
    
    newvels = velocity * scaleFactor;
end

function velScaleFactors = linearScalefactor(compToGoal, maxdist)                    % Function for linear scaling of velocity 
    compToGoal(compToGoal <0) = 0;                                                   % Don't impact whales which are behind the mean
    velScaleFactors = 1 - compToGoal/maxdist;   
    velScaleFactors(velScaleFactors < 0) = 0;                                        % Don't have negative velocities
    velScaleFactors(velScaleFactors > 1) = 1;                                         % Never increase velocity as that would be cheating...
                                                                                        % This shouldn't be needed, but doesn't harm to have it 
                                                                                        % I think
end

