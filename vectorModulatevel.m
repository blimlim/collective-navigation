function newSpeed = vectorModulatevel(agentPositions, neighbourMeans, goalPosition, speed, maxDist)
    % Function to reduce the speed of an individual class 2 whale (better navigator)
    % according to its distance from the average position of its neighbours. 
    
    % Reduce velocity via clipped linear function, then add clipped normal noise if desired
    
    goalVecs = goalPosition - agentPositions;
    goalUnitVecs = goalVecs ./ sqrt(sum(goalVecs.*goalVecs, 2));
    
    dispFromMeans = agentPositions - neighbourMeans;
    
    compToGoals = dot(dispFromMeans, goalUnitVecs, 2);
    
    scaleFactor = linearScalefactor(compToGoals, maxDist);
    
    newSpeed = speed .* scaleFactor;
    
end

function velLinScaleFactors = linearScalefactor(compToGoals, maxdist)                    % Function for linear scaling of velocity 
                                                   
    velLinScaleFactors = 1 - compToGoals/maxdist;  
    
    
    velLinScaleFactors(velLinScaleFactors < 0) = 0;                                        % Don't have negative speeds
    velLinScaleFactors(velLinScaleFactors > 1) = 1;                                        % Don't increase the speed. 
                                                                                  
end


