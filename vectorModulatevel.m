function newSpeed = vectorModulatevel(agentPositions, neighbourMeans, goalPosition, speed, maxDist, modulateSpeeds)
    % Function to reduce the speed of an individual class 2 whale (better navigator)
    % according to its distance from the average position of its neighbours. 
    
    % Reduce velocity via clipped linear function, then add clipped normal noise if desired
    
    % modulateSpeeds: off, goalAxis, or absoluteDistance
    assert(modulateSpeeds ~= "off", "vectorModulatevel was called while modulate speeds is 'off'")
    
    if (modulateSpeeds == "goalAxis")
        goalVecs = goalPosition - agentPositions;                           
        goalUnitVecs = goalVecs ./ sqrt(sum(goalVecs.*goalVecs, 2));        % Unit vectors from agents to goals.
        dispFromMeans = agentPositions - neighbourMeans;                    % Displacements of agents from neighbour mean positions.
        compToGoals = dot(dispFromMeans, goalUnitVecs, 2);                  % Components of displacements in goal direction.
        scaleFactor = linearScaleFactor(compToGoals, maxDist);
        newSpeed = speed .* scaleFactor;
    elseif (modulateSpeeds == "absoluteDistance") 
        dispFromMeans = agentPositions - neighbourMeans;
        absoluteDistances = sqrt(dot(dispFromMeans, dispFromMeans, 2));
        signedAbsoluteDistances = absoluteDistances .* (-sign(dispFromMeans(:,1))); % WARNING: ONLY WORKS FOR TRAVELING IN NEGATIVE X DIRECTION
        
        scaleFactor = linearScaleFactor(signedAbsoluteDistances, maxDist);
        newSpeed = speed .* scaleFactor;
        
    else
        Error(sprintf("Unknown value of modulateSpeeds in vectorModulatevel call. \n Required: 'off', 'goalAxis', or 'absoluteDistance' \n Provided: %s", modulateSpeeds))
    end
    
end

function velLinScaleFactors = linearScaleFactor(compToGoals, maxdist)                    % Function for linear scaling of velocity 
                                                   
    velLinScaleFactors = 1 - compToGoals/maxdist;  
    
    
    velLinScaleFactors(velLinScaleFactors < 0) = 0;                                        % Don't have negative speeds
    velLinScaleFactors(velLinScaleFactors > 1) = 1;                                        % Don't increase the speed. 
    velLinScaleFactors(isnan(velLinScaleFactors)) = 1;                                     % NaN's will occur when an agent has no neighbours,
                                                                                           % travel as normal if so.
                                                                                  
end


