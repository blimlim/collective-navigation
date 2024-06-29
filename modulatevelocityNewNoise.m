function newSpeed = modulatevelocityNewNoise(agentPosition, neighbourMean, goalPosition, speed, maxDist, noisyModulation, noiseNormalSD, normalMeanShift)
    % Function to reduce the speed of an individual class 2 whale (better navigator)
    % according to its distance from the average position of its neighbours. 
    
    % Reduce velocity via clipped linear function, then add clipped normal noise if desired
    
    goalUnitVec = (goalPosition - agentPosition)/norm(goalPosition - agentPosition);
    
    dispFromMean = agentPosition - neighbourMean;    % Displacement of current agent from neighbour mean.
    
    compToGoal = dispFromMean * (goalUnitVec');                                     % Distance of each agent from mean position along the 
                                                                                    % axis towards the goal from the mean position
    
    if noisyModulation == "off"                                                     % Reduce whale's velocity via piecewise linear funcion.
                                                                                    % No change if behind neighbours, linear decrease
                                                                                    % if 0<distance<maxdist
                                                                                    % and stop completly if distance>maxdist
        rawScaleFactor = linearScalefactor(compToGoal, maxDist);
        scaleFactor = rawScaleFactor;
        
    elseif noisyModulation == "simple"                                              % Same process as noisy modulation == "off", but add on 
                                                                                    % normal noise to scaling factor, and clip to be between 
                                                                                    % 0 and 1.
        rawScaleFactor = linearScalefactor(compToGoal, maxDist);
        noisedScaleFactor = rawScaleFactor + normrnd(0, noiseNormalSD);
        scaleFactor = max(0, min(1, noisedScaleFactor));                                % Clip to [0,1]
         
    elseif noisyModulation == "logitnormal_1side" || noisyModulation == "logitnormal_2side"
                                                                                    % Sample scaling factors from a logitnormal distribution.
                                                                                    % For "1side" option, only do this when distance >0. This 
                                                                                    % option allows for a nonzero shift in the normal mean
                                                                                    % to counter sudden velocity reductions.
                                                                                    % When distance < 0, this option sets the scaling 
                                                                                    % factor to 1.
                                                                                    
                                                                                    % For "2side" option, sample scaling factors regardless of 
                                                                                    % whether distance < 0 or not. 
                                                                                    % For this option, I've forced it to use a 0 shift on the mean,
                                                                                    % in spite of any provided value of normalMeanShift,
                                                                                    % though this could be changed. 
                                                                                    
        scaleFactor = logitnormScaleFactor(compToGoal, maxDist, noiseNormalSD, normalMeanShift, noisyModulation);
        
    else
        error("noisyModulation value not understood")
    end                                                                 
    newSpeed = speed * scaleFactor;
    
end

function velLinScaleFactors = linearScalefactor(compToGoal, maxdist)                    % Function for linear scaling of velocity 
    if compToGoal < 0
        compToGoal = 0;                                                              % Don't impact whales which are behind the mean
    end
                                                   
    velLinScaleFactors = 1 - compToGoal/maxdist;  
    
    
    velLinScaleFactors(velLinScaleFactors < 0) = 0;                                        % Don't have negative speeds
    velLinScaleFactors(velLinScaleFactors > 1) = 1;                                        % Don't increase the speed. This shouldn't occur but 
                                                                                     % no harm in having it here. 
                                                                                  
end


function velNoiseScaleFactors = logitnormScaleFactor(compToGoal, maxDist, noiseNormalSD, normalMeanShift, noisyModulation)
    
    if noisyModulation == "logitnormal_1side" 
        if compToGoal < 0
            velNoiseScaleFactors = 1;
        else
            velNoiseScaleFactors = logitnormrnd(-compToGoal./maxDist + normalMeanShift, noiseNormalSD);
        end
    elseif noisyModulation == "logitnormal_2side"
        velNoiseScaleFactors = logitnormrnd(-compToGoal./maxDist, noiseNormalSD); % No mean shift for 2 sided logitnormal
    end
    
end

function logitnormSample = logitnormrnd(mu, sd)
    %   mu = mean of normal distribution
    %   sd = sd of normal distribution
    normalSamp = normrnd(mu, sd);
    logitnormSample = invlogit(normalSamp);
end

function invlogit = invlogit(x)
    invlogit = exp(x)./(1+exp(x));
end


