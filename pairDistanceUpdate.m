%% Code for Johnston and Painter to update distances between pairs of
%% individuals. Called by Homing_Script.m.

pairDistanceVec = pdist(position);                                  % Calculate distances between all pairs of individuals.
pairDistances = squareform(pairDistanceVec);

% If there's only one individual, the above will make pairDistances empty
if nIndividuals == 1
    pairDistances = [0];
end

% Old:
% pairDistances(triu(ones(nIndividuals)==1,1)) = pairDistanceVec;     % Set pair distances for i =/= j.
% pairDistances(tril(ones(nIndividuals)==1,-1)) = pairDistanceVec;    % Set pair distances for i =/= j.     
