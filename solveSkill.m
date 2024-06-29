%% Given class 1 population fraction, delta, and navigation skill 
% kappa_1, solve for the class 2 navigation skill kappa_2 which 
% maintains the same average individual velocity towards target as for 
% the uniform population with kappa = 1.

% Should only be fed data for which a non-negative solution exists. Will 
% error out otherwise.
function [kappa_2, err] = solveSkill(delta, kappa_1)
    % Given one class with known navigation skill, skill1, and fraction of population, frac1,
    % solve for the navigation skill of the other class, such that the mean
    % velocity towards target:
    
    % frac_1 I_1(kappa_1)/I_0(kappa_1) + frac_2 I_1(kappa_2)/I_0(kappa_2) 
    
    % does not change from the mean velocity towards the target
    % for the uniform population with skill 1, (i.e. original code
    % had I_1(1)/I_0(1).
    
    % First check that a non negative solution exists for the given delta,
    % kappa_1. Error out if not
    if solChecker(delta, kappa_1) == 0
        error("No non negative solution for given inputs")
    end
    
    syms x;
    kappa_2 = double(vpasolve(besrat(x) == besrat(1)/(1-delta) - (delta/(1-delta))*besrat(kappa_1), x));
    
    % Error in the solution
    err = delta*besrat(kappa_1) + (1-delta)*besrat(kappa_2) - besrat(1);
end


