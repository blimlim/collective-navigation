%besrat(1)

[a, b] = solveSkill(0.5,0.5);
a
b

function [skill2, err] = solveSkill(skill1, frac1)
    % Given one class with known navigation skill, skill1, and fraction of population, frac1,
    % solve for the navigation skill of the other class, such that the mean
    % velocity towards target:
    
    % frac_1 I_1(skill_1)/I_0(skill_1) + frac_2 I_1(skill_2)/I_0(skill_2) 
    
    % does not change from the mean velocity towards the target
    % for the uniform population with skill 1, (i.e. original code
    % had I_1(1)/I_0(1).
    syms x;
    skill2 = vpasolve(besrat(x) == besrat(1)/(1-frac1) - (frac1/(1-frac1))*besrat(skill1), x);
    err = frac1*besrat(skill1) + (1-frac1)*besrat(skill2) - besrat(1);
end


