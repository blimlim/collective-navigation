
function r = besrat(x)
    % Return ration of modified Bessel functions of first kind
    % I_1(x)/I_0(x)
    r = besseli(1,x)./besseli(0,x);
end
