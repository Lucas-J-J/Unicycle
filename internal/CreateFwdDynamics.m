function [RHS, A, RHS_noTorque] = CreateFwdDynamics(y, T, SF, LF, RF, g, simplify)
%createFwdDynamics creates matrix and vector for solving dy = f(y)
%   y is state vector, format ["phi","phi'","theta","theta'",
%   "alpha","alpha'","psi","psi'"]'
%   T, SF, LF, RF are scalars, pre-evaluated at the current time
%   g is geometry struct
%   smallangle is string: "small angle",  "upright" or "none" /other
%       Signifies an approximation which is used for sinusoidal functions

ph = y(1); phd = y(2); th = y(3); thd = y(4);
al = y(5); ald = y(6); ps = y(7); psd = y(8);


MyRoundPi = @(x) pi/2*round(x/(pi/2)); %round to nearest pi/2, used for angle simplify
if strcmpi(simplify, "upright") 
    %apply upright approximation: alpha = pi/2 and theta = pi/2
    c = @(x) cos(MyRoundPi(x));
    s = @(x) sin(MyRoundPi(x)); 
elseif strcmpi(simplify, "small angle")
    %apply small angle approximation: linear term of Taylor series
    %Refers to functions at bottom of script including rules
    c =  @cos_smang;
    s = @sin_smang;
else
    %no approximation applied
    c =  @cos;
    s = @sin;
end

%RHS and A are defined based on equations in CreateMat.m and CreateVec.m
%Internal consistency is guaranteed by (1) use of Mathematica for algebra
%and (2) feeding output into input and reproducing initial signal.

%RHS is a vector (elems assigned individually for readability)
%Form [torque]-[first derivative squared terms]
RHS1 =  -((1/2)*(ald^2*(-g.lf)*g.mf*g.Ro*s(al - th) + ...
    ald^2*g.lf*g.mf*g.Ro*s(al + th) +...
    ald*g.lf*g.mf*phd*g.Ro*s(al + 2*th) +...
    - ald*g.lf*g.mf*phd*g.Ro*s(al - 2*th) +...
    - g.lf*g.mf*phd^2*g.Ro*s(al - th) +...
    g.lf*g.mf*phd^2*g.Ro*s(al + th) +...
    g.lf*g.mf*phd*g.Ro*thd*s(al + 2*th) +...
    g.lf*g.mf*phd*g.Ro*thd*s(al - 2*th) +...
    - 2*g.lf*g.mf*phd*g.Ro*thd*s(al) +...
    - 2*g.Iw*phd*thd*s(th) - 2*g.m*phd*g.Ro^2*thd*s(th) +...
    g.m*psd*g.Ro^2*thd*s(2*th) +...
    - 2*g.mf*phd*g.Ro^2*thd*s(th) + g.mf*psd*g.Ro^2*thd*s(2*th)));
RHS2 =  -(c(al)*s(th)*(2*ald*g.lf*g.mf*g.Ro*thd*c(al) +...
    phd*(ald*g.Ib + g.Iw*psd + g.m*psd*g.Ro^2 ...
    + g.mf*psd*g.Ro^2 - (g.Ip - g.Iw)*phd*c(th) +...
    2*ald*g.lf*g.mf*g.Ro*s(al)*s(th))) +...
    s(al)*((-(ald*g.Ib + g.Iw*psd))*thd +...
    - (g.m + g.mf)*psd*g.Ro^2*thd*c(th)^2 +...
    g.lf*g.mf*phd^2*g.Ro*c(al)*c(th)^3 +...
    c(th)*(phd*(g.Ib + 2*g.Ip - g.Iw +...
    g.m*g.Ro^2 + g.mf*g.Ro^2)*thd +...
    g.lf*g.mf*g.Ro*c(al)*(-phd^2 + thd^2 + 1.*phd^2*s(th)^2))) +...
    g.lf*g.mf*phd*g.Ro*thd*s(al)^2*s(2*th));
RHS3 = -( - (g.lf*g.mf*g.Ro*thd*(phd - psd*c(th)) +...
    (1/2)*(-g.Ib + g.If + g.lf^2*g.mf)*c(al)*(-phd^2 + 2*thd^2 + phd^2*c(2*th)))*s(al) +...
    (phd*thd*(-g.If + (-g.Ib + g.If)*c(2*al)) +...
    c(al)*(g.lf*g.Wf + g.l*g.Wr - g.lf*g.mf*phd*psd*g.Ro*c(th)))*s(th) +...
    - 2*g.lf^2*g.mf*phd*thd*s(al)^2*s(th));
RHS4 = -((-g.lf)*g.mf*g.Ro*c(al)^2*c(th)*(ald^2 + phd^2 + 2*ald*phd*c(th)) +...
    - c(th)*(g.lf*g.Wf + g.l*g.Wr + g.lf*g.mf*g.Ro*(ald^2 + phd^2 + thd^2)*s(al)^2 +...
    - (g.Ib - g.If + g.Ip - g.Iw - g.lf^2*g.mf)*phd^2*s(al)*s(th)) +...
    thd*c(al)*(ald*g.Ib - 2*ald*g.If - 2*ald*g.lf^2*g.mf - g.Iw*psd +...
    phd*(g.Ib + 2*g.Ip - g.Iw + g.m*g.Ro^2 + g.mf*g.Ro^2)*c(th) +...
    - (g.m + g.mf)*psd*g.Ro^2*c(th)^2 +...
    2*g.lf*g.mf*g.Ro*(-ald + phd*c(th))*s(al)*s(th)) +...
    -   phd*(2*ald*g.lf*g.mf*g.Ro*s(al)^2 +...
    - (ald*(g.Ib - 2*(g.If + g.lf^2*g.mf)) - psd*(g.Iw + (g.m + g.mf)*g.Ro^2))*s(al)*s(th) +...
    g.lf*g.mf*psd*g.Ro*s(th)^2));

RHS_noTorque = [RHS1 RHS2 RHS3 RHS4]';
RHS = [RHS1 RHS2 RHS3 RHS4]' - [T SF LF RF]';

%A is matrix of second-derivative coefficients.
A = [(1/4)*(-(g.lf*g.mf*g.Ro*c(al + 2*th)) + g.lf*g.mf*g.Ro*c(al - 2*th) + 4*g.Iw*c(th)), 0, (1/2)*(g.lf*g.mf*g.Ro*c(al - th) - g.lf*g.mf*g.Ro*c(al + th)), (1/2)*(2*g.Iw - g.m*g.Ro^2*c(2*th) + g.m*g.Ro^2 - g.mf*g.Ro^2*c(2*th) + g.mf*g.Ro^2);...
    ((-g.lf)*g.mf*g.Ro*c(al)^2 + s(al)*((-g.lf)*g.mf*g.Ro*c(th)^2*s(al) + (g.Ib + g.Ip)*s(th))),c(al)*(g.Ib + g.Ip + g.m*g.Ro^2 + g.mf*g.Ro^2 + g.lf*g.mf*g.Ro*s(al)*s(th)), (-1)*g.lf*g.mf*g.Ro*c(th), - (g.m + g.mf)*g.Ro^2*c(th)*s(al)*s(th);...
    (g.If + g.lf^2*g.mf)*c(th), - g.lf*g.mf*g.Ro*c(al)*c(th), (g.If + g.lf^2*g.mf), g.lf*g.mf*g.Ro*s(al)*s(th);...
    c(al)*s(th)*(g.If + g.Ip + g.lf^2*g.mf + g.lf*g.mf*g.Ro*s(al)*s(th)), ((-(g.If + g.Ip + g.lf^2*g.mf + g.m*g.Ro^2 + g.mf*g.Ro^2))*s(al) - g.lf*g.mf*g.Ro*s(th) - g.lf*g.mf*g.Ro*s(al)^2*s(th)), 0, - (g.m + g.mf)*g.Ro^2*c(al)*c(th)*s(th)];

%nested functions
function y = cos_smang(x)
    x = mod(x, 2*pi);
    rounded = MyRoundPi(x);

    %Taylor series about each multiple of pi/2
    %This allows for correct calculation of multiplications and additions
    %in argument, e.g. cos(alpha-theta).
    if rounded == 0 || rounded == 2*pi
        y = 1;
    elseif rounded == pi/2
        y = -(x-pi/2);
    elseif rounded == pi
        y = -1;
    elseif rounded == 3*pi/2
        y=x-3*pi/2;
    else
        disp(x)
        disp(rounded)
        error("output not assigned")
    end
end

function y = sin_smang(x)
    x = mod(x, 2*pi);
    rounded = MyRoundPi(x);
    
    %Taylor series about each multiple of pi/2
    %This allows for correct calculation of multiplications and additions
    %in argument, e.g. cos(alpha-theta).
    if rounded == 0 || rounded == 2*pi
        y = x;
    elseif rounded == pi/2
        y = 1;
    elseif rounded == pi
        y = -(x-pi);
    elseif rounded == 3*pi/2
        y=-1;
    else
        disp(x)
        disp(rounded)
        error("output not assigned")
    end

end

end

