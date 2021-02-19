function [SOLN,SOLN_key,t, toppled_tag] = ForwardDynamics(T, SF, RF, LF, t_start, t_end, init, g, simplify)
%ForwardDynamics solves for angles given torques
%   T, SF, RF, LF are function handles
%   init is initial values of angles and first derivative
%   tend is the ending time; start time = 0  is assumed
%   g is geometry struct
%   smallangle is string: "small angle",  "upright" or "none" /other
%       Signifies an approximation which is used for sinusoidal functions

toppled_tag = false;

SOLN_key = ["phi","phi'","phi''", "theta","theta'", "theta''", "alpha","alpha'","alpha''", "psi","psi'","psi''"];
t_topple = t_end;
function [dy] = f(t,y)
	%DO NOT use this function outside of ForwardDynamics
    %This is because goemetry and torque specification is inherited from
    %ForwardDynamics
    % y is ["phi","phi'","theta","theta'", "alpha","alpha'","psi","psi'"]
    
    T_l = T; SF_l = SF; RF_l = RF; LF_l = LF;
    
    %equation is processed in Mathematica
    [RHS, A] = CreateFwdDynamics(y, T_l, SF_l, LF_l, RF_l, g, simplify);

    phd = y(2); thd = y(4); ald = y(6);psd = y(8);

    dd=LUSolve(A,RHS);
    phdd = dd(1); thdd = dd(2); aldd = dd(3); psdd = dd(4);
    
    dy = [phd, phdd, thd, thdd, ald, aldd, psd, psdd]';
    
    if (sin(y(5))<0.7071067811865 || sin(y(3))<0.93969262078590 ) %20 degrees for theta, 45 deg for alpha
        if ~toppled_tag
            fprintf("Unicyle no longer upright at %.4f s\n", t)
            toppled_tag = true;
            t_topple = t;
        end
        dy = zeros(8,1);
    end
    
end

opts = odeset('RelTol',1e-3,'AbsTol',1e-6, 'MaxStep', 0.005);

[t,SOLN_raw] = ode45(@(t,y) f(t,y),[t_start t_end],init, opts);

%only returning "upright" potion of solution
t = t(t<=t_topple);
SOLN_raw = SOLN_raw(1:length(t), :);

 %{
Output of ode45 is in the format 
["phi","phi'","theta","theta'", "alpha","alpha'","psi","psi'"]
Using rows as data corresponding to t and columns as variables 
This is necessary as only angle and velocity are state variables
This should be transformed to
["phi","phi'","phi''", "theta","theta'", "theta''", "alpha","alpha'","alpha''", "psi","psi'","psi''"]
Using columns corresponding to time
We must numerically differentiate velocity with gradient() to recover acceleration
%}

SOLN = [ SOLN_raw(:,1:2)'; gradient(SOLN_raw(:,2),t)';...
         SOLN_raw(:, 3:4)'; gradient(SOLN_raw(:,4),t)';...
         SOLN_raw(:, 5:6)'; gradient(SOLN_raw(:,6),t)';...
         SOLN_raw(:, 7:8)'; gradient(SOLN_raw(:,8),t)'];

end