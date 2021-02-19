function [vXYZ, pathXYZ] = CalculatePath(t, phi, theta, alpha, psi, g, init)
%CalculatePath finds path in absolute coordinates

if nargin > 7
    init = [0, 0, 0];
end

%velocity in XYZ coordinates, as calculated using Mathematica
vXYZ = [g.Ro*(theta(2, :).*sin(phi(1, :))-psi(2, :).*cos(phi(1, :)).*sin(theta(1, :)));...
        -g.Ro*(theta(2, :).*cos(phi(1, :))+psi(2, :).*sin(phi(1, :)).*sin(theta(1, :)));...
        zeros(1, length(t))];
    
% vXYZ = [vXYZ; zeros(1, length(t))];
% for i=1:length(t)
%     vXYZ(4,i)=norm(vXYZ(1:3,i));
% end
pathXYZ = zeros(size(vXYZ));
for i=1:length(t)
    pathXYZ(:,i) = trapz(t(1:i), vXYZ(:, 1:i), 2)+ init;
end

% pathXYZ = pathXYZ ;

end