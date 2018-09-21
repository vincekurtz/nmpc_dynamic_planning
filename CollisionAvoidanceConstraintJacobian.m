function [G, Gmv, Ge] = CollisionAvoidanceConstraintFcn(X,U,e,data,params)
% Jacobian on custom constraint function for obstacle avoidance.

p = data.PredictionHorizon;
Nx = data.NumOfStates;
Nc = p;
Nmv = length(data.MVIndex);

% With respect to state variables
X1 = X(:,1:2)'; % robot
X2 = X(:,3:4)'; % obstacle

G = (X1 - X2)/vecnorm(X1-X2);

% With respect to control variables
Gmv = zeros(p,Nmv,Nc);

% With respect to slack variable
Ge = zeros(Nc);

end

