function [G,Gmv,Ge] = ObstacleAvoidancCostJacobian(X,U,e,data)
% Jacobian of the cost function for obstacle avoidance

p = data.PredictionHorizon;

% with respect to the state trajectories
G = zeros(p,4);    %TODO: complete

% with respect to control inputs
Gmv = zeros(p,2);   %TODO: complete

% with repect to slack variable e
Ge = 0;

end

