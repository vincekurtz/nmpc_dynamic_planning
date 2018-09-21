function cineq = CollisionAvoidanceConstraintFcn(X,U,e,data,params)
% Custom constraint function for obstacle avoidance.

p = data.PredictionHorizon;
r_safe = 2*params;  % safe distance TODO: set this as a parameter

X1 = X(2:p+1,1:2);
X2 = X(2:p+1,2:3);

dist = vecnorm(X1' - X2');

cineq = -(dist - r_safe)';

end

