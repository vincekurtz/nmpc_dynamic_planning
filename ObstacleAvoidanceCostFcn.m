function J = ObstacleAvoidanceCostFcn(X,U,e,data,params)
% Cost function for obstacle avoidance.

p = data.PredictionHorizon;  % length of X and U

P = [1 0 0 0 ; 0 1 0 0 ; 0 0 0 0 ; 0 0 0 0];
Q = [1 0 0 0 ; 0 1 0 0 ; 0 0 0 0 ; 0 0 0 0];
R = eye(2);
s = 1;
epsilon = 1e-4;

l_reg = 0;
l_obs = 0;
for i=2:p           
    % start from 2 since X(1,:) is the current state
    l_reg = l_reg + X(i,:)*Q*X(i,:)' + U(i,:)*R*U(i,:)';
    %l_obs = s / (((X(i,1:2)-X(i,3:4))*(X(i,1:2)-X(i,3:4))') + epsilon);
end

running_cost = l_reg + l_obs;
terminal_cost = X(p+1,:)*P*X(p+1,:)';

J = running_cost + terminal_cost;

end

