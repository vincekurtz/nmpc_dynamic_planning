function [A,Bmv] = RobotDynamicModelJacobian(x,u,params)
% Jacobian of state equations of a simple robot and static obstacle 
% 
% States:
% x(1) - x position of the robot
% x(2) - y position of the robot
% x(3) - x position of the obstacle
% x(4) - y position of the obstacle
%
% Controls
% u(1) - robot x velocity
% u(2) - robot y velocity

% States derivatives w.r.t. other states
A = zeros(4,4);

% States derivatives w.r.t. control inputs
Bmv = zeros(4,2);
Bmv(1,1) = 1;
Bmv(2,2) = 1;

end