function dxdt = RobotDynamicModel(x,u,params)
% State equations of a simple robot and static obstacle 
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

B = [eye(2);zeros(2)];
dxdt = B*u;

end

