%% NMPC Trajectory Planning and Control for a Single-Integrator robot
%
% Using nmpc to perform path planning and obstacle avoidance for a simple
% fully actuated mobile robot

clear;
clc;

%% System Model
%
% Our system consists of a single obstacle and a single robot in 2d space.
% The states are as follows:
%
% x(1) - x position of the robot
% x(2) - y position of the robot
% x(3) - x position of the obstacle
% x(4) - y position of the obstacle
%
% We assume perfect observability, so the output y(k) = x(k).
%
% Control inputs are desired velocities
%
% u(1) - robot x velocity
% u(2) - robot y velocity

nx = 4;
ny = 4;
nu = 2;
nlobj = nlmpc(nx,ny,nu);

Ts = 0.05;   % Sampling time
p = 60;     % planning horizon

% Control limits
u_lb = -1;
u_ub = 1;

% Initial conditions
x0 = [3;3;2;2];
u0 = zeros(nu,1);

% Obstacle radius is treated as a parameter of the system
r_safe = 0.2;
params = r_safe;
nlobj.Model.NumberOfParameters = 1;

%% Prediction Model

nlobj.Model.StateFcn = "RobotDynamicModel";
nlobj.Jacobian.StateFcn = "RobotDynamicModelJacobian";

nlobj.Ts = Ts;
nlobj.PredictionHorizon = 60;
nlobj.ControlHorizon = 60;

%% Cost Function

nlobj.Optimization.CustomCostFcn = "ObstacleAvoidanceCostFcn";
nlobj.Optimization.ReplaceStandardCost = true;
% TODO: include jacobian

%% Constraints

% Control bounds
for ct = 1:nu
    nlobj.MV(ct).Min = u_lb;
    nlobj.MV(ct).Max = u_ub;
end

% Collision Avoidance constraint
%nlobj.Optimization.CustomIneqConFcn = "CollisionAvoidanceConstraintFcn";
% TODO: include jacobian

%% Validation

% double check that everything above looks OK
validateFcns(nlobj,x0,u0,[],{params});

%% Trajectory Planning

options = nlmpcmoveopt;
options.parameters = {params};
% Find the next p moves
tic;
[~,~,info] = nlmpcmove(nlobj,x0,u0,[],[],options);
toc;  % time how long this takes

% And plot the results:
% Robot path
figure;
plot(info.Xopt(:,1), info.Xopt(:,2),'bo')
hold on
xlabel('x position')
ylabel('y position')

% Obstacle position
rectangle('Position',[x0(3)-r_safe, x0(4)-r_safe, 2*r_safe, 2*r_safe],'Curvature',[1,1],'FaceColor',[0.5,0.5,0.5]);


%% Trajectory following: the real-time stuff

% We'll also use non-linear mpc to track the trajectory
nlobj_tracking = nlmpc(nx,ny,nu);
nlobj_tracking.Model.NumberOfParameters = nlobj.Model.NumberOfParameters;

% Same model as above
nlobj_tracking.Model.StateFcn = nlobj.Model.StateFcn;
nlobj_tracking.Jacobian.StateFcn = nlobj.Jacobian.StateFcn;

% But shorter prediction and control horizons for faster speed
nlobj_tracking.Ts = Ts;
nlobj_tracking.PredictionHorizon = 10;
nlobj_tracking.ControlHorizon = 4;

% We'll use a standard cost function, with slightly higher weight on states
nlobj_tracking.Weights.ManipulatedVariablesRate = 0.2*ones(1,nu);
nlobj_tracking.Weights.OutputVariables = 5*ones(1,nx);

% Custom constraints for obstacle avoidance
nlobj_tracking.Optimization.CustomIneqConFcn = "CollisionAvoidanceConstraintFcn";

% Same bounds as before on control inputs
for ct = 1:nu
    nlobj_tracking.MV(ct).Min = -1;
    nlobj_tracking.MV(ct).Max = 1;
end

% We'll validate again to be sure everything is good
validateFcns(nlobj_tracking,x0,u0,[],{params});

%% Finally, run the simulation
Tsteps = 50;
xHistory = x0';
uHistory = [];
lastMV = zeros(nu,1);

% Treat the trajectory from path planning as the reference
Xopt = info.Xopt;
Xref = [Xopt(2:p+1,:); repmat(Xopt(end,:),Tsteps-p,1)];

options = nlmpcmoveopt;
options.Parameters = {params};
for k = 1:Tsteps
    % Plot actual position
    plot(xHistory(k,1),xHistory(k,2),'rx')
    % Obtain plant measurements with sensor noise.
    yk = xHistory(k,:)' + randn*0.1;
    % In our case thes are the same as state measurements
    xk = yk;
    % Compute the control moves with reference previewing.
    [uk,options,track_info] = nlmpcmove(nlobj_tracking,xk,lastMV,Xref(k:min(k+9,Tsteps),:),[],options);
    % Store the control move and update the last MV for the next step.
    uHistory(k,:) = uk';
    lastMV = uk;
    % Update the real plant states for the next step by solving the
    % continuous-time ODEs based on current states xk and input uk.
    ODEFUN = @(t,xk) RobotDynamicModel(xk,uk);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    % Store the state values.
    xHistory(k+1,:) = YOUT(end,:);
    
    % plot planned trajectory
    x1_plan = track_info.Xopt(:,1);
    x2_plan = track_info.Xopt(:,2);
    plot(x1_plan,x2_plan,'g-');
    
    % Wait a sec so we can see a pretty animation
    pause(0.1);
end

legend('Planned Trajectory','Actual Trajectory','MPC Plan','Location','southeast')

