clear all; close all;

%--------------------------------------------------------------------------
% setup simulation
%--------------------------------------------------------------------------
% axes
ez = [0; 0; 1];

% number of obstacles
num_obstacle = 14;

% number of sensors
num_sensors = 5;

% number of lidar scan directions
M = 8;

% lidar noise covariance
R_lidar = 0.1 * eye(M);

% map points from lidar
map_points = [];

% initial state covariance matrix
P_rob = 0.1  * eye(3);
P_range = 0.05 * eye(3 * 4);
P_bear = 0.05 * eye(3);
P = blkdiag(P_rob, P_range, P_bear);

% process noise covariance matrix
Q_rob = 0.01 * eye(3);
Q_range = zeros(3 * 4);
Q_bear = zeros(3);
Q = blkdiag(Q_rob, Q_range, Q_bear);

% observation noise covariance matrix
R_range = 0.1 * eye(4);
R_bear = 0.1;
R = blkdiag(R_range, R_bear);

% obstacles
colobj = setup_environment();

% true initial state
x_robot = [0.2; 2; 0.2];
x_range = [0 0 4 0 10 4 10 10 4 10 0 4]';
x_bear = colobj{14}.Pose(1:3,4);
x_true = [x_robot; x_range; x_bear];

% estimated initial state
x0 = normrnd(x_true, diag(P));

% mobile robot
robot = setup_robot(x_robot);

% true simulation
sim1 = display_simulation(robot, colobj, 1);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% control robot
%--------------------------------------------------------------------------
% sample time
ts = 1;

% initial state estimate
x_est = x0;

% history of true and estimated states
x_true_hist = x_true;
x_est_hist = x_est;

% let figure update
pause(1);

key = ' ';
key_pressed = 0;
set(sim1,'KeyPressFcn',@get_key);

% exit by pressing 'q'
while key ~= 'q'
    % move forward
    u = [0.1; 0];
    [x_fwd, ~] = state_transition(x_true, u, Q, ts);
    
    % check for collision
    robot.Pose(1:2, 4) = x_fwd(1:2);
    robot.Pose(1:3, 1:3) = rot(ez, x_fwd(3));
    fwd_col = false;
    for ii=1:num_obstacle
        [obj_col, ~, ~] = checkCollision(robot, colobj{ii});
        fwd_col = fwd_col || obj_col;
    end
    
    if fwd_col
        % rotate to avoid forward collision
        u = [0; 0.3];
        [x_rot, ~] = state_transition(x_true, u, Q, ts);
        
        % check for collision
        robot.Pose(1:2, 4) = x_rot(1:2);
        robot.Pose(1:3, 1:3) = rot(ez, x_rot(3));
        rot_col = false;
        for ii=1:num_obstacle
            [obj_col, ~, ~] = checkCollision(robot, colobj{ii});
            rot_col = rot_col || obj_col;
        end
        
        if rot_col
            % move backward
            u = [-0.05; 0];
            [x_bwd, ~] = state_transition(x_true, u, Q, ts);
            
            % check for collision
            robot.Pose(1:2, 4) = x_bwd(1:2);
            robot.Pose(1:3, 1:3) = rot(ez, x_bwd(3));
            bwd_col = false;
            for ii=1:num_obstacle
                [obj_col, ~, ~] = checkCollision(robot, colobj{ii});
                bwd_col = bwd_col || obj_col;
            end
          
            if bwd_col
                break;
            else
                x_true = x_bwd;
            end
        else
            x_true = x_rot;
        end
    else
        x_true = x_fwd;
    end
    
    % update robot object
    robot.Pose(1:2,4) = x_true(1:2);
    robot.Pose(1:3,1:3) = rot(ez, x_true(3));
    
    % update the true state history
    x_true_hist = [x_true_hist, x_true];
    
    % state estimation using extended kalman filter
    [x_est, x_est_hist, P] = ekf(x_est, x_est_hist, x_true_hist, u, Q, P, R, ts);
    
    % get map points using lidar
    new_points = get_map_points(robot, colobj, x_est, M, R_lidar);
    map_points = [map_points; new_points];
    
    % update true simulation frame
    delete(sim1.CurrentAxes.Children(1));
    [~,patchObj] = show(robot);
    patchObj.FaceColor = [1 0 0];
    patchObj.EdgeColor = 'none';
    drawnow;
end

plot_analysis(x_est_hist, x_true_hist, map_points, ts, 2);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% purpose: make plots for analysis
%  input:   x_est_hist = history of state estimates
%          x_true_hist = history of true states
%           map_points = map points from lidar and estimated state
%                   ts = sample time
%              fig_num = number of figure
% output:
%--------------------------------------------------------------------------
function [] = plot_analysis(x_est_hist, x_true_hist, map_points, ts, fig_num)
figure(fig_num);
t = ts * (0:size(x_true_hist,2) - 1);

subplot(1,3,1);
plot(t, x_true_hist(1,:), 'g', 'LineWidth', 3);
hold on;
plot(t, x_est_hist(1,:), 'r', 'LineWidth', 3);
hold off;
xlabel('time');
ylabel('x position');
title('x position vs time');
legend('true', 'estimate');

subplot(1,3,2);
plot(t, x_true_hist(2,:), 'g', 'LineWidth', 3);
hold on;
plot(t, x_est_hist(2,:), 'r', 'LineWidth', 3);
hold off;
xlabel('time');
ylabel('y position');
title('y position vs time');
legend('true', 'estimate');

subplot(1,3,3);
plot(t, x_true_hist(3,:), 'g', 'LineWidth', 3);
hold on;
plot(t, x_est_hist(3,:), 'r', 'LineWidth', 3);
hold off;
xlabel('time');
ylabel('theta');
title('theta vs time');
legend('true', 'estimate');

figure(fig_num + 1);
clf(fig_num + 1);
plot(map_points(:,1), map_points(:,2), 'kx', 'MarkerSize', 10);
hold on;
plot(x_true_hist(1,:), x_true_hist(2,:), 'g', 'LineWidth', 3);
plot(x_est_hist(1,:), x_est_hist(2,:), 'r', 'LineWidth', 3);
plot(x_est_hist(4:3:end,:)', x_est_hist(5:3:end,:)', 'ro', 'MarkerSize', 10);
plot(x_true_hist(4:3:end,:), x_true_hist(5:3:end,:), 'g+', 'MarkerSize', 10, 'LineWidth', 2);
hold off;
grid on;
view(-90,90);
axis([-1 11 -1 11]);
xlabel('X');
ylabel('Y');
title('environment map and robot path');
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% purpose: display simulation
%   input:    robot = collision box representing mobile robot
%            colobj = cell array of collision boxes for environment
%           fig_num = number of figure
%  output:      sim = figure handle for simulation
%--------------------------------------------------------------------------
function [sim] = display_simulation(robot, colobj, fig_num)
sim = figure(fig_num);
clf(sim);

% display walls
col = rand(1,3);
[~,patchObj] = show(colobj{1});
patchObj.FaceColor = col;
patchObj.EdgeColor = 'none';
hold on;
for ii=2:4
    [~, patchObj] = show(colobj{ii});
    patchObj.FaceColor = col;
    patchObj.EdgeColor = 'none';
end

% display desks
[~,patchObj] = show(colobj{5});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{6});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';

% display stools
[~,patchObj] = show(colobj{7});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{8});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{9});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';

% display other obstacles
[~,patchObj] = show(colobj{10});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{11});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{12});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{13});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{14});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';

% display robot
[~,patchObj] = show(robot);
patchObj.FaceColor = [1 0 0];
patchObj.EdgeColor = 'none';

sim.CurrentAxes.Children(end).Visible = 'off';

view(-90,90);
axis([-1 11 -1 11 0 4]);

rotate3d off;
zoom off;
brush off;
datacursormode off;
pan off;
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% purpose: create collision box representing mobile robot
%   input:    q0 = initial configuration
%  output: robot = collision box representing mobile robot
%--------------------------------------------------------------------------
function [robot] = setup_robot(q0)
% z-axis
ez = [0; 0; 1];

% robot size
robot = collisionBox(0.4, 0.2, 0.2);

% robot position
robot.Pose(1:2, 4) = q0(1:2);
robot.Pose(3, 4) = 0.1;

% robot orientation
robot.Pose(1:3, 1:3) = rot(ez, q0(3));
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% purpose: setup environment collision boxes
%   input:
%  output: colobj = cell array of collision boxes for environment
%--------------------------------------------------------------------------
function [colobj] = setup_environment()
ez=[0;0;1];

% walls
wL=10;wW=.5;wH=4;
colobj{1}=collisionBox(wL,wW,wH);
colobj{1}.Pose(1:3,4)=[wL/2;-wW/2;wH/2];
colobj{2}=collisionBox(wL,wW,wH);
colobj{2}.Pose(1:3,4)=[wL/2;wL+wW/2;wH/2];
colobj{3}=collisionBox(wL,wW,wH);
colobj{3}.Pose(1:3,1:3)=rot(ez,pi/2);
colobj{3}.Pose(1:3,4)=[-wW/2;wL/2;wH/2];
colobj{4}=collisionBox(wL,wW,wH);
colobj{4}.Pose(1:3,1:3)=rot(ez,pi/2);
colobj{4}.Pose(1:3,4)=[wL+wW/2;wL/2;wH/2];

% desks
dL=1;dW=2;dH=.9;
desk1loc=[5;5];desk2loc=[2;8];
colobj{5}=collisionBox(dL,dW,dH);
colobj{6}=collisionBox(dL,dW,dH);
colobj{5}.Pose(1:3,4)=[desk1loc;dH/2];
colobj{6}.Pose(1:3,4)=[desk2loc;dH/2];
colobj{6}.Pose(1:3,1:3)=rot(ez,pi/2);

% stools
sR=.25;sL=.4;
colobj{7}=collisionCylinder(sR,sL);
colobj{8}=collisionCylinder(sR,sL);
colobj{9}=collisionCylinder(sR,sL);
colobj{7}.Pose(1:3,4)=[4.2;4.2;sL/2];
colobj{8}.Pose(1:3,4)=[4.2;5.8;sL/2];
colobj{9}.Pose(1:3,4)=[2;7;sL/2];

% other obstacles
colobj{10}=collisionBox(1,2,2);
colobj{11}=collisionBox(1,2,2);
colobj{12}=collisionBox(2,1,2);
colobj{13}=collisionBox(2,1.5,2);
colobj{14}=collisionCylinder(.5,1.5);
colobj{10}.Pose(1:3,4)=[9.5;8;1];
colobj{11}.Pose(1:3,4)=[9.5;1;1];
colobj{12}.Pose(1:3,4)=[4;.5;1];
colobj{13}.Pose(1:3,4)=[6;.5;1];
colobj{14}.Pose(1:3,4)=[8;5;.75];
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% purpose: get pressed key
%--------------------------------------------------------------------------
function get_key(~, event)
assignin('base', 'key', event.Key);
assignin('base', 'key_pressed', 1);
end
%--------------------------------------------------------------------------