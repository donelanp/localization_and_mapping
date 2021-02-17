clear all; close all;

%--------------------------------------------------------------------------
% setup simulation
%--------------------------------------------------------------------------
% axes
ez = [0; 0; 1];

% number of lidar scan directions
M = 16;

% initial position covariance matrix
P = 0.1 * eye(3);

% process noise covariance matrix
Q = 0.01 * eye(3);

% observation noise covariance matrix
R = 0.01 * eye(8);
R = blkdiag(R, 0.01 * eye(M));

% true initial state
q_true = [0.2; 2; 0];

% initial state estimate
q0 = normrnd(q_true, diag(P));

% obstacles
colobj = setup_environment();

% mobile robot
robot = setup_robot(q_true);

% show simulation
sim = display_simulation(robot, colobj);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% control robot
%--------------------------------------------------------------------------
% sample time
ts = 1;

% initial state estimate
q_est = q0;

% history of true and estimated states
q_true_hist = q_true;
q_est_hist = q_est;

% let figure update
pause(1);

key = ' ';
key_pressed = 0;
set(sim,'KeyPressFcn',@get_key);

% exit keyboard control by pressing 'q'
while key ~= 'q'
    % wait until key pressed
    while key_pressed == 0
        pause(.05);
    end
    key_pressed = 0;
    
    % determine input velocities
    u = [0; 0];
    s = 0.1;
    switch key
        case 'w'
            % forward
            u(1) = s;
        case 's'
            % backward
            u(1) = -s;
        case 'a'
            % rotate counter-clockwise
            u(2) = s;
        case 'd'
            % rotate clockwise
            u(2) = -s;
    end
    
    % update the true state of the robot
    [q_true, ~] = state_transition(q_true, u, Q, ts);
    q_true_hist = [q_true_hist, q_true];
    
    % kalman filter prediction step
    [q_pred, F] = state_transition(q_est, u, Q, ts);
    P = F * P * F' + Q;    
    
    % get measurements at predicted state
    robot.Pose(1:2, 4) = q_pred(1:2);
    robot.Pose(1:3, 1:3) = rot(ez, q_pred(3));
    q_est_hist = [q_est_hist, q_pred];
    [z_pred, H, used_sensors_pred] = state_observation(q_est_hist, R, robot, colobj, M, ts);
    
    % get measurements at true state
    robot.Pose(1:2, 4) = q_true(1:2);
    robot.Pose(1:3, 1:3) = rot(ez, q_true(3));
    [z_true, ~, used_sensors_true] = state_observation(q_true_hist, R, robot, colobj, M, ts);
    
    % innovation and innovation covariance
    % only compare for common measurements
    [used_sensors_common, ind_pred, ind_true] = intersect(used_sensors_pred, used_sensors_true);
    H = H(ind_pred, :);
    y = z_true(ind_true) - z_pred(ind_pred);
    S = H * P * H' + R(used_sensors_common, used_sensors_common);
    
    % kalman gain
    K = P * H' / S;
    
    % update state and covariance estimate
    q_est = q_pred + K * y;
    P = (eye(3) - K * H) * P;
    q_est_hist(:,end) = q_est;
    
    % update simulation frame
    delete(sim.CurrentAxes.Children(1));
    [~,patchObj] = show(robot);
    patchObj.FaceColor = [218,165,32]/218;
    patchObj.EdgeColor = 'none';
end

% plot results
figure(3);
t = ts * (0:size(q_true_hist,2) - 1);

subplot(1,3,1);
plot(t, q_true_hist(1,:), 'g', 'LineWidth', 3);
hold on;
plot(t, q_est_hist(1,:), 'r', 'LineWidth', 3);
hold off;
xlabel('time');
ylabel('x position');
title('x position vs time');
legend('true', 'estimate');

subplot(1,3,2);
plot(t, q_true_hist(2,:), 'g', 'LineWidth', 3);
hold on;
plot(t, q_est_hist(2,:), 'r', 'LineWidth', 3);
hold off;
xlabel('time');
ylabel('y position');
title('y position vs time');
legend('true', 'estimate');

subplot(1,3,3);
plot(t, q_true_hist(3,:), 'g', 'LineWidth', 3);
hold on;
plot(t, q_est_hist(3,:), 'r', 'LineWidth', 3);
hold off;
xlabel('time');
ylabel('theta');
title('theta vs time');
legend('true', 'estimate');
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% purpose: display simulation
%   input:  robot = collision box representing mobile robot
%          colobj = cell array of collision boxes for environment
%  output:    sim = figure handle for simulation
%--------------------------------------------------------------------------
function [sim] = display_simulation(robot, colobj)
sim = figure(2);
clf(sim);

% display walls
show(colobj{1});
hold on;
for ii=2:4
    show(colobj{ii});
end

% display desks
[~,patchObj] = show(colobj{5});
patchObj.FaceColor = [.8 0 1];
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{6});
patchObj.FaceColor = [.8 0 1];
patchObj.EdgeColor = 'none';

% display stools
[~,patchObj] = show(colobj{7});
patchObj.FaceColor = [0 1 1];
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{8});
patchObj.FaceColor = [0 1 1];
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{9});
patchObj.FaceColor = [0 1 1];
patchObj.EdgeColor = 'none';

% display other obstacles
[~,patchObj] = show(colobj{10});
patchObj.FaceColor = [1 .8 .8];
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{11});
patchObj.FaceColor = [1 .8 .8];
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{12});
patchObj.FaceColor = [1 .8 .8];
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{13});
patchObj.FaceColor = [1 .8 .8];
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{14});
patchObj.FaceColor = [.5 1 .5];
patchObj.EdgeColor = 'none';

% display robot
[~,patchObj] = show(robot);
patchObj.FaceColor = [218,165,32]/218;
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