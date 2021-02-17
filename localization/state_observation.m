%--------------------------------------------------------------------------
% purpose: get various observations relating to robot
%   input:       q_hist = full state history
%                     R = observation noise covariance matrix
%                 robot = collision box representing robot
%                colobj = cell aray of collision boxes for environment
%                     M = number of scan directions
%                    ts = sample time
%  output:            z = current observations
%                     H = observation matrix evaluated at robot's state  
%          used_sensors = indices of used sensors
%--------------------------------------------------------------------------
function [z, H, used_sensors] = state_observation(q_hist, R, robot, colobj, M, ts)
% axes
ey = [0; 1; 0];
ez = [0; 0; 1];

% location of upper room corners
corners = [0 0 10 10; 0 10 10 0; 4 4 4 4];

% proximity sensor range
prox_range = 1;

% gps sensor range
gps_range = 10;

% lidar sensor range
lidar_range = 8;

% keep track of used sensors
used_sensors = false(size(R,1),1);

z = [];
H = [];

%--------------------------------------------------------------------------
% proximity sensor
%--------------------------------------------------------------------------
z_prox = [];
H_prox = [];
for ii=1:length(colobj)
    % get distance from robot to each object in environment
    [~, dist, pts] = checkCollision(robot,colobj{ii});
    
    if ~isnan(dist)
        % no collision, object in sensor range, and distance is closest
        % proximity
        if dist <= prox_range && (isempty(z_prox) || (~isempty(z_prox) && dist < z_prox))
            z_prox = dist;
            H_prox = [-(pts(1:2,2) - pts(1:2,1))' / z_prox, 0];
            used_sensors(1) = 1;
        end
    end
end

% append to output
z = [z; z_prox];
H = [H; H_prox];
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% IMU acceleration
%--------------------------------------------------------------------------
z_acc = [];
H_acc = [];

if size(q_hist,2) > 2
   delta = diff(diff(q_hist(1:2,end-2:end), 1, 2), 1, 2);
   z_acc = norm(delta) / ts^2;
   H_acc = [delta' / ts^2 / z_acc, 0];
   used_sensors(2) = 1;
end

% append to output
z = [z; z_acc];
H = [H; H_acc];
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% IMU angular velocity
%--------------------------------------------------------------------------
z_ang = [];
H_ang = [];

if size(q_hist,2) > 1
   z_ang = (q_hist(3,end)-q_hist(3,end-1)) / ts;
   H_ang = [0 0 1/ts];
   used_sensors(3) = 1;
end

% append to output
z = [z; z_ang];
H = [H; H_ang];
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% local GPS
%--------------------------------------------------------------------------
z_gps = vecnorm(corners - robot.Pose(1:3, 4))';
H_gps = [(robot.Pose(1:2,4) - corners(1:2,:))' ./ z_gps, zeros(4,1)];

% determine which sensors are in range
in_range = z_gps <= gps_range;
z_gps(~in_range) = [];
H_gps(~in_range,:) = [];
used_sensors(3 + find(in_range)) = 1;

% append to output
z = [z; z_gps];
H = [H; H_gps];
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% bearing sensor
%--------------------------------------------------------------------------
% big cylinder object
p = colobj{14}.Pose(1:3,4);
delta = p(1:2) - robot.Pose(1:2,4);
x = delta(1)^2 + delta(2)^2;
z_bearing = atan2(delta(2), delta(1)) - q_hist(3,end);
H_bearing = [-delta(2) / x, delta(1) / x, -1];
z = [z; z_bearing];
H = [H; H_bearing];
used_sensors(8) = 1;
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% lidar
%--------------------------------------------------------------------------
z_lidar = zeros(M,1);
H_lidar = zeros(M,3);

% scan directions
ang = 2 * pi * (0:M-1) / M;

% if difference between the upper and lower bound is less than epsilon, the
% bounds will be treated as the same
ray_epsilon = 0.1;

% cylinder representing ray
ray = collisionCylinder(0.001, 1);

% find distance for each scan angle
for ii=1:M
    % lower bound on the ray length
    ray_lower = 0;
    
    % upper bound on the ray length
    ray_upper = 10;
    
    ray_diff = ray_upper - ray_lower;
    
    % find ray length before a collision
    loop_count = 0;
    while (loop_count < 10) && (ray_diff > ray_epsilon)
        loop_count = loop_count + 1;
        
        % alter length
        ray.Length = ray_upper;
        
        % align ray with robot
        ray.Pose(1:3, 1:3) = rot(ez, ang(ii)) * rot(ey, pi/2) * robot.Pose(1:3, 1:3);
        ray.Pose(1:3, 4) = robot.Pose(1:3, 4) + ray_upper / 2 * [cos(ang(ii)); sin(ang(ii)); 0];
        
        % determine if there is a collision
        collision = false;
        for jj=1:length(colobj)
           collision = collision || checkCollision(ray,colobj{jj});
        end
        
        ray_diff = ray_upper - ray_lower;
        
        if collision
            % shrink the ray if there is a collision
            ray_upper = ray_upper - ray_diff / 2;
        else
            % grow the ray if there is not a collision
            ray_lower = ray_upper;
            ray_upper = ray_upper + ray_diff / 2;
        end
        
        ray_diff = ray_upper - ray_lower;
    end
    
    z_lidar(ii) = ray_lower + ray_diff / 2;
    
    % get witness points to nearest object
    ray.Length = z_lidar(ii);
    ray.Pose(1:3, 1:3) = rot(ez, ang(ii)) * rot(ey, pi/2) * robot.Pose(1:3, 1:3);
    ray.Pose(1:3, 4) = robot.Pose(1:3, 4) + z_lidar(ii) / 2 * [cos(ang(ii)); sin(ang(ii)); 0];
    
    min_dist = inf;
    p = zeros(3,1);
    for jj=1:length(colobj)
       [~, dist, pts] = checkCollision(ray, colobj{jj});
       if dist < min_dist
           min_dist = dist;
           p = pts(:,2);
       end
    end
    
    H_lidar(ii,:) = [(robot.Pose(1:2,4) - p(1:2))' / z_lidar(ii), 0];
end

% determine which sensors are in range
in_range = z_lidar <= lidar_range;
z_lidar(~in_range) = [];
H_lidar(~in_range,:) = [];
used_sensors(8 + find(in_range)) = 1;

% append to output
z = [z; z_lidar];
H = [H; H_lidar];
%--------------------------------------------------------------------------

% add noise to observations
v = normrnd(zeros(size(z)), diag(R(used_sensors,used_sensors)));
z = z + v;

% indices of used sensors
used_sensors = find(used_sensors);
end
%--------------------------------------------------------------------------