%--------------------------------------------------------------------------
% purpose: get map points using lidar measurements and estimated state
%   input:      robot = collision box representing mobile robot
%              colobj = cell array of collision boxes for environment
%               x_est = estimated state
%                   M = number of lidar directions
%                   R = lidar noise covariance
%  output: map_points = 2D map points from lidar measurements
%--------------------------------------------------------------------------
function [map_points] = get_map_points(robot, colobj, x_est, M, R)
% rotation axes
ey = [0; 1; 0];
ez = [0; 0; 1];

% lidar range
range = 8;

% lidar measurements
z = zeros(M, 1);

% scan directions
ang = 2 * pi * (0:M-1)' / M;

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
    
    z(ii) = ray_lower + ray_diff / 2;
end

% add noise
z = normrnd(z, diag(R));

% create map points
map_points = x_est(1:2)' + z .* [cos(ang) sin(ang)];

% remove points that are out of range
in_range = z <= range;
map_points(~in_range,:) = [];
end