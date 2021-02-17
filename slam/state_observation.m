%--------------------------------------------------------------------------
% purpose: get various observations relating to robot
%   input:       x_hist = full state history
%                     R = observation noise covariance matrix
%  output:            z = current observations
%                     H = observation matrix evaluated at robot's state  
%          used_sensors = indices of used sensors
%--------------------------------------------------------------------------
function [z, H, used_sensors] = state_observation(x_hist, R)
% gps sensor range
gps_range = 10;

% keep track of used sensors
used_sensors = false(size(R,1),1);

% current state
x = x_hist(:,end);

% robot position
pos_rob = [x(1:2); 0.1];

z = [];
H = [];

%--------------------------------------------------------------------------
% local GPS
%--------------------------------------------------------------------------
corners = reshape(x(4:15), [3,4]);
z_gps = vecnorm(corners - pos_rob)';
H_gps = zeros(4, numel(x_hist(:,1)));
H_gps(:,1:3) = [-(corners(1:2,:) - pos_rob(1:2))' ./ z_gps, zeros(4,1)];
H_gps(1,4:6) = (corners(1:3,1) - pos_rob(1:3))' ./ z_gps(1);
H_gps(2,7:9) = (corners(1:3,2) - pos_rob(1:3))' ./ z_gps(2);
H_gps(3,10:12) = (corners(1:3,3) - pos_rob(1:3))' ./ z_gps(3);
H_gps(4,13:15) = (corners(1:3,4) - pos_rob(1:3))' ./ z_gps(4);

% determine which sensors are in range
in_range = z_gps <= gps_range;
z_gps(~in_range) = [];
H_gps(~in_range,:) = [];
used_sensors(0 + find(in_range)) = 1;

% append to output
z = [z; z_gps];
H = [H; H_gps];
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% bearing sensor
%--------------------------------------------------------------------------
% big cylinder object
p = x_hist(16:18,:);
delta = p(1:2) - pos_rob(1:2);
x = delta(1)^2 + delta(2)^2;
z_bearing = atan2(delta(2), delta(1)) - x_hist(3,end);
H_bearing = zeros(1, numel(x_hist(:,1)));
H_bearing(1:3) = [-delta(2) / x, delta(1) / x, -1];
H_bearing(16:18) = [delta(2) / x, -delta(1) / x, 0];
z = [z; z_bearing];
H = [H; H_bearing];
used_sensors(5) = 1;
%--------------------------------------------------------------------------


% add noise to observations
v = normrnd(zeros(size(z)), diag(R(used_sensors,used_sensors)));
z = z + v;

% indices of used sensors
used_sensors = find(used_sensors);
end
%--------------------------------------------------------------------------