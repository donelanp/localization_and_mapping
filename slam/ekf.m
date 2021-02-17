%--------------------------------------------------------------------------
% purpose: estimate state via extended kalman filter
%   input:         x_est = previous state estimate
%             x_est_hist = history of state estimates
%            x_true_hist = history of true states
%                      u = control input [linear velocity; angular velocity]
%                      Q = process noise covariance matrix
%                      P = state covariance matrix
%                      R = observation noise covariance matrix
%                     ts = sample time
%  output:         x_est = new state estimate
%             x_est_hist = updated history of state estimates
%                      P = new state covariance matrix
%--------------------------------------------------------------------------

function [x_est, x_est_hist, P] = ekf(x_est, x_est_hist, x_true_hist, u, Q, P, R, ts)
% rotation axis
ez = [0; 0; 1];

% kalman filter prediction step
[x_pred, F] = state_transition(x_est, u, Q, ts);
P = F * P * F' + Q;

% get measurements at true state
[z_true, ~, used_sensors_true] = state_observation(x_true_hist, R);

% get measurements at predicted state
x_est_hist = [x_est_hist, x_pred];
[z_pred, H, used_sensors_pred] = state_observation(x_est_hist, R);

% innovation and innovation covariance
% only compare for common measurements
[used_sensors_common, ind_pred, ind_true] = intersect(used_sensors_pred, used_sensors_true);
H = H(ind_pred, :);
y = z_true(ind_true) - z_pred(ind_pred);
S = H * P * H' + R(used_sensors_common, used_sensors_common);

% kalman gain
K = P * H' / S;

% update state and covariance estimate
x_est = x_pred + K * y;
P = (eye(size(P)) - K * H) * P;
x_est_hist(:,end) = x_est;
end