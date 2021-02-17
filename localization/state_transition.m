%--------------------------------------------------------------------------
% purpose: update the state of the mobile robot
%   input:      q = current state of mobile robot [x; y; theta] where theta is
%                   orientation about the z-axis
%               u = control input [linear velocity; angular velocity]
%               Q = process noise covariance matrix
%              ts = sample time
%  output: q_next = next state of mobile robot
%               F = state transition matrix evaluated at (q,u)
%--------------------------------------------------------------------------
function [q_next, F] = state_transition(q, u, Q, ts)
% orientation of robot
theta = q(3);

% model uncertainty
w = normrnd(zeros(size(q)), diag(Q));

% kinematic model
f = [cos(theta) 0; sin(theta) 0; 0 1];
qdot = f * u + w;

% next state
q_next = q + qdot * ts;

% state transition matrix
F = [1 0 -sin(theta) * u(1) * ts; 0 1 cos(theta) * u(1) * ts; 0 0 1];
end
%--------------------------------------------------------------------------