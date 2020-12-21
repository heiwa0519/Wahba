function [C_opt, q_opt] = foam1993(v_b,v_i,w)
%%% Fast Optimal Matrix Algorithm (FOAM)
% 
% Inputs:
%   v_b = Unit measurement vectors in the spacecraft body frame (3xn).
%   v_i = Corresponding unit vectors known in the inertial frame (3xn).
%   w = Non negative weight assigned to each observations (nx1). 
%       Markley1993 describes two ways of defining weight: 
%       1) Equal to the reciprocal of the variance of the measurement.
%       2) Nomalized such that their sum is 1.
%
% Outputs:
%   q_opt = Optimal quaternion that transforms vectors in inertial frame to 
%           vectors in body frame ([q0,q1,q2,q3]').
%   C_opt = Optimal rotation matrix that transforms vectors in inertial
%           frame to vectors in body frame.   
%
% References:  
%   [1] F. Landis Markley - Attitude Determination using Vector 
%       Observations, A Fast Optimal Matrix Algorithm (1993)
% 
% Rishav (2020/11/8)

% Newton-Raphson method tolerance
tolerance = 10e-5;

% Attitude profile matrix (Eqn 4)
B = (v_b.*repmat(w,[1,3])')*v_i';

det_B = det(B);
adj_B = det(B)*inv(B);
norm_B = norm(B, 'fro');
norm_adj_B = norm(adj_B, 'fro');

% Characteristic equation for Newton-Raphson method: (Eqn 23)
%  f(lambda) = (lambda^2 - ||B||^2)^2 - 8*lambda*det_B - 4*||adj_B||^2 = 0 
lambda = sum(w); % Shuster1981 and markley1993
last_lambda = 0.0;
while abs(lambda - last_lambda) >= tolerance
    last_lambda = lambda;
    
    f = (lambda^2 - norm_B^2)^2 - 8*lambda*det_B - 4*norm_adj_B^2;
    f_dot = 4*lambda*(lambda^2 - norm_B^2) - 8*det_B;
    lambda = lambda - f/f_dot;
end
kappa = 0.5*(lambda^2 - norm_B^2); % Eqn 18
zeta = kappa*lambda - det_B; % Eqn 19

% Determine optimal rotation matrix (Eqn 14)
C_opt = ((kappa + norm_B^2)*B + lambda*adj_B' - B*(B')*B)/zeta;
q_opt = dcm2Quaternion(C_opt);
end
