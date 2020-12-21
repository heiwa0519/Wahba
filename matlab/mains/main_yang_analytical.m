% 2020/12/20
clc
clear
close all
format long

% Two unit vectors known in inertial frame
v_i = [ 0.2673, 0.5345, 0.8018; ... 
      -0.3124, 0.9370, 0.1562]';

% Attitude matrix (v_b = C*v_i)
C_truth = [ 0.5335, 0.8080, 0.2500; ... 
            -0.8080, 0.3995, 0.4330;
            0.2500, -0.4330, 0.8660];

SNR = 80; % Measurement noise    
v_b = awgn(C_truth*v_i, SNR);
w = 1./var(v_b)'; % Measurement weights
[C_estimate, q_estimate] = yang_analytical2013(v_b, v_i, w);
 
fprintf('C_truth \n'); disp(C_truth);
fprintf('C_estimate \n'); disp(C_estimate);
fprintf('q_estimate \n'); disp(q_estimate);
