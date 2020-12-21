% 2020/11/4
clc
clear
close all
format long

% Two unit vectors in body frame (measurement)
r1 = [ 0.8273, 0.5541, -0.0920]';
r2 = [ -0.8285, -0.5522, -0.0955]';

% Attitude matrix (v_b = C*v_i)
C_truth = [ 0.5335, 0.8080, 0.2500; ... 
            -0.8080, 0.3995, 0.4330;
            0.2500, -0.4330, 0.8660];

SNR = 80; % Measurement noise
b1 = awgn(C_truth*r1, SNR);
b2 = awgn(C_truth*r2, SNR);
[C_estimate, q_estimate] = triad1964(b1, b2, r1, r2);

fprintf('C_truth \n'); disp(C_truth);
fprintf('C_estimate \n'); disp(C_estimate);
fprintf('q_estimate \n'); disp(q_estimate);
