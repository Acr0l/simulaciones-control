% Motor parameters
% dt is 0.02 s

R = 1.0;       % Resistance (Ohms)
L = 0.5;       % Inductance (Henries)
k_e = 0.01;    % Back-EMF constant (V/(rad/s))
k_t = 0.1;     % Torque constant (Nm/A)

% Define the transfer function for current in response to applied voltage
numerator = [1];
denominator = [L, R];
current_tf = tf(numerator, denominator);

% Define torque as output using current_tf and torque constant
torque_tf = k_t * current_tf;

pidTuner(torque_tf, 'pid')
