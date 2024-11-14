% Cart parameters
m = .7; % mass of the cart (kg)
b = 0.0001; % damping coefficient (Ns/m)

% Position transfer function (X(s)/T(s))
numerator = [1];
denominator = [m, b, 0];
position_tf = tf(numerator, denominator);

% Open PID tuner for position control
pidTuner(position_tf, 'PID');
