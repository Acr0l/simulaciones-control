import os
import time

import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np
from lib.voltage import apply_motor_voltage

# ---------------------------------- CONTROL ---------------------------------

Kp_pos = 0.32846  # Proportional gain for pos
Ki_pos = 0.010445   # Integral gain for pos
Kd_pos = 2.5823  # Derivative gain for pos

Kp_motor = 10.501
Ki_motor = 3.1389
Kd_motor = 5.2948
# Initialize PID variables
prev_error_pos = 0
integral_pos = 0

prev_error_motor = 0
integral_motor = 0

pos_setpoint = .5
angle_setpoint = 0
# ------------------------------------ GRAPH ---------------------------------
# Initialize lists to store data for plotting
time_list = []

pos_list = []
pos_sp_list = []

theta_list = []
theta_sp_list = []


def pid_ball_control(current_pos, dt):
    global prev_error_pos, integral_pos

    global pos_setpoint, angle_setpoint

    # Error calculation for the ball position
    error_pos = pos_setpoint - current_pos

    # PID terms for angle control
    proportional = Kp_pos * error_pos
    integral_pos += error_pos * dt
    derivative = (error_pos - prev_error_pos) / dt
    prev_error_pos = error_pos

    # PID output (control signal: desired voltage to apply to the motor)
    control_signal = proportional + \
        (Ki_pos * integral_pos) + (Kd_pos * derivative)

    angle_setpoint = control_signal
    # Collect data for plotting
    time_list.append(time.time() - start)
    pos_list.append(current_pos)
    pos_sp_list.append(pos_setpoint)

# def pid_torque_control(current_torque, dt, o_voltage):


def pid_motor_control(current_angle, dt, o_voltage):
    global prev_error_motor, integral_motor, angle_setpoint

    # Error calculation for the cart position
    error_motor = angle_setpoint - current_angle

    # PID terms for motor control
    proportional = Kp_motor * error_motor
    integral_motor += error_motor * dt if dt > 0 else 0
    derivative = (error_motor - prev_error_motor) / dt if dt > 0 else 0
    prev_error_motor = error_motor

    # PID output (control signal: desired voltage to apply to the motor)
    control_signal = proportional + \
        (Ki_motor * integral_motor) + (Kd_motor * derivative)

    o_voltage(control_signal)

    # Collect data
    theta_list.append(current_angle)
    theta_sp_list.append(angle_setpoint)


# Get current file path
current_path = os.path.dirname(os.path.abspath(__file__))

# Model's path
model_path = os.path.join(
    current_path, "../models/pole_beam/pole_beam.xml")

# Crear el modelo y la información
m = mujoco.MjModel.from_xml_path(model_path)
d = mujoco.MjData(m)

print(m.body("ball").inertia)
with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 180:
        step_start = time.time()
        dt = m.opt.timestep

        # Control de la simulación
        current_theta = -d.sensor("theta").data.copy() % (2 * np.pi)
        current_alpha = np.pi / 2 - d.sensor("alpha").data.copy()
        current_pos = .2 - d.joint("ball_joint").qpos[0]

        current_torque = d.ctrl[0].copy()
        pid_ball_control(current_pos, dt)
        pid_motor_control(current_torque, dt,
                          lambda x: apply_motor_voltage(m, d, 0, x, [0, m.opt.timestep]))

        # Avanzar la simulación "un paso"
        mujoco.mj_step(m, d)

        # Actualizar la vista
        viewer.sync()

        # Esperar hasta el siguiente paso (para mantener la velocidad de la simulación)
        if dt > 0:
            time.sleep(dt)

# Plot the data after the simulation ends
plt.figure(figsize=(12, 8))

# Plot for ball control
plt.subplot(2, 1, 1)
plt.plot(time_list, pos_list, label="Ball position")
plt.plot(time_list, pos_sp_list, label="Ball setpoint")
plt.ylabel("Ball position")
plt.xlabel("Time (s)")
plt.legend()

# Plot for motor control
plt.subplot(2, 1, 2)
plt.plot(time_list, theta_list, label="Motor angle")
plt.plot(time_list, theta_sp_list, label="Motor setpoint")
plt.ylabel("Motor angle")
plt.xlabel("Time (s)")
plt.ylim(-np.pi, np.pi)
plt.legend()

plt.show()
