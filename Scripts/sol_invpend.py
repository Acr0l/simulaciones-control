import os
import time

import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
from lib.voltage import apply_motor_voltage

# ---------------------------------- CONTROL ---------------------------------
Kp_angle = 505.6  # Proportional gain for angle
Ki_angle = 2935   # Integral gain for angle
Kd_angle = 21.78    # Derivative gain for angle

Kp_motor = 14.3156
Ki_motor = 51.8962
Kd_motor = 0.00036261

# Setpoints
angle_setpoint = 0
motor_setpoint = 0
pos_setpoint = 0

# Initialize PID variables
prev_error_angle = 0
integral_angle = 0

prev_error_motor = 0
integral_motor = 0

# Data collection lists
time_data = []
angle_setpoint_data = []
current_angle_data = []
motor_setpoint_data = []
current_torque_data = []
voltage_data = []


def pid_motor_control(current_torque, dt, o_voltage):
    global prev_error_motor, integral_motor, motor_setpoint

    # Error calculation for the cart position
    error_motor = motor_setpoint - current_torque

    # PID terms for motor control
    proportional = Kp_motor * error_motor
    integral_motor += error_motor * dt
    derivative = (error_motor - prev_error_motor) / dt
    prev_error_motor = error_motor

    # PID output (control signal: desired voltage to apply to the motor)
    control_signal = proportional + \
        (Ki_motor * integral_motor) + (Kd_motor * derivative)

    applied_voltage = o_voltage(control_signal)
    applied_voltage = applied_voltage if isinstance(
        applied_voltage, (int, float)) else applied_voltage[0]

    # Collect data
    voltage_data.append(applied_voltage)
    motor_setpoint_data.append(motor_setpoint)
    current_torque_data.append(current_torque)


def pid_angle_control(current_angle, current_pos, dt):
    global prev_error_angle, integral_angle, motor_setpoint, angle_setpoint

    # Error calculation for the pendulum angle
    error_angle = angle_setpoint - (current_angle)
    # PID terms for angle control
    proportional = Kp_angle * error_angle
    integral_angle += error_angle * dt
    derivative = (error_angle - prev_error_angle) / dt
    prev_error_angle = error_angle

    # PID output (control signal: desired voltage to apply to the motor)
    control_signal = proportional + \
        (Ki_angle * integral_angle) + (Kd_angle * derivative)

    # Apply the control signal to the motor (adjust voltage)
    motor_setpoint = control_signal

    # Collect data
    angle_setpoint_data.append(angle_setpoint)
    current_angle_data.append(current_angle)

# Get current file path
current_path = os.path.dirname(os.path.abspath(__file__))

# Model's path
model_path = os.path.join(
    current_path, "../models/inverted_pendulum/inverted_pend_model.xml")

# Create model and data objects (the MuJoCo storage structures)
m = mujoco.MjModel.from_xml_path(model_path)
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    # Run the simulation for 30 seconds
    while viewer.is_running() and time.time() - start < 180:
        step_start = time.time()
        dt = m.opt.timestep - (time.time() - step_start)

        # Control
        current_angle = d.sensor("theta").data.copy()
        current_pos = d.joint("car_joint").qpos[0]

        pid_angle_control(current_angle, current_pos, dt)

        current_torque = d.ctrl[0].copy()
        pid_motor_control(current_torque, dt, lambda voltage: apply_motor_voltage(
            m, d, 0, voltage, [0, m.opt.timestep]))

        # Collect time data
        time_data.append(time.time() - start)

        # Advance the simulation "one step"
        mujoco.mj_step(m, d)

        # Update view
        viewer.sync()

        # Wait until the next step (to maintain the simulation speed)
        if dt > 0:
            time.sleep(dt)

# Plotting the results
plt.figure(figsize=(12, 6))

# Plot for angle control
plt.subplot(2, 2, 1)
plt.plot(time_data, angle_setpoint_data, label='Angle Setpoint')
plt.plot(time_data, current_angle_data, label='Current Angle')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Angle Control')
plt.legend()

# Plot for motor control
plt.subplot(2, 2, 2)
plt.plot(time_data, motor_setpoint_data, label='Motor Setpoint')
plt.plot(time_data, current_torque_data, label='Current Torque')
plt.xlabel('Time (s)')
plt.ylabel('Torque (Nm)')
plt.title('Motor Control')
plt.legend()

plt.subplot(2, 2, 3)
plt.plot(time_data, voltage_data, label='Motor Voltage')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.title('Motor Voltage')
plt.legend()

plt.tight_layout()
plt.show()
