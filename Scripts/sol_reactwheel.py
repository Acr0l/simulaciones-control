import os
import time

import mujoco
import mujoco.viewer
from lib.voltage import apply_motor_voltage

# PID Controller Parameters
Kp = -450.2637  # Proportional gain
Ki = -1313.4007   # Integral gain
Kd = -22.6168   # Derivative gain
# Kp = -949.0983  # Proportional gain
# Ki = -4747.6256   # Integral gain
# Kd = -27.8704   # Derivative gain

# PID State Variables
pid_state = {
    'integral': 0.0,
    'previous_error': 0.0
}

# Desired setpoint (center position)
setpoint = 0.0  # Assuming center position corresponds to 0 radians


def apply_pid_control(model, data, actuator_id, setpoint, dt, pid_state):
    """
    Apply PID control to the actuator.

    :param model: MuJoCo model
    :param data: MuJoCo data
    :param actuator_id: ID of the actuator to control
    :param setpoint: Desired position
    :param dt: Time step
    :param pid_state: Dictionary containing 'integral' and 'previous_error'
    """
    # Get current position and velocity
    current_pos = data.joint("axis").qpos
    current_vel = data.joint("axis").qvel

    # Calculate error
    error = setpoint - current_pos

    # Update integral with anti-windup
    pid_state['integral'] += error * dt

    # Calculate derivative
    derivative = (error - pid_state['previous_error']) / dt if dt > 0 else 0.0

    # PID output
    control_signal = Kp * error + Ki * pid_state['integral'] + Kd * derivative

    # Optional: Scale control signal to voltage limits
    max_voltage = 5.0 * 10**10
    min_voltage = -5.0 * 10**10
    voltage = max(min(control_signal, max_voltage), min_voltage)

    # Update previous error
    pid_state['previous_error'] = error

    # Apply voltage using the existing function
    apply_motor_voltage(model, data, actuator_id, voltage, [0, m.opt.timestep])


# Get route
current_path = os.path.dirname(os.path.abspath(__file__))

# Model path
model_path = os.path.join(
    # Ensure correct file extension
    current_path, "../models/reaction_wheel/reaction_wheel_model.xml")

# Create model and data
m = mujoco.MjModel.from_xml_path(model_path)
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    last_time = start
    d.ctrl[0] = .01
    # One step of the simulation
    mujoco.mj_step(m, d)

    # Updaate the view
    viewer.sync()
    # Run sim for 30 seconds.
    print(m.body('pend_arm'))
    print(m.body('wheel'))
    while viewer.is_running() and time.time() - start < 30:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Control using PID
        apply_pid_control(m, d, actuator_id=0,
                          setpoint=setpoint, dt=dt, pid_state=pid_state)

        # One step of the simulation
        mujoco.mj_step(m, d)

        # Updaate the view
        viewer.sync()

        # Wait until the next step (to maintain simulation speed)
        time_until_next_step = m.opt.timestep - (time.time() - current_time)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
