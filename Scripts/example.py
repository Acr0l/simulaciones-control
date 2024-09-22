import os
import time

import mujoco
import mujoco.viewer
from lib.voltage import apply_motor_voltage

# Obtener la ruta actual del archivo
current_path = os.path.dirname(os.path.abspath(__file__))

# Ruta al modelo
# Resto del código...rear el modelo e información
model_path = os.path.join(
    current_path, "../models/pole_beam/pole_beam.xml")

# Crear el modelo y la información
m = mujoco.MjModel.from_xml_path(model_path)
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    # Correr la simulación por 30 segundos
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()
        # Control de la simulación
        apply_motor_voltage(m, d, 0, .2)

        # Avanzar la simulación "un paso"
        mujoco.mj_step(m, d)

        # Actualizar la vista
        viewer.sync()

        # Esperar hasta el siguiente paso (para mantener la velocidad de la simulación)
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
