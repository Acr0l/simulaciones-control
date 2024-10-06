# Automatic Control Simulations in MuJoCo

This repository contains a set of simulated environments in MuJoCo, designed to help automatic control students experiment and test different algorithms and techniques in a digital space. The environments allow the simulation of dynamic and control systems that are difficult or expensive to recreate in real life.

## What is MuJoCo?

MuJoCo (Multi-Joint dynamics with Contact) is an advanced physics simulation engine that allows simulating mechanical systems with multiple bodies, contacts, and joints. It is widely used in robotics, biomechanics, and in the design of automatic control systems.

### Key Features of MuJoCo

- **Accurate and Fast Simulation**: MuJoCo is designed to perform real-time dynamic calculations efficiently.
- **Flexible Model Definitions**: You can create custom models, define constraints, friction, contact, and more.
- **Python Integration**: Simulations can be controlled through Python scripts, allowing for automated experiments and advanced analysis.

## Prerequisites

To use this repository, you will need:

- Python 3.x
- [MuJoCo](http://www.mujoco.org/) installed (Or just the python library)
- The `mujoco` library

Install the dependencies by running:

```bash
pip install mujoco
```

## How to Use the Repository

This repository contains a set of models ready to be simulated. You can load and run the simulations using Python. Below is a basic example of how to load a model and control the simulation:

### Example Code

```python

import mujoco
import mujoco.viewer
import os

# Load MuJoCo model (MJCF file)
model_path = "path/to/file.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model,data) as viewer:
    while viewer.is_running():

        # Control here

        mujoco.mj_step(model, data)

        viewer.sync()
```

This code initializes a MuJoCo model, creates a simulation, and runs a loop where you can observe the system’s evolution. You can also modify the actuators to control the model’s components in real time.

**Note:** The `mujoco.viewer.launch_passive` function is a helper function that creates a window to visualize the simulation. You can also use the `mujoco.viewer.launch` function to interact with the simulation using the mouse and keyboard. **Any of the two functions _require_ importing the `mujoco.viewer` module**.

## Repository Structure

- `/models`: Contains MuJoCo models in XML format that represent different automatic control systems.
- `/scripts`: Contains Python scripts to load and control the simulations.
- `/samples`: Contains sample code snippets to help you get started.
- `/docs`: Documentation and guides for the included experiments. (Not ready)

## Contributions

This repository is under continuous development. If you have any suggestions, improvements, or would like to contribute, you are welcome to do so! You can submit a pull request or open an issue with your ideas.

## License

This project is licensed under the [MIT License](https://mit-license.org/).
