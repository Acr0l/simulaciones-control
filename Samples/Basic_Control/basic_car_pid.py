import time

import mujoco
import mujoco.viewer
import numpy as np
import random

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        
    def update(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

throttle = 0
steering = 0 

t_pid = PIDController(.1, .0, .08)
s_pid = PIDController(.8, .000001, .007)
Ts = .1

t = 0

mass = 50
radius = .1
width = .2
length = .25
inertia = 1
manual = True

paused = False
def compute_torque(t,s):
    tr = (mass*radius / 2) * t + (inertia*radius/2)*s
    tl = mass*radius/2* t - inertia*radius/2*s
    return tl, tr

def controller(model, data):
    global throttle, steering, manual
    tl, tr = compute_torque(throttle, steering)
    data.ctrl[0] = tr
    data.ctrl[1] = tl
    if manual:
      v_x = data.sensordata[0]
      v_y = data.sensordata[1] 
      v_z = data.sensordata[2]
      g_x = data.sensordata[3]
      g_y = data.sensordata[4]
      g_z = data.sensordata[5]
      v_g = np.sqrt(v_x**2 + v_y**2)
      g_g = np.sqrt(g_x**2 + g_y**2)
      throttle = t_pid.update(throttle, -v_g, Ts)
      steering = s_pid.update(steering, g_g, Ts)
    else:
      angle1 = data.sensor('compass').data[0]
      angle2 = data.sensor('compass').data[1]
      angle =  np.arctan2(-angle1, -angle2)
      if angle < 0:
        angle += 2*np.pi

      # Normalize angle

      
      # Get object position (x,y) from data.xpos
      coords = data.xpos[1, 0:2]
      angle_to_target = np.arctan2(target[0] - coords[0], -target[1] + coords[1])
      if angle_to_target < 0:
        angle_to_target += 2*np.pi
      # Get object orientation from data.qpos
      distance = np.linalg.norm(target - coords)
      steering = s_pid.update(angle_to_target, angle, Ts) *-8
      throttle = t_pid.update(np.linalg.norm(target), np.linalg.norm(coords), Ts) *-80
      #print("Distance: ", round(distance,2))
      #print("Angle: ", np.rad2deg(angle_to_target))
      print("AAT: ", round(np.rad2deg(angle_to_target),2), " ANGLE: ", round(np.rad2deg(angle),2))
      print("DIST: ", round(distance,2), " POS: ", round(coords[0],2), round(coords[1],2), " TARGET: ", round(target[0],2), round(target[1],2))
      
def coord_gen():
    return np.array([random.random() * 1.5, random.random() * 1.5])

def key_callback(keycode):
  global steering,d, throttle, manual
  if chr(keycode) == ' ':
  #paused = not paused
  #if keycode == 45:
      print("manual")
      global manual, target
      manual = not manual
      target = coord_gen()
  if keycode == 265:
      throttle += 1000
  if keycode == 264:
      throttle -= 1000
  if keycode == 263:
      steering -= 100000
  if keycode == 262:
      steering += 100000

      
def main():
    global m,d

    m = mujoco.MjModel.from_xml_path('./car2.xml')
    d = mujoco.MjData(m)
    
    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()

    # Config Camera
    cam.azimuth = 90
    cam.elevation = 90
    cam.distance = 5
    cam.lookat = np.array([0,0,1.5])

    mujoco.mjv_defaultOption(opt)
    scene = mujoco.MjvScene(m, maxgeom=10000)
    
    print("Las flechas mandan impulsos, para avanzar tienes que presionar varias veces UP, lo mismo para girar.\nPara cambiar a modo automático presiona ESPACIO, para cambiar a manual presiona ESPACIO de nuevo.\nLa coordenada objetivo se cambia cada vez que se entra al modo automático.\n Se utilizó ESPACIO como tecla porque M y A representan otros comandos en el visualizador y no son registrados por KeyCallback.")
    
    mujoco.set_mjcb_control(controller)
    with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
      start = time.time()
      #mouse_test = mujoco.mjv_select(m, d, opt, 1., 1., 1., scene)
      while viewer.is_running():
        step_start = time.time()
        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        if not paused:
          mujoco.mj_step(m, d)
              # Pick up changes to the physics state, apply perturbations, update options from GUI.
          viewer.sync()
    
        # Example modification of a viewer option: toggle contact points every two seconds.
        with viewer.lock():
          viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)
    
    
        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
          time.sleep(time_until_next_step)
          
if __name__ == "__main__":
    main()