# KP = 0.6
# KI = 0.0
# KD = 0.0
#---------------
# ku is the ultimate gain. meaning the largest proportional gain we can have
# with still having a steady oscillation
#ziegler - nichols  method
#classic PID : KI = 0.6 * ku /Tu
#              Kd = 3 * ku * Tu / 40



import numpy as np
import matplotlib.pyplot as plt
import turtle 
import time

# GLOBAL PARAMS
TIME_STEP = 0.001
SIM_TIME = 100         # Simulation duration in time steps (adjust as needed)
SETPOINT_HEIGHT = 100  # Desired altitude (for example, 100 units)
SETPOINT_ANGLE = 0     # Desired pitch angle (in degrees)
MASS = 1.1752             # kg
MAX_THRUST = 100.00      # Newtons
GRAVITY = -9.81        # m/s²
# For rotational dynamics:
MOI = 0.1              # Moment of inertia (kg·m²) - estimated
MAX_TORQUE = 1.0       # Maximum torque available (arbitrary units)

# --- PID GAINS FOR THRUST VECTORING (attitude control) --- 
KP_att = 0.36
KI_att = 40.0
KD_att = 0.00081
antiWindup = True

# --- PID GAINS FOR ALTITUDE (if you want a separate controller) ---
KP_alt = 1.0
KI_alt = 0.0
KD_alt = 0.0

# ===============================
# Modified Rocket Class for 2-DOF motion (translation + rotation)
# ===============================
class Rocket(object):
    def __init__(self):
        # Create turtle object to represent the rocket
        self.body = turtle.Turtle()
        self.body.shape('square')
        self.body.color('black')
        self.body.penup()
        # Starting position
        self.x = 0
        self.y = -100
        self.body.goto(self.x, self.y)
        # Orientation (in degrees) and angular velocity
        self.angle = 0.0            # degrees; 0 means pointing upward
        self.angular_velocity = 0.0 # degrees per second
        
        # Linear velocities
        self.vx = 0.0  # m/s
        self.vy = 0.0  # m/s
        
        # Accelerations
        self.ax = 0.0
        self.ay = 0.0

    def update_dynamics(self, thrust, tvc_angle, torque):
        """
        Update the rocket's state given a thrust magnitude, a thrust vector deflection angle,
        and an applied torque. 
        tvc_angle is the angle (in degrees) by which the thrust vector is deflected from vertical.
        """
        # Decompose thrust into components. Assume tvc_angle is relative to the vertical.
        thrust_rad = np.deg2rad(tvc_angle)
        # F_y is the vertical component, F_x is the horizontal component.
        F_y = thrust * np.cos(thrust_rad)
        F_x = thrust * np.sin(thrust_rad)
        
        # Update linear acceleration (F = m * a)
        self.ax = F_x / MASS
        self.ay = GRAVITY + (F_y / MASS)
        
        # Simple Euler integration for translation:
        self.vx += self.ax * TIME_STEP
        self.vy += self.ay * TIME_STEP
        self.x += self.vx * TIME_STEP
        self.y += self.vy * TIME_STEP
        
        # Update rotational dynamics (using simple Euler integration)
        # Torque = MOI * angular_acceleration (angular acceleration in degrees/s²)
        angular_acc = torque / MOI  # in rad/s^2, then convert to degrees/s^2:
        angular_acc_deg = np.rad2deg(angular_acc)
        self.angular_velocity += angular_acc_deg * TIME_STEP
        self.angle += self.angular_velocity * TIME_STEP
        
        # Update turtle graphics:
        self.body.goto(self.x, self.y)
        self.body.setheading(self.angle)

    def get_altitude(self):
        return self.y

    def get_angle(self):
        return self.angle

# ===============================
# PID Controller Class for Attitude (for thrust vectoring)
# ===============================
class PID_Attitude(object):
    def __init__(self, KP, KI, KD, target):
        self.kp = KP
        self.ki = KI
        self.kd = KD 
        self.setpoint = target  # desired angle in degrees
        self.error = 0.0
        self.integral_error = 0.0
        self.last_error = 0.0
        self.output = 0.0

    def compute(self, current_angle):
        self.error = self.setpoint - current_angle
        self.integral_error += self.error * TIME_STEP
        derivative = (self.error - self.last_error) / TIME_STEP
        self.last_error = self.error
        
        self.output = self.kp*self.error + self.ki*self.integral_error + self.kd*derivative
        
        # Anti-windup: if output saturates, don't integrate further
        if self.output > MAX_THRUST:
            self.output = MAX_THRUST
            if antiWindup:
                self.integral_error -= self.error * TIME_STEP
        elif self.output < -MAX_THRUST:
            self.output = -MAX_THRUST
            if antiWindup:
                self.integral_error -= self.error * TIME_STEP
        return self.output  # This output could be interpreted as the desired torque or tvc_angle

# ===============================
# (Optional) PID Controller for Altitude
# ===============================
class PID_Altitude(object):
    def __init__(self, KP, KI, KD, target):
        self.kp = KP
        self.ki = KI
        self.kd = KD 
        self.setpoint = target  # desired altitude
        self.error = 0.0
        self.integral_error = 0.0
        self.last_error = 0.0
        self.output = 0.0

    def compute(self, current_altitude):
        self.error = self.setpoint - current_altitude
        self.integral_error += self.error * TIME_STEP
        derivative = (self.error - self.last_error) / TIME_STEP
        self.last_error = self.error
        
        self.output = self.kp*self.error + self.ki*self.integral_error + self.kd*derivative
        
        # Clamp output to [0, MAX_THRUST]
        if self.output > MAX_THRUST:
            self.output = MAX_THRUST
        elif self.output < 0:
            self.output = 0
        return self.output

# ===============================
# Simulation Class
# ===============================
class Simulation(object):
    def __init__(self):
        self.rocket = Rocket()
        # PID for attitude control (tvc_angle) - we want to maintain SETPOINT_ANGLE
        self.pid_att = PID_Attitude(KP_att, KI_att, KD_att, SETPOINT_ANGLE)
        # PID for altitude control (optional)
        self.pid_alt = PID_Altitude(KP_alt, KI_alt, KD_alt, SETPOINT_HEIGHT)
        self.timer = 0
        # Data storage for graphing
        self.times = np.array([])
        self.altitudes = np.array([])
        self.angles = np.array([])
        self.tvc_angles = np.array([])  # computed by the attitude PID
        self.thrusts = np.array([])     # commanded thrust from altitude PID

    def cycle(self):
        sim_active = True
        while sim_active:
            # Compute altitude control to determine required thrust
            current_altitude = self.rocket.get_altitude()
            thrust_command = self.pid_alt.compute(current_altitude)
            
            # Compute attitude control to determine the thrust vector deflection (tvc_angle)
            current_angle = self.rocket.get_angle()
            tvc_angle_command = self.pid_att.compute(current_angle)
            # For simplicity, let’s interpret the PID output for attitude as the desired deflection angle.
            # You might also map this to a torque command. Here, we assume:
            torque_command = np.deg2rad(tvc_angle_command) * MAX_TORQUE  # simple mapping
            
            # Update the rocket's dynamics (thrust, tvc_angle, and applied torque)
            self.rocket.update_dynamics(thrust_command, tvc_angle_command, torque_command)
            
            # Record simulation data for plotting
            self.times = np.append(self.times, self.timer * TIME_STEP)
            self.altitudes = np.append(self.altitudes, self.rocket.get_altitude())
            self.angles = np.append(self.angles, self.rocket.get_angle())
            self.tvc_angles = np.append(self.tvc_angles, tvc_angle_command)
            self.thrusts = np.append(self.thrusts, thrust_command)
            
            time.sleep(TIME_STEP)
            self.timer += 1
            if self.timer > SIM_TIME:
                sim_active = False
        self.graph_data()

    def graph_data(self):
        fig, axs = plt.subplots(4, sharex=True)
        axs[0].set_ylabel('Altitude')
        axs[0].plot(self.times, self.altitudes, label='Altitude')
        axs[0].legend()
        
        axs[1].set_ylabel('Angle (deg)')
        axs[1].plot(self.times, self.angles, label='Rocket Angle', color='green')
        axs[1].legend()
        
        axs[2].set_ylabel('TVC Angle (deg)')
        axs[2].plot(self.times, self.tvc_angles, label='TVC Command', color='red')
        axs[2].legend()
        
        axs[3].set_ylabel('Thrust (N)')
        axs[3].plot(self.times, self.thrusts, label='Thrust', color='blue')
        axs[3].set_xlabel('Time (s)')
        axs[3].legend()
        
        plt.show()

def main():
    sim = Simulation()
    sim.cycle()

main()
