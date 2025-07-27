from matplotlib import pyplot as plt
import numpy as np
import time
import PID as pid
import Graphics as gh
import Physics as phys
import RocketConfig
import sys
import os
import pandas as pd
from scipy.interpolate import interp1d



# -------------------------------
# Moment of Inertia Calculation:
# -------------------------------
# Define a helper function for a cylinder's moment of inertia about its own center:
def inertia_cylinder(mass, length, radius):
    # I = (1/12) * m * (3*r^2 + L^2)
    return (1.0/12.0) * mass * (3 * radius**2 + length**2)

# Estimated dimensions (converted to meters) and masses (in kg)
# Rocket Body: 90 cm length, 7.5 cm diameter -> radius = 7.5/2 = 3.75 cm = 0.0375 m
body_length = 0.90       # m
body_radius = 0.0375     # m
m_body = 0.8             # Estimated mass in kg

# Rocket Base: 9 cm length, 12.5 cm diameter -> radius = 6.25 cm = 0.0625 m
base_length = 0.09       # m
base_radius = 0.0625     # m
m_base = 0.15            # Estimated mass in kg

# Battery 1 (assume two identical units): 6.5 cm length, 1 cm diameter -> radius = 0.5 cm = 0.005 m
bat1_length = 0.065      # m
bat1_radius = 0.005      # m
m_bat1 = 0.03            # Estimated mass in kg (each)

# Battery 2: 13.5 cm length, 1 cm diameter -> radius = 0.005 m
bat2_length = 0.135      # m
bat2_radius = 0.005      # m
m_bat2 = 0.05            # Estimated mass in kg

# Assume the overall rocket's center of gravity (CG) is at 0.44 m from the nose.
overall_cg = 0.44  # meters

# For each component, we need an estimate of its center position along the rocket's length.
# (These positions are approximate; adjust them according to your actual rocket design.)
# Let's assume the rocket is oriented with the nose at 0 m and the tail at the bottom.
# - Body center: at half the body length, i.e., 0.90/2 = 0.45 m from the nose.
body_center = 0.45

# - Base: typically at the tail end. Assume its center is at the middle of the base.
# If the base is attached at the bottom, its center might be around body_length + base_length/2.
base_center = body_length + base_length/2  # 0.90 + 0.045 = 0.945 m

# - Batteries: Assume they are mounted inside the rocket.
# We'll assume battery 1 centers are at 0.70 m and battery 2 center at 0.80 m.
bat1_center = 0.70
bat2_center = 0.80

# Calculate individual moments of inertia about each component's own center:
I_body = inertia_cylinder(m_body, body_length, body_radius)
I_base = inertia_cylinder(m_base, base_length, base_radius)
I_bat1 = inertia_cylinder(m_bat1, bat1_length, bat1_radius)
I_bat2 = inertia_cylinder(m_bat2, bat2_length, bat2_radius)

# Use the parallel axis theorem to shift each component's inertia to the overall CG.
def parallel_axis(I_component, mass, component_center, overall_cg):
    d = abs(component_center - overall_cg)
    return I_component + mass * d**2

I_body_adj = parallel_axis(I_body, m_body, body_center, overall_cg)
I_base_adj = parallel_axis(I_base, m_base, base_center, overall_cg)
I_bat1_adj = parallel_axis(I_bat1, m_bat1, bat1_center, overall_cg)
I_bat2_adj = parallel_axis(I_bat2, m_bat2, bat2_center, overall_cg)

# If you have two units of battery 1, multiply accordingly:
I_bat1_total = 2 * I_bat1_adj

# Sum all contributions to obtain the total moment of inertia:
calculated_mmoi = I_body_adj + I_base_adj + I_bat1_total + I_bat2_adj
print("Calculated Moment of Inertia (mmoi):", calculated_mmoi)



# -------------------------------
# A. Read Main Rocket Data (newrocket.csv)
# -------------------------------
csv_file = os.path.join(os.path.dirname(__file__), 'newrocket.csv')
column_names = [
    'Time (s)', 'Altitude (m)', 'Total velocity (m/s)', 'Total acceleration (m/s²)',
    'Angle of attack (°)', 'Pitch rate (°/s)', 'Mass (g)', 'Thrust (N)'
]
data = pd.read_csv(csv_file, comment='#', header=None, names=column_names)
thrust_profile = interp1d(data['Time (s)'], data['Thrust (N)'], fill_value="extrapolate")
print("Main rocket data (first 5 rows):")
print(data.head())

# -------------------------------
# B. Read Additional Data Files
# -------------------------------
# 1. CG vs. Time:
cg_csv_file = os.path.join(os.path.dirname(__file__), 'CGvsTime.csv')
cg_column_names = ['Time (s)', 'CG (m)']  # manually defined header
cg_data = pd.read_csv(cg_csv_file, comment='#', header=None, names=cg_column_names)
print("CG Data Columns:", cg_data.columns)
cg_interp = interp1d(cg_data['Time (s)'], cg_data['CG (m)'], fill_value="extrapolate")

# 2. Roll vs. Yaw:
roll_yaw_csv_file = os.path.join(os.path.dirname(__file__), 'RollvsYaw.csv')
roll_yaw_column_names = ['Time (s)', 'Roll (deg)', 'Yaw (deg)']
roll_yaw_data = pd.read_csv(roll_yaw_csv_file, comment='#', header=None, names=roll_yaw_column_names)
print("Roll vs. Yaw Data Columns:", roll_yaw_data.columns)
roll_interp = interp1d(roll_yaw_data['Time (s)'], roll_yaw_data['Roll (deg)'], fill_value="extrapolate")
yaw_interp = interp1d(roll_yaw_data['Time (s)'], roll_yaw_data['Yaw (deg)'], fill_value="extrapolate")


# -------------------------------
# C. Set Initial Conditions from newrocket.csv
# -------------------------------
initial_altitude = data['Altitude (m)'].iloc[0]
initial_velocity = data['Total velocity (m/s)'].iloc[0]
initial_aoa = np.deg2rad(data['Angle of attack (°)'].iloc[0])  # convert to radians
initial_pitch_rate = np.deg2rad(data['Pitch rate (°/s)'].iloc[0])  # convert to radians/s
initial_mass = data['Mass (g)'].iloc[0] / 1000.0  # convert grams to kg

state_vector = {
    "ax": 0,
    "vx": initial_velocity,   # example: using total velocity as horizontal velocity
    "px": initial_altitude,   # using altitude as one coordinate (adjust as needed)
    "az": 0,
    "vz": initial_velocity,   # similarly for vertical component
    "pz": initial_altitude,
    "alpha": initial_aoa,
    "omega": initial_pitch_rate,
    "theta": 0.0              # desired pitch angle (setpoint) for stabilization
}

# -------------------------------
# D. Set Up Vehicle Properties
# -------------------------------
vehicle = RocketConfig.vehicleProperties(
    mass=initial_mass,
    mmoi=calculated_mmoi,         # Your calculated or estimated moment of inertia
    com2TVC=0.5,                  # This might remain your baseline thrust-vectoring lever arm, or you may update it based on design
                                  #Moment arm is CG - CP = 44 -33 = 11 cm
    servo_lim=np.deg2rad(15),
    servo_rate_lim=np.deg2rad(150)
)



# -------------------------------
# E. Initialize Data Storage for Graphing
# -------------------------------
time_ret = []
angles_ret = []
vert_pos_ret = []
hori_pos_ret = []
pid_output_ret = []  # PID controller output
cg_ret = []          # Center of Gravity over time from CGvsTime.csv
roll_ret = []        # Roll angle over time from RollvsYaw.csv
yaw_ret = []         # Yaw angle over time from RollvsYaw.csv

# Graph labels for six plots:
graph_labels = [
    {"xlab": "Time (s)", "ylab": "Angle (deg)", "title": "Rocket Pitch Angle"},
    {"xlab": "Time (s)", "ylab": "Pos Z", "title": "Horizontal Position"},
    {"xlab": "Time (s)", "ylab": "Pos X", "title": "Vertical Position"},
    {"xlab": "Time (s)", "ylab": "PID Output", "title": "PID Controller Output"},
    {"xlab": "Time (s)", "ylab": "CG (m)", "title": "Center of Gravity vs. Time"},
    {"xlab": "Time (s)", "ylab": "Angle (deg)", "title": "Roll & Yaw vs. Time"}
]

# -------------------------------
# F. Initialize Graphics Handler
# -------------------------------
graphics = gh.GraphicHandler()
graphics.graphsHandler(len(graph_labels), graph_labels)

# Create agents for visualization (rocket and optional target)
rocket = graphics.createAgent('black')
target = graphics.createAgent('red')

# -------------------------------
# G. Initialize Physics Simulation & PID Controller
# -------------------------------
rocket_phys = phys.ThreeDofPhysics(state_vector, vehicle.mass, vehicle.mmoi)
controller = pid.PID(0.07, 0.01, 0.01, 0)
controller.setLims(-10, 10)

# -------------------------------
# H. Simulation Loop
# -------------------------------
sim_time = 0.0
time_lim = 10.0   # total simulation time in seconds
delta_t = 0.1     # simulation time step
tvc_input = 0     # initial TVC actuator input

while sim_time < time_lim:
    # Get current thrust from thrust profile using simulation time.
    current_thrust = thrust_profile(sim_time)
    
    # Compute actuator forces (using current TVC input, current thrust, vehicle properties, and delta_t)
    forces = rocket_phys.calculate_actuator_forces(tvc_input, current_thrust, vehicle, delta_t)
    
    # Update physics simulation.
    rocket_phys.apply_forces(forces, delta_t)
    
    # Update graphics: move the agent (using pz for x and px for y as an example) and rotate by theta.
    graphics.moveAgent(rocket, rocket_phys.state_vector["pz"], rocket_phys.state_vector["px"])
    graphics.rotateAgent(rocket, rocket_phys.state_vector["theta"])
    
    # Record simulation data.
    angles_ret.append(np.rad2deg(rocket_phys.state_vector['theta']))
    hori_pos_ret.append(rocket_phys.state_vector["pz"])
    vert_pos_ret.append(rocket_phys.state_vector["px"])
    time_ret.append(sim_time)
    
    # Compute PID controller output using the current angle (theta) and update TVC input.
    tvc_input = controller.compute(rocket_phys.state_vector['theta'], delta_t)
    pid_output_ret.append(tvc_input)
    
    # Get current values from the additional data:
    current_cg = cg_interp(sim_time)
    cg_ret.append(current_cg)
    current_roll = roll_interp(sim_time)
    roll_ret.append(current_roll)
    current_yaw = yaw_interp(sim_time)
    yaw_ret.append(current_yaw)
    
    # Increment simulation time.
    sim_time += delta_t

# -------------------------------
# I. Display Graphs
# -------------------------------
# We will plot six graphs:
# 1. Rocket Pitch Angle vs. Time
# 2. Horizontal Position (pz) vs. Time
# 3. Vertical Position (px) vs. Time
# 4. PID Controller Output vs. Time
# 5. Center of Gravity vs. Time
# 6. Roll & Yaw vs. Time (both on the same plot)
fig, axs = plt.subplots(6, sharex=True, figsize=(8, 8))
axs[0].set_ylabel(graph_labels[0]['ylab'])
axs[0].set_title(graph_labels[0]['title'])
axs[0].plot(time_ret, angles_ret, 'b-', label='Pitch Angle')
axs[0].legend()

axs[1].set_ylabel(graph_labels[1]['ylab'])
axs[1].set_title(graph_labels[1]['title'])
axs[1].plot(time_ret, hori_pos_ret, 'g-', label='Horizontal Pos')
axs[1].legend()

axs[2].set_ylabel(graph_labels[2]['ylab'])
axs[2].set_title(graph_labels[2]['title'])
axs[2].plot(time_ret, vert_pos_ret, 'r-', label='Vertical Pos')
axs[2].legend()

axs[3].set_ylabel(graph_labels[3]['ylab'])
axs[3].set_title(graph_labels[3]['title'])
axs[3].plot(time_ret, pid_output_ret, 'c-', label='PID Output')
axs[3].legend()

axs[4].set_ylabel(graph_labels[4]['ylab'])
axs[4].set_title(graph_labels[4]['title'])
axs[4].plot(time_ret, cg_ret, 'm-', label='CG')
axs[4].legend()

axs[5].set_ylabel(graph_labels[5]['ylab'])
axs[5].set_title(graph_labels[5]['title'])
axs[5].plot(time_ret, roll_ret, 'y-', label='Roll')
axs[5].plot(time_ret, yaw_ret, 'k--', label='Yaw')
axs[5].legend()

axs[5].set_xlabel(graph_labels[0]['xlab'])
plt.tight_layout()
plt.show()

# Alternatively, if you prefer to use your Graphics module’s showGraphs method,
# you could package the data as:
graphs = [
    (time_ret, angles_ret),
    (time_ret, hori_pos_ret),
    (time_ret, vert_pos_ret),
    (time_ret, pid_output_ret),
    (time_ret, cg_ret),
    (time_ret, roll_ret),  # You may choose to combine roll & yaw differently.
]
# For roll & yaw, you might need to modify the Graphics module to support multiple lines.
graphics.showGraphs(graphs)



# 1. Rocket Pitch Angle vs. Time 
# What It Represents:
# This graph shows the rocket’s pitch angle (in degrees) over time. A pitch angle of 0 degrees means the rocket is perfectly vertical.
# If you see the angle deviate significantly, it means the rocket is tilting forward or backward.
# Large oscillations suggest the rocket is struggling to stabilize, possibly indicating a need to tune the PID gains.


# 2. Horizontal Position (Pos Z) vs. Time 
# This is the rocket’s horizontal displacement (often called “downrange distance” if you imagine it moving along the ground).
# A positive slope means the rocket is drifting horizontally in the positive Z direction.
# The rate of change indicates how fast it’s moving horizontally.


# 3. Vertical Position (Pos X) vs. Time 
# This graph shows the rocket’s vertical position (or altitude) over time.
# A higher value means the rocket is ascending; a lower value means it’s descending.
# A nearly constant slope indicates steady ascent or descent.


# 4. PID Controller Output vs. Time 

# This is output of PID controller, which is used as the thrust vector control (TVC) input.
# It tells you how aggressively the controller is trying to correct any pitch error.
# A higher output suggests the controller is demanding more thrust deflection to fix a perceived pitch error.
# If it saturates at a max or min value, the controller may not have enough authority to correct further.


# 5. Center of Gravity (CG) vs. Time 
# This plots the rocket’s center-of-gravity as read from your CGvsTime.csv file.
# It shows how the CG shifts (often due to fuel burn or mass redistribution) throughout the flight.
# If CG moves significantly, it can alter the rocket’s stability and the moment arm for thrust vectoring.


# 6. Roll & Yaw vs. Time
# Two curves on the same axes: one for roll angle, the other for yaw angle (both in degrees). These angles come from your RollvsYaw.csv.
# Roll is rotation around the rocket’s longitudinal axis; yaw is rotation around a vertical axis.
# Significant deviations here could mean the rocket is rolling or yawing more than intended,
# which might require additional control loops if you want to manage those axes.
