import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
import math

class InteractiveRocketViewer:
    def __init__(self, csv_file):
        """Initialize the interactive rocket viewer with CSV data."""
        self.df = pd.read_csv(csv_file)
        self.setup_data()
        self.current_frame = 0
        self.is_playing = True
        self.animation = None
        self.updating_slider = False  # Flag to prevent recursion
        
    def setup_data(self):
        """Preprocess the data for visualization."""
        # Convert time from ms to seconds
        self.df['Time (s)'] = self.df['Time (ms)'] / 1000.0
        
        # Convert input angles to radians for calculations
        self.df['Input x (rad)'] = np.radians(self.df['Input x'])
        self.df['Input y (rad)'] = np.radians(self.df['Input y'])
        
        # Rocket cylinder parameters
        self.rocket_height = 10.0
        self.rocket_radius = 1.0
        
        # Pre-create rocket mesh to avoid recreating every frame
        self.rocket_meshes = self.create_all_rocket_meshes()
        
    def create_cylinder_mesh(self, height=10, radius=1, resolution=20):
        """Create a cylinder mesh for the rocket."""
        theta = np.linspace(0, 2*np.pi, resolution)
        z = np.linspace(0, height, 10)
        
        theta_mesh, z_mesh = np.meshgrid(theta, z)
        
        x_mesh = radius * np.cos(theta_mesh)
        y_mesh = radius * np.sin(theta_mesh)
        
        return x_mesh, y_mesh, z_mesh
    
    def create_rocket_cone(self, height=2, radius=1, resolution=20):
        """Create a cone for the rocket nose."""
        theta = np.linspace(0, 2*np.pi, resolution)
        z = np.linspace(0, height, 6)
        
        theta_mesh, z_mesh = np.meshgrid(theta, z)
        
        r_mesh = radius * (1 - z_mesh / height)
        x_mesh = r_mesh * np.cos(theta_mesh)
        y_mesh = r_mesh * np.sin(theta_mesh)
        z_mesh = z_mesh + self.rocket_height
        
        return x_mesh, y_mesh, z_mesh
    
    def create_rocket_fins(self):
        """Create simple fins for the rocket."""
        fins = []
        fin_height = 2
        fin_width = 2.5
        
        for i in range(4):
            angle = i * np.pi / 2
            
            # Simple rectangular fin
            x_fin = np.array([
                [self.rocket_radius * np.cos(angle), (self.rocket_radius + fin_width) * np.cos(angle)],
                [self.rocket_radius * np.cos(angle), (self.rocket_radius + fin_width) * np.cos(angle)]
            ])
            y_fin = np.array([
                [self.rocket_radius * np.sin(angle), (self.rocket_radius + fin_width) * np.sin(angle)],
                [self.rocket_radius * np.sin(angle), (self.rocket_radius + fin_width) * np.sin(angle)]
            ])
            z_fin = np.array([
                [0, 0],
                [fin_height, fin_height]
            ])
            
            fins.append((x_fin, y_fin, z_fin))
        
        return fins
    
    def create_all_rocket_meshes(self):
        """Pre-create all rocket mesh components."""
        return {
            'cylinder': self.create_cylinder_mesh(),
            'cone': self.create_rocket_cone(),
            'fins': self.create_rocket_fins()
        }
    
    def apply_rotation(self, x, y, z, angle_x, angle_y):
        """Apply rotation transforms based on input angles."""
        # Rotation around X-axis (pitch)
        cos_x, sin_x = np.cos(angle_x), np.sin(angle_x)
        y_rot = y * cos_x - z * sin_x
        z_rot = y * sin_x + z * cos_x
        x_rot = x
        
        # Rotation around Y-axis (yaw)
        cos_y, sin_y = np.cos(angle_y), np.sin(angle_y)
        x_final = x_rot * cos_y + z_rot * sin_y
        y_final = y_rot
        z_final = -x_rot * sin_y + z_rot * cos_y
        
        return x_final, y_final, z_final
    
    def update_rocket_display(self, frame_idx):
        """Update only the rocket display without clearing axes."""
        # Store current view
        elev = self.ax.elev
        azim = self.ax.azim
        
        # Clear only the rocket objects
        for artist in self.ax.collections[:]:
            artist.remove()
        for artist in self.ax.artists[:]:
            artist.remove()
        
        # Get current angles
        angle_x = self.df.iloc[frame_idx]['Input x (rad)']
        angle_y = self.df.iloc[frame_idx]['Input y (rad)']
        time_val = self.df.iloc[frame_idx]['Time (s)']
        
        # Apply rotations to pre-created meshes
        x_cyl, y_cyl, z_cyl = self.rocket_meshes['cylinder']
        x_rot, y_rot, z_rot = self.apply_rotation(x_cyl, y_cyl, z_cyl, angle_x, angle_y)
        
        x_cone, y_cone, z_cone = self.rocket_meshes['cone']
        x_cone_rot, y_cone_rot, z_cone_rot = self.apply_rotation(x_cone, y_cone, z_cone, angle_x, angle_y)
        
        # Plot rocket components
        self.ax.plot_surface(x_rot, y_rot, z_rot, alpha=0.9, color='lightblue', 
                           edgecolor='navy', linewidth=0.2)
        self.ax.plot_surface(x_cone_rot, y_cone_rot, z_cone_rot, alpha=0.9, color='red', 
                           edgecolor='darkred', linewidth=0.2)
        
        # Plot fins
        for x_fin, y_fin, z_fin in self.rocket_meshes['fins']:
            x_fin_rot, y_fin_rot, z_fin_rot = self.apply_rotation(x_fin, y_fin, z_fin, angle_x, angle_y)
            self.ax.plot_surface(x_fin_rot, y_fin_rot, z_fin_rot, alpha=0.8, color='orange',
                               edgecolor='darkorange', linewidth=0.2)
        
        # Restore view
        self.ax.view_init(elev=elev, azim=azim)
        
        # Update title
        self.ax.set_title(f'Time: {time_val:.2f}s | X: {math.degrees(angle_x):.1f}° | Y: {math.degrees(angle_y):.1f}°',
                         fontsize=14, fontweight='bold', color='white')
        
        # Update slider position without triggering callback
        if not self.updating_slider:
            self.updating_slider = True
            self.time_slider.set_val(frame_idx)
            self.updating_slider = False
    
    def setup_axes(self):
        """Set up the 3D axes with fixed properties."""
        self.ax.set_xlim([-15, 15])
        self.ax.set_ylim([-15, 15])
        self.ax.set_zlim([0, 15])
        self.ax.set_xlabel('X', fontsize=12, color='white')
        self.ax.set_ylabel('Y', fontsize=12, color='white')
        self.ax.set_zlabel('Z', fontsize=12, color='white')
        
        # Add coordinate axes
        self.ax.plot([0, 8], [0, 0], [0, 0], 'r-', linewidth=2, alpha=0.6)
        self.ax.plot([0, 0], [0, 8], [0, 0], 'g-', linewidth=2, alpha=0.6)
        self.ax.plot([0, 0], [0, 0], [0, 8], 'b-', linewidth=2, alpha=0.6)
        
        # Style the axes
        self.ax.tick_params(colors='white')
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        self.ax.grid(False)
        
        # Set initial view
        self.ax.view_init(elev=20, azim=45)
    
    def on_slider_change(self, val):
        """Handle slider value changes."""
        if not self.updating_slider:
            frame_idx = int(val)
            self.current_frame = frame_idx
            self.update_rocket_display(frame_idx)
            self.fig.canvas.draw_idle()
    
    def on_key_press(self, event):
        """Handle keyboard events."""
        if event.key == ' ':  # Spacebar to play/pause
            self.is_playing = not self.is_playing
        elif event.key == 'left':  # Left arrow for previous frame
            self.current_frame = max(0, self.current_frame - 1)
            self.update_rocket_display(self.current_frame)
            self.fig.canvas.draw_idle()
        elif event.key == 'right':  # Right arrow for next frame
            self.current_frame = min(len(self.df) - 1, self.current_frame + 1)
            self.update_rocket_display(self.current_frame)
            self.fig.canvas.draw_idle()
        elif event.key == 'r':  # R to reset view
            self.ax.view_init(elev=20, azim=45)
            self.fig.canvas.draw_idle()
    
    def animate(self, frame):
        """Animation function for automatic playback."""
        if self.is_playing:
            self.current_frame = frame % len(self.df)
            self.update_rocket_display(self.current_frame)
        return []
    
    def create_interactive_viewer(self):
        """Create the interactive 3D rocket viewer."""
        # Set matplotlib to non-interactive mode to avoid recursion
        plt.ioff()
        
        # Create figure
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.patch.set_facecolor('black')
        
        # Create 3D subplot
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor('black')
        
        # Setup axes properties
        self.setup_axes()
        
        # Create time slider
        slider_ax = plt.axes([0.2, 0.02, 0.6, 0.03])
        slider_ax.set_facecolor('gray')
        self.time_slider = Slider(
            slider_ax, 'Time', 0, len(self.df) - 1,
            valinit=0, valfmt='%d', color='lightblue'
        )
        self.time_slider.on_changed(self.on_slider_change)
        
        # Connect keyboard events
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Initial rocket display
        self.update_rocket_display(0)
        
        # Create animation with longer interval to reduce CPU load
        self.animation = FuncAnimation(
            self.fig, self.animate, frames=len(self.df),
            interval=150, repeat=True, blit=True
        )
        
        # Add control instructions
        control_text = [
            'CONTROLS:',
            '• Mouse: Rotate view (drag)',
            '• Spacebar: Play/Pause',
            '• Arrow Keys: Step frame',
            '• R: Reset view',
            '• Slider: Jump to time'
        ]
        
        for i, text in enumerate(control_text):
            weight = 'bold' if i == 0 else 'normal'
            self.fig.text(0.02, 0.95 - i*0.04, text, fontsize=10 if i == 0 else 9, 
                         fontweight=weight, color='white')
        
        # Turn interactive mode back on for display
        plt.ion()
        plt.show()
        
        return self.animation

def main():
    """Main function to run the interactive rocket viewer."""
    csv_file = 'newaug9.csv'
    
    try:
        # Create viewer instance
        viewer = InteractiveRocketViewer(csv_file)
        
        print("=== INTERACTIVE ROCKET VIEWER ===")
        print(f"Loaded {len(viewer.df)} data points")
        print(f"Duration: {viewer.df['Time (s)'].max():.2f} seconds")
        print(f"Angle ranges: X({viewer.df['Input x'].min():.1f}° to {viewer.df['Input x'].max():.1f}°), " +
              f"Y({viewer.df['Input y'].min():.1f}° to {viewer.df['Input y'].max():.1f}°)")
        print("\nStarting interactive viewer...")
        print("Use mouse to rotate around the rocket!")
        
        # Start the interactive viewer
        anim = viewer.create_interactive_viewer()
        
        # Keep the program running
        input("Press Enter to exit...")
        
    except FileNotFoundError:
        print(f"Error: Could not find file '{csv_file}'")
        print("Please make sure the CSV file is in the same directory as this script.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
