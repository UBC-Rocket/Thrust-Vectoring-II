import tkinter as tk
from enum import Enum

# RocketState Enum
class RocketState(Enum):
    IDLE = 1
    IGNITION = 2
    ASCENT = 3
    DESCENT = 4
    LANDING = 5

# RocketStateMachine class
class RocketStateMachine:
    def __init__(self, gui):
        self.current_state = RocketState.IDLE
        self.is_button_pressed = False
        self.gui = gui  # Pass GUI to handle updates

    def get_current_state(self):
        return self.current_state

    def set_current_state(self, state):
        self.current_state = state
        self.gui.update_state_label()  # Update the state in the GUI when it changes

    def transition_to(self, new_state):
        self.set_current_state(new_state)
        self.handle_state_change(new_state)

    def handle_state_change(self, new_state):
        if new_state == RocketState.IGNITION:
            self.start_ignition_sequence()
        elif new_state == RocketState.ASCENT:
            self.start_ascent_timer()
        elif new_state == RocketState.DESCENT:
            self.start_descent_timer()
        elif new_state == RocketState.LANDING:
            self.check_sensor_data()

    def start_ignition_sequence(self):
        self.gui.update_state_label("Ignition started...")
        self.gui.window.after(5000, self.transition_to, RocketState.ASCENT)  # Transition to ASCENT after 5 seconds

    def start_ascent_timer(self):
        self.gui.update_state_label("Ascent started...")
        self.gui.window.after(10000, self.transition_to, RocketState.DESCENT)  # Transition to DESCENT after 10 seconds

    def start_descent_timer(self):
        self.gui.update_state_label("Descent started...")
        self.gui.window.after(5000, self.transition_to, RocketState.LANDING)  # Transition to LANDING after 5 seconds

    def check_sensor_data(self):
        self.gui.update_state_label("Checking sensor data...")
        self.gui.window.after(2000, self.transition_to, RocketState.IDLE)  # Simulate receiving sensor data and transition back to IDLE

    def simulate_button_press(self):
        self.is_button_pressed = True
        self.gui.update_state_label("Button pressed, transitioning from IDLE to IGNITION.")
        self.transition_to(RocketState.IGNITION)

# RocketController class
class RocketController:
    def __init__(self, gui):
        self.state_machine = RocketStateMachine(gui)

    def check_for_transitions(self):
        current_state = self.state_machine.get_current_state()

        if current_state == RocketState.IDLE:
            if not self.state_machine.is_button_pressed:
                self.state_machine.simulate_button_press()
        elif current_state == RocketState.IGNITION:
            pass
        elif current_state == RocketState.ASCENT:
            pass
        elif current_state == RocketState.DESCENT:
            pass
        elif current_state == RocketState.LANDING:
            pass


# GUI Code for Button Press
class RocketGUI:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("Rocket State Machine")

        # Window size and background color
        self.window.geometry("500x300")
        self.window.config(bg="lightblue")

        # Label to show the current state with styling
        self.state_label = tk.Label(self.window, text="Current State: IDLE", font=('Arial', 14), bg="lightblue")
        self.state_label.pack(pady=20)

        # Create a button to simulate the ignition button press
        self.button = tk.Button(self.window, text="Start Ignition", command=self.start_ignition, font=('Arial', 14), bg="green", fg="white", width=20, height=2)
        self.button.pack(pady=20)

        # Create RocketController instance with GUI reference
        self.controller = RocketController(self)

        # Start the GUI main loop
        self.update_state_label()
        self.window.mainloop()

    def start_ignition(self):
        print("Button pressed, transitioning from IDLE to IGNITION.")
        self.controller.state_machine.simulate_button_press()

    def update_state_label(self, message=""):
        # Update the state label with the current state
        current_state = self.controller.state_machine.get_current_state()
        state_text = f"Current State: {current_state.name}"

        if message:  # If a custom message is passed (e.g., during state transition)
            state_text += f"\n{message}"

        self.state_label.config(text=state_text)

        # Update the label every 1 second to check the current state
        self.window.after(1000, self.update_state_label)


# Main function to start the rocket simulation
def main():
    gui = RocketGUI()  # Initialize GUI with controller

if __name__ == "__main__":
    main()
