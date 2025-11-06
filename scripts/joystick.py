"""
Driver class for Logitech Extreme 3D Pro joystick.
"""

import numpy as np
import pygame

from robosuite.devices import Device
from robosuite.utils.transform_utils import rotation_matrix


class Joystick(Device):
    """
    A minimalistic driver class for the Logitech Extreme 3D Pro joystick.
    
    Button mapping:
        Trigger (button 0): Toggle gripper
        Button 2: Reset simulation
    
    Axis mapping:
        X axis (left/right): Control y-axis movement
        Y axis (forward/backward): Control x-axis movement  
        Throttle: Control z-axis movement
        Z axis (twist): Control yaw rotation
        HAT: Control pitch/roll rotation
    """

    def __init__(
        self,
        pos_sensitivity=1.0,
        rot_sensitivity=1.0,
        vendor_id=None,
        product_id=None,
    ):
        """
        Initialize the Logitech Extreme 3D Pro joystick device.

        Args:
            pos_sensitivity (float): Sensitivity for position control
            rot_sensitivity (float): Sensitivity for rotation control
            vendor_id: Not used (for compatibility)
            product_id: Not used (for compatibility)
        """
        self._display = None
        self._pos_sensitivity = pos_sensitivity
        self._rot_sensitivity = rot_sensitivity

        self._control = np.zeros(6)  # 3 position + 3 rotation
        self._reset_state = 0
        self._enabled = False
        self._use_gripper = True

        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()
        
        # Check if joystick is connected
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No joystick detected! Please connect your Logitech Extreme 3D Pro.")
        
        # Initialize the first joystick
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        print(f"Connected to: {self.joystick.get_name()}")
        print(f"Number of axes: {self.joystick.get_numaxes()}")
        print(f"Number of buttons: {self.joystick.get_numbuttons()}")
        print(f"Number of hats: {self.joystick.get_numhats()}")
        
        # Button states for toggle detection
        self._button_states = {}
        self._gripper_state = False  # False = open, True = closed
        
        # Deadzone for axes
        self._deadzone = 0.1

    def _apply_deadzone(self, value):
        """Apply deadzone to axis value."""
        if abs(value) < self._deadzone:
            return 0.0
        return value

    def start_control(self):
        """Start control loop."""
        self._enabled = True

    def get_controller_state(self):
        """
        Get the current state of the joystick.
        
        Returns:
            dict: Dictionary containing control values and gripper state
        """
        # Process pygame events
        pygame.event.pump()
        
        # Read axes
        # Extreme 3D Pro axes:
        # 0: X axis (left/right)
        # 1: Y axis (forward/backward)  
        # 2: Z axis (twist)
        # 3: Throttle
        
        x_axis = self._apply_deadzone(self.joystick.get_axis(0))  # X movement
        y_axis = self._apply_deadzone(self.joystick.get_axis(1))  # Y movement
        twist = self._apply_deadzone(self.joystick.get_axis(2))   # Yaw rotation
        throttle = self.joystick.get_axis(3)  # Z movement (throttle is -1 to 1)
        
        # Convert throttle from [-1, 1] to [0, 1] range and invert
        z_axis = -throttle
        z_axis - z_axis*0.01
        # Position control (x, y, z) - Counter-clockwise 90 degree rotation
        self._control[0] = y_axis * self._pos_sensitivity * 0.001
        self._control[1] = x_axis * self._pos_sensitivity * 0.001
        self._control[2] = (z_axis - 0.5) * self._pos_sensitivity * 0.001
        
        # Rotation control (roll, pitch, yaw)
        # Use HAT for pitch and roll - Clockwise 90 degree rotation
        if self.joystick.get_numhats() > 0:
            hat = self.joystick.get_hat(0)
            self._control[3] = -hat[1] * self._rot_sensitivity * 0.002  # Roll
            self._control[4] = hat[0] * self._rot_sensitivity * 0.002  # Pitch
        else:
            self._control[3] = 0
            self._control[4] = 0
            
        self._control[5] = twist * self._rot_sensitivity * 0.002  # Yaw
        
        # Read buttons
        # Button 0 (trigger): Toggle gripper
        # Button 2: Reset
        trigger_pressed = self.joystick.get_button(0)
        reset_button = self.joystick.get_button(2) if self.joystick.get_numbuttons() > 2 else False
        
        # Toggle gripper on trigger press (detect rising edge)
        if trigger_pressed and not self._button_states.get('trigger', False):
            self._gripper_state = not self._gripper_state
        self._button_states['trigger'] = trigger_pressed
        
        # Reset button
        self._reset_state = 1 if reset_button else 0
        
        return dict(
            dpos=self._control[:3],
            rotation=self._control[3:],
            raw_drotation=self._control[3:],
            grasp=int(self._gripper_state),
            reset=self._reset_state,
        )

    @property
    def control(self):
        """
        Get the current control state.
        
        Returns:
            np.array: 6-dimensional control array [dx, dy, dz, droll, dpitch, dyaw]
        """
        return np.array(self._control)

    @property
    def control_gripper(self):
        """
        Get the current gripper state.
        
        Returns:
            int: 1 if gripper should be closed, -1 if open
        """
        return 1 if self._gripper_state else -1

    def reset_internal_state(self):
        """Reset internal state of the device."""
        self._control = np.zeros(6)
        self._reset_state = 0
        self._gripper_state = False
        self._button_states = {}

    def close(self):
        """Close the joystick connection."""
        if hasattr(self, 'joystick'):
            self.joystick.quit()
        pygame.quit()