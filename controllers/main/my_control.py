import numpy as np

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        self.on_ground = True
        self.height_desired = 0.5
        self.index_current_setpoint = 0 
        self.setpoints = [[4.5,1.0]]
        self.max_vx = 0.3  # maximum velocity in the x direction
        self.max_vy = 0.3  # maximum velocity in the y direction

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
        # Take off
        if self.on_ground and sensor_data['range_down'] < 0.49:
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command

        # Check for obstacles
        if sensor_data['range_front'] < 0.2:
            if sensor_data['range_left'] > sensor_data['range_right']:
                control_command = [0.0, -0.8, 0.0, self.height_desired]
            else:
                control_command = [0.0, 0.8, 0.0, self.height_desired]
        else:
            # Move forward
            #control_command = [0.2, 0.0, 0.0, self.height_desired]
            
            
            # Hover at the final setpoint
            if self.index_current_setpoint == len(self.setpoints):
                control_command = [0.0, 0.0, 0.0, self.height_desired]
          
                self.height_desired -= 0.1
                control_command = [0.0, 0.0, 0.0, self.height_desired]
                if self.height_desired <= 0:
                    self.on_ground = True
                return control_command
            
        
            # Get the goal position and drone position
            x_goal, y_goal = self.setpoints[self.index_current_setpoint]
            x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
            distance_drone_to_goal = np.linalg.norm([x_goal - x_drone, y_goal- y_drone])
        
            # When the drone reaches the goal setpoint, e.g., distance < 0.1m
            if distance_drone_to_goal < 0.1:
                # Select the next setpoint as the goal position
                self.index_current_setpoint += 1
                # Hover at the final setpoint
                if self.index_current_setpoint == len(self.setpoints):
                    control_command = [0.0, 0.0, 0.0, self.height_desired]
                    return control_command
        
            # Calculate the control command based on current goal setpoint
            x_goal, y_goal = self.setpoints[self.index_current_setpoint]
            x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
            v_x, v_y = x_goal - x_drone, y_goal - y_drone
            v_x_scaled = v_x / max(abs(v_x), abs(v_y), 1) * self.max_vx
            v_y_scaled = v_y / max(abs(v_x), abs(v_y), 1) * self.max_vy
            control_command = [v_x_scaled, v_y_scaled, 0.0, self.height_desired]
            return control_command
        
        # Land
        if not self.on_ground and sensor_data['range_down'] < 0.49:
            self.height_desired -= 0.5
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            if self.height_desired <= 0:
                self.on_ground = True

        return control_command