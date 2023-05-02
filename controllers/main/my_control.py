import numpy as np

class MyController():
    def __init__(self):
        self.on_ground = True
        self.height_desired = 0.5
        self.obstacle_detected = False
        self.obstacle_direction = None
        self.target_x = None
        self.target_y = None
        self.target_distance_threshold = 1
        self.landing_pad_found = False
        self.obstacle_distance_threshold = 0.2
        self.max_velocity = 1.0
        self.hover_duration = 0.5
        self.hover_start_time = None
        self.hovering = False
        self.land_velocity = 0.1
        self.landing_threshold = 0.1
     
    def step_control(self, sensor_data):
        # Take off
        if self.on_ground and sensor_data['range_down'] < 0.49:
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        elif self.on_ground:
            return [0.0, 0.0, 0.0, self.height_desired]

        # Obstacle avoidance
        elif sensor_data['range_front'] < self.obstacle_distance_threshold:
            if sensor_data['range_left'] < self.obstacle_distance_threshold and sensor_data['range_right'] < self.obstacle_distance_threshold:
                # If there are obstacles in front and on both sides, move backwards
                self.obstacle_detected = True
                self.obstacle_direction = 'backward'
                control_command = [0.0, 0.2, 0.0, self.height_desired]
                return control_command
            elif sensor_data['range_left'] > sensor_data['range_right']:
                # If there is an obstacle on the left, move right
                self.obstacle_detected = True
                self.obstacle_direction = 'right'
                control_command = [0.0, -0.2, 0.0, self.height_desired]
                return control_command
            else:
                # If there is an obstacle on the right or no obstacle, move left
                self.obstacle_detected = True
                self.obstacle_direction = 'left'
                control_command = [0.0, 0.2, 0.0, self.height_desired]
                return control_command

        # Land
        elif sensor_data['landed']:
            self.on_ground = True
            self.landing_pad_found = False
            self.hovering = False
            control_command = [0.0, 0.0, 0.0, 0.0]
            return control_command
        elif self.landing_pad_found and abs(sensor_data['position'][0] - self.target_x) < self.landing_threshold \
                and abs(sensor_data['position'][1] - self.target_y) < self.landing_threshold \
                and abs(sensor_data['velocity'][0]) < self.landing_threshold \
                and abs(sensor_data['velocity'][1]) < self.landing_threshold:
            control_command = [0.0, 0.0, -self.land_velocity, 0.0]
            return control_command
        elif self.landing_pad_found:
            if self.hovering:
                if time.time() - self.hover_start_time < self.hover_duration:
                    control_command = [0.0, 0.0, 0.0, self.height_desired]
                    return control_command
                else:
                    self.hovering = False
                    self.hover_start_time = None
            else:
                self.hovering = True
                self.hover_start_time = time.time()
                control_command = [0.0, 0.0, 0.0, 0.0]
                return control_command
    
        # Move to landing pad
        elif self.target_x is not None and self.target_y is not None:
            target_distance = np.sqrt((sensor_data['position'][0] - self.target_x) ** 2 + (sensor_data['position'][1] - self.target_y) ** 2)
            if target_distance < self.target_distance_threshold:
                self.landing_pad_found = True
                control_command = [0.0, 0.0, 0.0, 0.0]
                return control_command
            else:
                angle_to_target = np.arctan2(self.target_y - sensor_data['position'][1], self.target_x - sensor_data['position'][0])
                angle_error = angle_to_target - sensor_data['heading']
                if angle_error > np.pi:
                    angle_error -= 2 * np.pi
                elif angle_error < -np.pi:
                    angle_error += 2 * np.pi
                if np.abs(angle_error) > 0.1:
                    control_command = [0.0, 0.0, np.sign(angle_error) * 0.5, self.height_desired]
                    return control_command
                else:
                    velocity = self.max_velocity * np.clip(target_distance, 0, 10) / 10
                    control_command = [velocity, 0.0, 0.0, self.height_desired]
                    return control_command

"""
# You can change anything in this file except the file name of 'my_control.py',
# the class name of 'MyController', and the method name of 'step_control'.

# Available sensor data includes data['t'], data['x_global'], data['y_global'],
# data['roll'], data['pitch'], data['yaw'], data['v_forward'], data['v_left'],
# data['range_front'], data['range_left'], data['range_back'],
# data['range_right'], data['range_down'], data['yaw_rate'].
import numpy as np

class MyController():
    def __init__(self):
        self.on_ground = True
        self.height_desired = 0.5
        self.obstacle_detected = False
        self.obstacle_direction = None
        self.target_x = None
        self.target_y = None
        self.target_distance_threshold = 1
        self.landing_pad_found = False
        self.obstacle_distance_threshold = 0.2
        self.max_velocity = 1.0
     
    def step_control(self, sensor_data):
        # Take off
        if self.on_ground and sensor_data['range_down'] < 0.49:
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command

        # Land
        elif sensor_data['range_down'] > 0.5:
            self.height_desired -= 0.005
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            self.on_ground = False
            return control_command

        # Obstacle avoidance
        elif sensor_data['range_front'] < 0.2:
            if sensor_data['range_left'] < 0.2 and sensor_data['range_right'] < 0.2:
                # If there are obstacles in front and on both sides, move backwards
                self.obstacle_detected = True
                self.obstacle_direction = 'backward'
                control_command = [0.0, 0.2, 0.0, self.height_desired]
                return control_command
            elif sensor_data['range_left'] > sensor_data['range_right']:
                # If there is an obstacle on the left, move right
                self.obstacle_detected = True
                self.obstacle_direction = 'right'
                control_command = [0.0, -0.2, 0.0, self.height_desired]
                return control_command
            else:
                # If there is an obstacle on the right or no obstacle, move left
                self.obstacle_detected = True
                self.obstacle_direction = 'left'
                control_command = [0.0, 0.2, 0.0, self.height_desired]
                return control_command
        # Navigate to landing pad
        elif not self.landing_pad_found and not self.obstacle_detected:
            # Calculate target position based on optic-flow sensor
            delta_x = sensor_data['v_left'] / sensor_data['range_down']
            delta_y = sensor_data['v_forward'] / sensor_data['range_down']
            if self.target_x is None or self.target_y is None:
                self.target_x = sensor_data['x_global'] + delta_x
                self.target_y = sensor_data['y_global'] + delta_y
    
            # Update target position if necessary
            distance_to_target = np.sqrt((sensor_data['x_global'] - self.target_x)**2 + (sensor_data['y_global'] - self.target_y)**2)
            if distance_to_target < self.target_distance_threshold:
                self.landing_pad_found = True
                control_command = [0.0, 0.0, 0.0, self.height_desired]
                return control_command
            else:
                self.target_x = sensor_data['x_global'] + delta_x
                self.target_y = sensor_data['y_global'] + delta_y
    
            # Calculate velocity commands based on target position
            delta_x = self.target_x - sensor_data['x_global']
            delta_y = self.target_y - sensor_data['y_global']
            angle_to_target = np.arctan2(delta_y, delta_x)
            heading_error = angle_to_target - sensor_data['yaw']
            if heading_error > np.pi:
                heading_error -= 2*np.pi
            elif heading_error < -np.pi:
                heading_error += 2*np.pi
    
            velocity_forward = min(self.max_velocity, 0.5*np.sqrt(delta_x**2 + delta_y**2)*np.cos(heading_error))
            velocity_left = min(self.max_velocity, 0.5*np.sqrt(delta_x**2 + delta_y**2)*np.sin(heading_error))
            control_command = [velocity_forward, velocity_left, 0.0, self.height_desired]
            return control_command
    
        # No obstacle detected, continue moving forward
        else:
            if self.obstacle_detected:
                # If an obstacle was previously detected and avoided, continue in the new direction
                if self.obstacle_direction == 'left':
                    control_command = [0.0, 0.2, 0.0, self.height_desired]
                    self.obstacle_detected = False
                    self.obstacle_direction = None
                    return control_command
                elif self.obstacle_direction == 'right':
                    control_command = [0.0, -0.2, 0.0, self.height_desired]
                    self.obstacle_detected = False
                    self.obstacle_direction = None
                    return control_command
                elif self.obstacle_direction == 'backward':
                    control_command = [0.0, 0.0, 0.0, self.height_desired]
                    self.obstacle_detected = False
                    self.obstacle_direction = None
                    return control_command
            control_command = [0.2, 0.0, 0.0, self.height_desired]
            return control_command
         

               
"""
"""
    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
        # Take off
        if self.on_ground and sensor_data['range_down'] < 0.49:
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            self.on_ground = False
            return control_command

        # Obstacle avoidance
        if sensor_data['range_front'] < 0.2 or sensor_data['range_left'] < 0.2 or sensor_data['range_right'] < 0.2:
            desired_vx = 0.0
            if sensor_data['range_left'] < 0.2:
                desired_vy = 0.2
            elif sensor_data['range_right'] < 0.2:
                desired_vy = -0.2
            else:
                desired_vy = 0.0
            desired_yaw_rate = 0.0
            desired_alt = self.height_desired
            control_command = [desired_vx, desired_vy, desired_yaw_rate, desired_alt]
            return control_command

        # Land on the landing pad
        else:
            # Move forward or backward based on optic-flow sensor readings
            if sensor_data['v_forward'] < -0.01:
                desired_vx = 0.2
            elif sensor_data['v_forward'] > 0.01:
                desired_vx = -0.2
            else:
                desired_vx = 0.0

            # Move left or right based on optic-flow sensor readings
            if sensor_data['v_left'] < -0.01:
                desired_vy = 0.2
            elif sensor_data['v_left'] > 0.01:
                desired_vy = -0.2
            else:
                desired_vy = 0.0

            # Maintain altitude
            desired_alt = self.height_desired

            # Calculate desired yaw rate
            if sensor_data['yaw'] < -5.0:
                desired_yaw_rate = 30.0
            elif sensor_data['yaw'] > 5.0:
                desired_yaw_rate = -30.0
            else:
                desired_yaw_rate = 0.0

            control_command = [desired_vx, desired_vy, desired_yaw_rate, desired_alt]
            return control_command"""