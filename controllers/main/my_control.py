


"""
#Lands on the first setpoint then stops
import numpy as np

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        self.on_ground = True
        self.height_desired = 0.5
        self.index_current_setpoint = 0 
        self.setpoints = [[4.5,1.0],[1.0,0.85]]
        self.max_vx = 0.3  # maximum velocity in the x direction
        self.max_vy = 0.3  # maximum velocity in the y direction

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
        # Take off
        if self.on_ground and sensor_data['range_down'] < 0.49:
            self.on_ground = False
            control_command = [0.0, 0.0, 0.0, self.height_desired]

        # Check for obstacles
        elif sensor_data['range_front'] < 0.2 or sensor_data['range_back'] < 0.2:
            if sensor_data['range_left'] > sensor_data['range_right']:
                control_command = [0.0, 0.8, 0.0, self.height_desired]
            else:
                control_command = [0.0, -0.8, 0.0, self.height_desired]
        else:
            # Get the goal position and drone position
            x_goal, y_goal = self.setpoints[self.index_current_setpoint]
            x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
            distance_drone_to_goal = np.linalg.norm([x_goal - x_drone, y_goal - y_drone])
        
            # When the drone reaches the goal setpoint, e.g., distance < 0.1m
            if distance_drone_to_goal < 0.1:
                # Hover at the setpoint and land
                if self.index_current_setpoint == 0:
                    control_command = [0.0, 0.0, 0.0, 0.0]
                    self.height_desired = 0.2
                # Take off and go to the next setpoint
                elif self.index_current_setpoint == 1:
                    self.index_current_setpoint = 0
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
"""

"""
from my_controller_funcs import MyControl

class MyController:
    def __init__(self):
        self.controller = MyControl(position=np.zeros(3))

    def step_control(self, sensor_data):
        avoidance_distance = 0.2
        detection_distance = 0.000001
        landing_speed = 0.5
        takeoff_distance = 1.0
        return_speed = 0.5

        velocity = self.controller.obstacle_avoidance(data=sensor_data, speed=1.0, avoidance_distance=avoidance_distance)

        if not self.controller.landing_pad_detected:
            self.controller.search_landing_pad(data=sensor_data, detection_distance=detection_distance)

        if self.controller.landing_pad_detected:
            self.controller.land_on_pad(data=sensor_data, landing_pad_position=np.array([0.0, 0.0, 0.0]), landing_speed=landing_speed)
        else:
            self.controller.take_off(takeoff_distance=takeoff_distance)

        if self.controller.return_to_initial(return_speed=return_speed):
            self.controller.land_on_initial(landing_speed=landing_speed)
            return [0.0, 0.0, 0.0, 0.0]

        yaw_rate = np.arctan2(velocity[1], velocity[0])
        v_forward = np.linalg.norm(velocity) * np.cos(yaw_rate)
        v_left = np.linalg.norm(velocity) * np.sin(yaw_rate)
        altitude = 0.5 #-velocity[2]
        
        control_command = [v_forward, v_left, yaw_rate, altitude]

        return control_command

"""

"""
class MyController():
    def __init__(self):
        self.on_ground = True
        self.height_desired = 0.5
        self.target_point = None

    def navigate_to_point(self, sensor_data, target_point):
        # Compute desired pitch and roll angles based on target point
        dx = target_point[0] - sensor_data['x_global']
        dy = target_point[1] - sensor_data['y_global']
        distance = np.sqrt(dx ** 2 + dy ** 2)
        pitch = -np.arctan2(dx, distance)
        roll = np.arctan2(dy, distance)

        # Compute desired yaw angle based on current heading
        yaw = np.deg2rad(sensor_data['yaw'])
        if yaw > np.pi:
            yaw -= 2 * np.pi
        yaw_error = np.arctan2(np.sin(yaw - np.pi), np.cos(yaw - np.pi))
        yaw_rate = yaw_error / np.pi

        # Compute desired vertical velocity based on current height
        z_error = self.height_desired - sensor_data['range_down']
        vz = np.clip(z_error * 0.5, -0.5, 0.5)

        # Compute desired horizontal velocity based on pitch and roll angles
        vx = np.clip(pitch * 0.5, -0.5, 0.5)
        vy = np.clip(roll * 0.5, -0.5, 0.5)

        # Update target point if reached
        if distance < 0.1:
            self.target_point = None

        # Return control command
        control_command = [vx, vy, vz, yaw_rate]
        return control_command
        
    def obstacle_avoidance(data, speed, avoidance_distance):
        # define avoidance vector
        avoidance_vector = np.zeros(2)
    
        # check if obstacle is in front of the drone
        if data['range_front'] < avoidance_distance:
            avoidance_vector += [np.cos(np.radians(data['yaw'] + 90)), np.sin(np.radians(data['yaw'] + 90))]
    
        # check if obstacle is to the left of the drone
        if data['range_left'] < avoidance_distance:
            avoidance_vector += [np.cos(np.radians(data['yaw'] + 180)), np.sin(np.radians(data['yaw'] + 180))]
    
        # check if obstacle is behind the drone
        if data['range_back'] < avoidance_distance:
            avoidance_vector += [np.cos(np.radians(data['yaw'] - 90)), np.sin(np.radians(data['yaw'] - 90))]
    
        # check if obstacle is to the right of the drone
        if data['range_right'] < avoidance_distance:
            avoidance_vector += [np.cos(np.radians(data['yaw'])), np.sin(np.radians(data['yaw']))]
    
        # check if any avoidance vector was created
        if not np.array_equal(avoidance_vector, [0, 0]):
            # normalize the avoidance vector
            avoidance_vector /= np.linalg.norm(avoidance_vector)
    
            # adjust the speed to avoid obstacles
            speed *= 0.5
    
        return speed, avoidance_vector
        
    def search_landing_pad(sensor_data):
        # Choose a random location within the arena
        x = np.random.uniform(low=-2.5, high=2.5)
        y = np.random.uniform(low=-1.5, high=1.5)
    
        # Search for the landing pad using A* algorithm
        current_pos = (sensor_data['x_global'], sensor_data['y_global'])
        target_pos = (x, y)
    
        path = a_star_search(current_pos, target_pos, sensor_data)
    
        return path  
        
    def land_on_pad(self, sensor_data):
        if sensor_data['range_down'] < 0.49:
            self.height_desired = 0.0
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        else:
            self.height_desired -= 0.005
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
            
    def takeoff(height_desired):
        control_command = [0.0, 0.0, 0.0, height_desired]
        return control_command
        
    def return_to_initial(self, initial_pos):
        # Calculate the Euclidean distance from current position to initial position
        dist = np.linalg.norm(np.array([sensor_data['x_global'], sensor_data['y_global']]) - np.array(initial_pos))
    
        # If the drone is within a certain distance of the initial position, land
        if dist < 0.2:
            self.landed = True
            control_command = [0.0, 0.0, 0.0, 0.0]
            return control_command
        # Otherwise, navigate to the initial position
        else:
            target_pos = initial_pos
            control_command = self.navigate_to_point(sensor_data, target_pos)
            return control_command

    def land_on_initial(self, sensor_data):
        x, y = self.initial_position
        while True:
            target_point = (x, y, self.height_desired)
            control_command = self.navigate_to_point(sensor_data, target_point)
            if control_command is not None:
                return control_command
            elif abs(sensor_data['x_global'] - x) < 0.05 and abs(sensor_data['y_global'] - y) < 0.05:
                return self.land_on_pad(sensor_data)
            else:
                continue

    



"""

#Best one, works but the initial and last point are not random, obstacle avoidance need to be improved
#time needs to be improved .
import numpy as np

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        self.on_ground = True
        self.landed = False
        self.height_desired = 0.5
        self.index_current_setpoint = 0 
        self.setpoints = [[4.5,1.0],[1.0,0.85]]
        self.max_vx = 0.3  # maximum velocity in the x direction
        self.max_vy = 0.3  # maximum velocity in the y direction

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
             
        print(sensor_data['range_down'])    

        # Check for obstacles
        if sensor_data['range_front'] < 0.3 or sensor_data['range_back']<0.3:
            if sensor_data['range_left'] > sensor_data['range_right']:
                control_command = [0.0, 0.8, 0.0, self.height_desired]
            else:
                control_command = [0.0, -0.8, 0.0, self.height_desired]
        else:
            
            # Get the goal position and drone position
            x_goal, y_goal = self.setpoints[self.index_current_setpoint]
            x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
            distance_drone_to_goal = np.linalg.norm([x_goal - x_drone, y_goal- y_drone])
            print(x_goal, y_goal, x_drone, y_drone, distance_drone_to_goal)
            v_x, v_y = x_goal - x_drone, y_goal - y_drone
            v_x_scaled = v_x / max(abs(v_x), abs(v_y), 1) * self.max_vx
            v_y_scaled = v_y / max(abs(v_x), abs(v_y), 1) * self.max_vy
            
            
            # When the drone reaches the goal setpoint, e.g., distance < 0.1m
            if distance_drone_to_goal < 0.13:
                if sensor_data['range_down'] < 0.49:
                
             
                    control_command = [0.0, 0.0, 0.0, 0.0]
                    self.landed = True
                    
                    if self.landed == True and sensor_data['range_down'] <0.012:
                        self.index_current_setpoint += 1
                        self.landed = False
                        control_command = [0.0, 0.0, 0.0, self.height_desired]
    
                    #land
                    if self.index_current_setpoint >= len(self.setpoints):
                        self.index_current_setpoint = 1
                        control_command = [0.0, 0.0, 0.0, 0.0]
                        self.on_ground = False
                if sensor_data['range_down'] > 0.49:                        
                    self.index_current_setpoint = 1 
                    control_command = [0.0, 0.0, 0.0, self.height_desired]  
                
            else:
                
                control_command = [v_x_scaled, v_y_scaled, 0.0, self.height_desired]
            
        return control_command





"""import numpy as np
import time

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        self.on_ground = True
        self.height_desired = 0.5
        self.index_current_setpoint = 0 
        self.setpoints = [[4.5,1.0],[1.0,0.75]]
        self.max_vx = 0.3  # maximum velocity in the x direction
        self.max_vy = 0.3  # maximum velocity in the y direction

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
        # Take off
        if self.on_ground and sensor_data['range_down'] < 0.49:
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            

        # Check for obstacles
        if sensor_data['range_front'] < 0.2 or sensor_data['range_back']<0.2:
            if sensor_data['range_left'] > sensor_data['range_right']:
                control_command = [0.0, 0.8, 0.0, self.height_desired]
            else:
                control_command = [0.0, -0.8, 0.0, self.height_desired]
        else:
            # Move forward
            #control_command = [0.2, 0.0, 0.0, self.height_desired]
            
            # Get the goal position and drone position
            x_goal, y_goal = self.setpoints[self.index_current_setpoint]
            x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
            distance_drone_to_goal = np.linalg.norm([x_goal - x_drone, y_goal- y_drone])
        
            # Hover at the final setpoint
            # When the drone reaches the goal setpoint, e.g., distance < 0.1m
            if distance_drone_to_goal < 0.1:
                # Select the next setpoint as the goal position
                
                
                control_command = [0.0, 0.0, 0.0, self.height_desired]
                
                self.height_desired -= 0.1
                control_command = [0.0, 0.0, 0.0, self.height_desired]
                
               
            
                self.index_current_setpoint += 1
        
        
            
            
                
        
            # Calculate the control command based on current goal setpoint
            x_goal, y_goal = self.setpoints[self.index_current_setpoint]
            x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
            v_x, v_y = x_goal - x_drone, y_goal - y_drone
            v_x_scaled = v_x / max(abs(v_x), abs(v_y), 1) * self.max_vx
            v_y_scaled = v_y / max(abs(v_x), abs(v_y), 1) * self.max_vy
            control_command = [v_x_scaled, v_y_scaled, 0.0, self.height_desired]
            
        


        return control_command"""