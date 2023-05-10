

#Best but obstacles conditions contradicting themselves, takeoff pt not random
import numpy as np

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        self.on_ground = True
        self.landed = False
        self.height_desired = 0.5
        self.index_current_setpoint = 0 
        self.control_command_rot = 0 

        setpoints = [[5.0, 0.0], [5.0, 3.0], [4.5, 3.0], [4.5, 0.0], [4.4, 0.0], [4.4, 3.0], [4.2, 3.0], [4.2, 0.0], [4.0, 0.0], [4.0, 3.0], [3.8, 3.0], [3.8, 0.0], [3.6, 0.0], [3.6, 3.0], [3.4, 3.0], [3.4, 0.0]]
        gap = 0.25
        
        vector1 = np.array(setpoints[1]) - np.array(setpoints[0])  # calculate the vector between the first two setpoints
        vector2 = np.array(setpoints[3]) - np.array(setpoints[2])  # calculate the vector between the last two setpoints
        vector3 = np.array(setpoints[5]) - np.array(setpoints[4])  # calculate the vector between the first two setpoints
        vector4 = np.array(setpoints[7]) - np.array(setpoints[6])
        vector5 = np.array(setpoints[9]) - np.array(setpoints[8]) 
        vector6 = np.array(setpoints[11]) - np.array(setpoints[10]) 
        vector7 = np.array(setpoints[13]) - np.array(setpoints[12])
        vector8 = np.array(setpoints[15]) - np.array(setpoints[14])     
        
        distance1 = np.linalg.norm(vector1)  # calculate the distance of the first vector
        distance2 = np.linalg.norm(vector2)
        distance3 = np.linalg.norm(vector3)
        distance4 = np.linalg.norm(vector4)
        distance5 = np.linalg.norm(vector5)
        distance6 = np.linalg.norm(vector6)
        distance7 = np.linalg.norm(vector7)
        distance8 = np.linalg.norm(vector8)
        
        num_points1 = int(distance1 / gap)  # calculate the number of points needed for the first vector with the given gap
        num_points2 = int(distance2 / gap)
        num_points3 = int(distance3 / gap)
        num_points4 = int(distance4 / gap)
        num_points5 = int(distance5 / gap)
        num_points6 = int(distance6 / gap)
        num_points7 = int(distance7 / gap) 
        num_points8 = int(distance8 / gap)  
        
        step_vector1 = vector1 / num_points1  # calculate the step vector for generating points on the first vector
        step_vector2 = vector2 / num_points2
        step_vector3 = vector3 / num_points3
        step_vector4 = vector4 / num_points4
        step_vector5 = vector5 / num_points5
        step_vector6 = vector6 / num_points6 
        step_vector7 = vector7 / num_points7 
        step_vector8 = vector8 / num_points8  
        
        points1 = [setpoints[0] + i * step_vector1 for i in range(num_points1+1)]  # generate all the points on the first vector with the given gap
        points2 = [setpoints[2] + i * step_vector2 for i in range(num_points2+1)]  # generate all the points on the second vector with the given gap
        points3 = [setpoints[4] + i * step_vector3 for i in range(num_points3+1)]
        points4 = [setpoints[6] + i * step_vector4 for i in range(num_points4+1)]
        points5 = [setpoints[8] + i * step_vector5 for i in range(num_points5+1)]
        points6 = [setpoints[10] + i * step_vector6 for i in range(num_points6+1)]
        points7 = [setpoints[12] + i * step_vector7 for i in range(num_points7+1)]
        points8 = [setpoints[14] + i * step_vector8 for i in range(num_points8+1)]
        
        array = points1 + points2 + points3 + points4 + points5 + points6 + points7 + points8
        
        x_take_off = 1
        y_take_off = 0.85
        take_off_pt = [x_take_off,y_take_off]
        goals = [[point[0], point[1]] for point in array]
        print(type(goals))
        goals.append(take_off_pt)
        self.setpoints = goals
        print(self.setpoints)
        print(len(self.setpoints))
        
        #self.setpoints = [[4.5,0.5],[1.0,0.85]]
        self.max_vx = 1  # maximum velocity in the x direction
        self.max_vy = 1  # maximum velocity in the y direction

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
        # Get the goal position and drone position
        x_goal, y_goal = self.setpoints[self.index_current_setpoint]
        x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
        distance_drone_to_goal = np.linalg.norm([x_goal - x_drone, y_goal- y_drone])
        
        print(x_goal, y_goal, x_drone, y_drone, distance_drone_to_goal)
        v_x, v_y = x_goal - x_drone, y_goal - y_drone
        v_x_scaled = (v_x / max(abs(v_x), abs(v_y), 1)) * self.max_vx
        v_y_scaled = (v_y / max(abs(v_x), abs(v_y), 1)) * self.max_vy     
        print(sensor_data['yaw'])    
        #print(sensor_data['range_down']) 
        #print("forward vel ",sensor_data['v_forward']) 
        #print("left vel ",sensor_data['v_left'])
        #print("range left",sensor_data['range_left']) 
        #print("range right",sensor_data['range_right'])
        #print("range front",sensor_data['range_front'])
        #print("range back",sensor_data['range_back'])
        #print(self.index_current_setpoint)
        # Check for obstacles
        
        #front good
        if sensor_data['range_front'] < 0.4:# and sensor_data['v_forward'] > 0.0:
            if sensor_data['range_left'] > sensor_data['range_right']:# or sensor_data['v_left'] > 0.0:
                control_command = [0.0, 0.5, 0.0, self.height_desired]
            else:
                control_command = [0.0, -0.5, 0.0, self.height_desired]
        #elif sensor_data['range_front'] < 0.1 and sensor_data['v_left'] != 0:
        #    control_command = [-0.5, 0.0, 0.0, self.height_desired]
        #when moving back
        #elif sensor_data['range_back'] < 0.3:# and sensor_data['v_forward'] < 0.0 :
        #    if sensor_data['range_left'] < sensor_data['range_right']:# or sensor_data['v_left'] < 0.0:
        #        control_command = [0.0, -0.5, 0.9, self.height_desired]
        #    else:
        #        control_command = [0.0, 0.5, -0.9, self.height_desired]
        #elif sensor_data['range_back'] < 0.1 and sensor_data['v_left'] != 0:
        #    control_command = [0.5, 0.0, 0.0, self.height_desired]
        #right 
        #elif sensor_data['range_right'] < 0.3: #and sensor_data['v_left'] < 0.0:
        #    if sensor_data['range_front'] > sensor_data['range_back']:# or sensor_data['v_forward'] < 0.0:
        #        control_command = [0.5, 0.0, -0.9, self.height_desired]
        #    else:
        #        control_command = [-0.5, 0.0, 0.9, self.height_desired]
        #elif sensor_data['range_right'] < 0.1 and sensor_data['v_forward'] != 0:
        #    control_command = [0.0, 0.5, 0.0, self.height_desired]
                       
        #left
        #elif sensor_data['range_left'] < 0.3:# and sensor_data['v_left'] > 0.0:
        #    if sensor_data['range_front'] > sensor_data['range_back']:# or sensor_data['v_forward'] > 0.0:
        #        control_command = [-0.5, 0.0, 0.0, self.height_desired]
        #    else:
        #        control_command = [-0.5, 0.0, 0.0, self.height_desired]
        #elif sensor_data['range_left'] < 0.1 and sensor_data['v_forward'] != 0:
        #    control_command = [0.0, -0.5, 0.0, self.height_desired]
        

        else:
        # When the drone reaches the goal setpoint, e.g., distance < 0.1m
            if self.index_current_setpoint == 0:
                if distance_drone_to_goal < 0.1:        
                    control_command = [0.0, 0.0, 1.4, self.height_desired] 
                    if 1.5 > sensor_data['yaw'] > 1.4:
                        self.index_current_setpoint += 1
                        control_command = [v_x_scaled, v_y_scaled, 0.0, self.height_desired] 
                    else:
                        #self.index_current_setpoint += 1 
                        control_command = [0.0, 0.0, 1.4, self.height_desired]
                else:
                    control_command = [v_x_scaled, v_y_scaled, 0.0, self.height_desired]
            
            elif self.index_current_setpoint in range(1,12):
                if distance_drone_to_goal < 0.1: 
                    self.index_current_setpoint += 1
                    control_command = [v_x_scaled, 0.0, 0.0, self.height_desired]
                else:
                    control_command = [v_x_scaled, 0.0, 0.0, self.height_desired]  
            
            elif self.index_current_setpoint == 12:
                if distance_drone_to_goal < 0.1:        
                    control_command = [0.0, 0.0, 3.2, self.height_desired] 
                    self.index_current_setpoint += 1
                    if 3.1> sensor_data['yaw'] > 3.0:
                        self.index_current_setpoint += 1
                        control_command = [0.0, v_y_scaled, 0.0, self.height_desired] 
                    else:
                        #self.index_current_setpoint += 1 
                        control_command = [0.0, 0.0, 3.2, self.height_desired]
                else:
                    control_command = [v_x_scaled, 0.0, 0.0, self.height_desired]
            
            elif self.index_current_setpoint == 13:
                if distance_drone_to_goal < 0.1:        
                    control_command = [0.0, 0.0, 1.4, self.height_desired] 
                    if 1.5 > sensor_data['yaw'] > 1.4:
                        self.index_current_setpoint += 1
                        control_command = [v_x_scaled, 0.0, 0.0, self.height_desired] 
                    else:
                        #self.index_current_setpoint += 1 
                        control_command = [0.0, 0.0, 1.4, self.height_desired]
                else:
                    control_command = [0.0, 0.3, 0.0, self.height_desired]
                    
            else:
                if distance_drone_to_goal < 0.1: 
                    self.index_current_setpoint += 1
                    control_command = [v_x_scaled, 2.0, 0.0, self.height_desired]
                else:
                    control_command = [v_x_scaled, 2.0, 0.0, self.height_desired]     
            """if distance_drone_to_goal < 0.13:

                if sensor_data['range_down'] < 0.49:
                    print("pad detected")
                    control_command = [0.0, 0.0, 0.0, 0.0]
                    self.landed = True
                    
                    if self.landed == True and sensor_data['range_down'] <0.012:
                        self.index_current_setpoint = len(self.setpoints) - 1
                        self.landed = False
                        control_command = [0.0, 0.0, 0.0, self.height_desired]
    
                    #land
                    if self.index_current_setpoint == len(self.setpoints) - 1:
                        control_command = [0.0, 0.0, 0.0, 0.0]
                        self.on_ground = False
                else: 
                    if self.index_current_setpoint == 0:
                        
                        control_command = [0.0, 0.0, 1.57, self.height_desired] 
                        if 1.58 > sensor_data['yaw'] > 1.57:
                            self.index_current_setpoint += 1 
                        else:
                            control_command = [v_x_scaled, v_y_scaled, 1.57, self.height_desired]
                        
                    else:
                        self.index_current_setpoint += 1 
                        control_command = [v_x_scaled, v_y_scaled, 0.0, self.height_desired]     
                    
            #elif distance_drone_to_goal > 0.13 and distance_drone_to_goal < 0.49:
            #    if sensor_data['range_front'] < 0.1 or sensor_data['range_back']<0.1 or sensor_data['range_left'] < 0.1 or sensor_data['range_right'] <0.1:
            #        self.index_current_setpoint += 3 
            #        control_command = [v_x_scaled, v_y_scaled, 0.0, self.height_desired]
            #    else: 
            #        control_command = [v_x_scaled, v_y_scaled, 0.0, self.height_desired]  
                
            else:
                control_command = [v_x_scaled, v_y_scaled, 0.0, self.height_desired] 
            """    
        print(control_command)
        return control_command




"""

#Best one, works but the initial is not random, obstacle avoidance need to be improved
#time needs to be improved .
import numpy as np

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        self.on_ground = True
        self.landed = False
        self.height_desired = 0.5
        self.index_current_setpoint = 0 

        setpoints = [[5.0, 0.0], [5.0, 3.0], [4.5, 3.0], [4.5, 0.0], [4.0, 0.0], [4.0, 3.0], [3.5, 3.0], [3.5, 0.0]]
        gap = 0.20
        
        vector1 = np.array(setpoints[1]) - np.array(setpoints[0])  # calculate the vector between the first two setpoints
        vector2 = np.array(setpoints[3]) - np.array(setpoints[2])  # calculate the vector between the last two setpoints
        vector3 = np.array(setpoints[5]) - np.array(setpoints[4])  # calculate the vector between the first two setpoints
        vector4 = np.array(setpoints[7]) - np.array(setpoints[6]) 
        
        distance1 = np.linalg.norm(vector1)  # calculate the distance of the first vector
        distance2 = np.linalg.norm(vector2)
        distance3 = np.linalg.norm(vector3)
        distance4 = np.linalg.norm(vector4)  # calculate the distance of the second vector
        
        num_points1 = int(distance1 / gap)  # calculate the number of points needed for the first vector with the given gap
        num_points2 = int(distance2 / gap)
        num_points3 = int(distance3 / gap)
        num_points4 = int(distance4 / gap)  # calculate the number of points needed for the second vector with the given gap
        
        step_vector1 = vector1 / num_points1  # calculate the step vector for generating points on the first vector
        step_vector2 = vector2 / num_points2
        step_vector3 = vector3 / num_points3
        step_vector4 = vector4 / num_points4  
        
        points1 = [setpoints[0] + i * step_vector1 for i in range(num_points1+1)]  # generate all the points on the first vector with the given gap
        points2 = [setpoints[2] + i * step_vector2 for i in range(num_points2+1)]  # generate all the points on the second vector with the given gap
        points3 = [setpoints[4] + i * step_vector3 for i in range(num_points3+1)]
        points4 = [setpoints[6] + i * step_vector4 for i in range(num_points4+1)]
        
        array = points1 + points2 + points3 + points4
        
        x_take_off = 1
        y_take_off = 0.85
        take_off_pt = [x_take_off,y_take_off]
        goals = [[point[0], point[1]] for point in array]
        print(type(goals))
        goals.append(take_off_pt)
        self.setpoints = goals
        print(self.setpoints)
        


        #self.setpoints = [[4.5,0.5],[1.0,0.85]]
        self.max_vx = 0.5  # maximum velocity in the x direction
        self.max_vy = 0.5  # maximum velocity in the y direction

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
        # Get the goal position and drone position
        x_goal, y_goal = self.setpoints[self.index_current_setpoint]
        x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
        distance_drone_to_goal = np.linalg.norm([x_goal - x_drone, y_goal- y_drone])
        print(x_goal, y_goal, x_drone, y_drone, distance_drone_to_goal)
        v_x, v_y = x_goal - x_drone, y_goal - y_drone
        v_x_scaled = v_x / max(abs(v_x), abs(v_y), 1) * self.max_vx
        v_y_scaled = v_y / max(abs(v_x), abs(v_y), 1) * self.max_vy     
        #print(sensor_data['yaw'])    
        print(sensor_data['range_down'])  
        #print("range left",sensor_data['range_left']) 
        #print("range right",sensor_data['range_right'])

        # Check for obstacles
        
        if sensor_data['range_front'] < 0.3 or sensor_data['range_back']<0.3:
            if sensor_data['range_left'] > sensor_data['range_right']:
                control_command = [0.0, 0.8, -0.9, self.height_desired]
            else:
                control_command = [0.0, -0.8, 0.9, self.height_desired]
        elif sensor_data['range_right'] < 0.3:
            if sensor_data['range_front'] > sensor_data['range_back']:
                control_command = [1.0, 0.0, -0.9, self.height_desired]
            else:
                control_command = [1.0, 0.0, 0.9, self.height_desired]
        #elif sensor_data['range_left'] < 0.3:
        #    if sensor_data['range_front'] > sensor_data['range_back']:
        #       control_command = [-1.0, 0.0, -0.9, self.height_desired] 
        #    else:
        #       control_command = [-1.0, 0.0, 0.9, self.height_desired]        

        else:
            # When the drone reaches the goal setpoint, e.g., distance < 0.1m
            if distance_drone_to_goal < 0.13:
                if sensor_data['range_down'] < 0.49:
                
                    print("pad detected")
                    control_command = [0.0, 0.0, 0.0, 0.0]
                    self.landed = True
                    
                    if self.landed == True and sensor_data['range_down'] <0.012:
                        #problem here doesnt go back after touching the pad
                        self.index_current_setpoint = len(self.setpoints) - 1
                        self.landed = False
                        control_command = [0.0, 0.0, 0.0, self.height_desired]
    
                    #land
                    if self.index_current_setpoint == len(self.setpoints) - 1:
                        
                        control_command = [0.0, 0.0, 0.0, 0.0]
                        self.on_ground = False
                        
                #if near to goal but no pad detected        
                if sensor_data['range_down'] > 0.49:
                    self.index_current_setpoint += 1 
                    control_command = [v_x_scaled, v_y_scaled, 0.0, self.height_desired] 
            elif distance_drone_to_goal > 0.1 and distance_drone_to_goal < 0.5:
                if sensor_data['range_front'] < 0.15 or sensor_data['range_back']<0.15 or sensor_data['range_left'] < 0.3 or sensor_data['range_right'] <0.3:
                        
                    self.index_current_setpoint += 1 
                    control_command = [0.8, 0.0, -sensor_data['yaw'], self.height_desired]          
                else:
                    control_command = [v_x_scaled, v_y_scaled, 0.0, self.height_desired]
                
                
            else:
                control_command = [v_x_scaled, v_y_scaled, -sensor_data['yaw'], self.height_desired]
        print(control_command)    
        return control_command

"""