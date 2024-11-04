# Examples of basic methods for simulation competition
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2
from enum import Enum
import logging
import time
from threading import Timer
import math
import time

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2
from enum import Enum


# Global variables
on_ground = True
height_desired = 0.3
timer = None
startpos = None
timer_done = None
altitude_command = 0.0
counter = 0
counter_obstacle = 0
counter_front = 0
counter_back = 0
counter_check = 0
state = 3.75
counter_plateform = 0
mode_landing = 0
x_goal = 0
counter_backward = 0
counter_landing = 0
counter_s = 0
VELOCITY = 0.2
counter_back_final = 0
drop = 0
drop2 = 0
sensor_data = {}


class DroneModes(Enum):
    TAKEOFF = 1
    HOVER = 2
    SEARCH = 3
    AVOIDANCE = 4
    LANDING = 5
    BOUNDARY_CHECK = 6
    GRID = 7  # Nouveau mode pour le quadrillage
    BACK = 8
    END = 9

start_time = 0.0

class DroneController:
    def __init__(self):
        self.mode = DroneModes.TAKEOFF
        self.counter = 0
        # Initialize other variables as needed.

    def takeoff_control(self, sensor_data):

        global on_ground, altitude_command, height_desired, start_time
        print("x_global et y global ", sensor_data['x_global'], "  ", sensor_data['y_global'])
        if sensor_data['range_down'] < 0.29:
            on_ground = False
            altitude_command=altitude_command+0.02
            time.sleep(0.05)
            print("attitude voulue: ", altitude_command, ", attitude actuelle : ", sensor_data['range_down'])
            return [0.0, 0.0, altitude_command, 0.0]
        else:
            print("je rentre dans le else")
            start_time = time.time()
            self.mode = DroneModes.HOVER  # Transition to hover mode after takeoff.
            return [0.0, 0.0, altitude_command, 0.0]
        
    def takeoff_control2(self, sensor_data):

        global on_ground, altitude_command, height_desired, start_time
        print("x_global et y global ", sensor_data['x_global'], "  ", sensor_data['y_global'])
        if sensor_data['range_down'] < 0.29:
            on_ground = False
            altitude_command=altitude_command+0.02
            time.sleep(0.05)
            print("attitude voulue: ", altitude_command, ", attitude actuelle : ", sensor_data['range_down'])
            return [0.0, 0.0, altitude_command, 0.0]
        else:
            print("je rentre dans le else")
            start_time = time.time()
            self.mode = DroneModes.HOVER  # Transition to hover mode after takeoff.
            return [0.0, 0.0, altitude_command, 0.0]

    def hover_control(self, sensor_data):
        # Ici, nous maintenons le drone en position stationnaire.
        if (time.time() - start_time) >= 2.0:
            self.mode = DroneModes.SEARCH
        # self.mode = DroneModes.SEARCH
        return [0.2, 0.0, height_desired, 0.0]
    
    def hover_control2(self, sensor_data):
        # Ici, nous maintenons le drone en position stationnaire.
        if (time.time() - start_time) >= 2.0:
            self.mode = DroneModes.SEARCH
        # self.mode = DroneModes.SEARCH
        return [-0.2, 0.0, height_desired, 0.0]
    
    def search_control(self, sensor_data):
        # Vérifier si le drone doit passer en mode stationnaire
        altitude_error = height_desired - sensor_data['range_down']
        altitude_command = np.clip(altitude_error * 0.5, -1, 1)
        
        if sensor_data['x_global'] >= 3.7:
            print("Je suis en mode grid")
            self.mode = DroneModes.GRID
            return [0.0, 0.0, height_desired, 0.0]  # Commandes pour maintenir la position

        # Inclure la logique pour la recherche de la plateforme et l'évitement des obstacles.
        forward_velocity, lateral_velocity, altitude_command, yaw_rate = self.manage_obstacle_avoidance(sensor_data)
        yaw_rate = 0.0
        
        # Continuez le mouvement si la plateforme n'est pas trouvée et le drone n'est pas encore en position stationnaire.
        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]


    def grid_control(self, sensor_data):
        """ Contrôle le drone pour quadriller la zone spécifiée tout en évitant les obstacles. """
        global counter, counter_obstacle, counter_front, counter_back, counter_check, state, counter_plateform, mode_landing, x_goal, height_desired
        if sensor_data['yaw'] > 0.08:
            counter_check = 0
            print("Trop grand, ", sensor_data['yaw'])
            print("je suis trop a gauche, je vais tourne droite")
            return [0.0, 0.0, height_desired, 0.05]
        if sensor_data['yaw'] < -0.08:
            counter_check = 0
            print("Trop petit", sensor_data['yaw'])
            print("je suis trop a droite, je vais tourne gauche")

            return [0.0, 0.0, height_desired, -0.05]
        # Définir les limites du quadrillage et la taille des pas
        x_min, x_max = 3.70, 4.7
        y_min, y_max = 0.4, 2.6
        step_size = 0.3  # Taille de chaque pas dans le quadrillage (30 cm)
        
        if self.find_platform(sensor_data) and counter_obstacle == 0:
            print("je vois la plateforme et il n'y a pas d obstacle")
            forward_velocity = 0.0
            lateral_velocity = 0.0
            yaw_rate = 0.0
            self.mode = DroneModes.LANDING
            return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
           
        else:
            time.sleep(0.01)  
            yaw_rate = 0.0
            # Assurer que le drone reste dans les limites
            if not (x_min-0.4 <= sensor_data['x_global'] <= x_max+0.2 and y_min - 0.3<= sensor_data['y_global'] <= y_max+0.3):
                    if (sensor_data['y_global'] <= y_min - 0.3) and counter == 0:
                        forward_velocity = 0.1
                        lateral_velocity = 0.4
                        yaw_rate = 0
                        
                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                    if (sensor_data['y_global'] > y_max + 0.3) and counter == 1:
                        forward_velocity = 0.1
                        lateral_velocity = -0.2
                        yaw_rate = 0
                       
                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                    if (sensor_data['x_global'] <= x_min - 0.4):
                        if sensor_data['range_front'] > 0.2:
                            forward_velocity = 0.4
                            lateral_velocity = 0.0
                            yaw_rate = 0
                        else: 
                            forward_velocity = 0.0
                            lateral_velocity = 0.4
                            yaw_rate = 0.0
                        
                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                    
                    if (sensor_data['x_global'] >= x_max+ 0.2):
                        forward_velocity = -0.4
                        lateral_velocity = 0.0
                        yaw_rate = 0
                        
                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                    
                    else:
                        forward_velocity = 0.1
                        lateral_velocity = 0.4
                        yaw_rate = 0
                    # Réinitialiser la position ou ajuster les commandes si nécessaire
                    
                            
                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate] # Exemple de commande pour se repositionner
                    
            else:   
                # Déterminer la prochaine position dans le quadrillage
                yaw_rate = 0.0
                if counter == 0:
                    if sensor_data['x_global'] < 3.5:
                        forward_velocity = 0.2
                        lateral_velocity = 0.0
                        yaw_rate = 0
                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                    if sensor_data['y_global'] < y_max:
                        forward_velocity = 0.0
                        lateral_velocity = 0.3
                        yaw_rate = 0
                        
                        if counter_check <= 40 and sensor_data['y_global'] < y_max - 0.3:
                            if 20 < counter_check < 30:
                                mode_landing = 1
                            if sensor_data['range_left'] < 0.6 or sensor_data['range_back'] < 0.2:
                                counter_check = 170
                                forward_velocity = 0.0
                                lateral_velocity = 0.0
                                yaw_rate = 0.0
                                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                            else:
                                
                                if self.find_platform(sensor_data):
                                    if 20 < counter_check < 30:

                                        mode_landing = 1
                                    if counter_plateform < 40:
                                        counter_plateform +=1
                                        forward_velocity = -0.3
                                        return[forward_velocity, lateral_velocity, height_desired, yaw_rate]
                                    else: 
                                        self.mode = DroneModes.LANDING
                                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                                else: 
                                    counter_check += 1
                                    forward_velocity = -0.3
                                    lateral_velocity = 0.0
                                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                        if 40 < counter_check <= 100 and sensor_data['y_global'] < y_max - 0.3:
                            if 60 < counter_check < 90:
                                mode_landing = 2
                            if sensor_data['range_left'] < 0.6 or sensor_data['range_front'] < 0.2:
                                counter_check = 180
                            else: 
                                if self.find_platform(sensor_data):

                                    if 60 < counter_check > 90:
                                        mode_landing = 2
                                    if counter_plateform < 40:
                                        counter_plateform +=1
                                        forward_velocity = 0.3
                                        return[forward_velocity, lateral_velocity, height_desired, yaw_rate]
                                    else: 
                                        self.mode = DroneModes.LANDING
                                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                                counter_check += 1
                                forward_velocity = 0.525
                                lateral_velocity = 0.0
                                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                        if 100 < counter_check <= 155 and sensor_data['y_global'] < y_max - 0.3:
                            counter_check += 1
                            forward_velocity = -0.42
                            lateral_velocity = 0.0
                            return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                        
                        if 155 < counter_check <= 300 and sensor_data['y_global'] < y_max - 0.1:
                            mode_landing = 0
                            counter_check += 1
                  
                            
                            if sensor_data['range_left'] < 0.6 or counter_obstacle == 1 or counter_obstacle == 2: 
                                
                                if sensor_data['range_left'] < 0.5:
        
                                    forward_velocity = 0.3
                                    lateral_velocity = 0.0
                                    counter_obstacle = 1
                                    counter_check = 180
                                
                                if sensor_data['range_left'] > 0.3 and sensor_data['range_back'] > 0.3 and counter_obstacle == 1:
                                    forward_velocity = 0.0
                                    lateral_velocity = 0.3
                                    counter_check = 180
                                        
                                if sensor_data['range_left'] > 0.3 and sensor_data['range_back'] < 0.3 and counter_obstacle == 1:
                                    counter_obstacle = 2       
                                    counter_check = 180
                                        
                                if sensor_data['range_left'] > 0.3 and sensor_data['range_back'] < 0.3 and counter_obstacle == 2:
                                    forward_velocity = 0.0
                                    lateral_velocity = 0.3
                                    counter_check = 180
                                    
                                        
                                if sensor_data['range_back'] > 0.6 and counter_obstacle == 2 and sensor_data['range_right'] > 0.5:
                                    forward_velocity = -0.6
                                    lateral_velocity = 0.0
                                    counter_check = 180
                                    
                                if sensor_data['range_back'] > 0.4 and counter_obstacle == 2 and sensor_data['range_right'] < 0.5:
                                    counter_obstacle = 3
                                    
                                if counter_obstacle == 3 and sensor_data['range_right'] < 0.4:
                                    forward_velocity = 0.0
                                    lateral_velocity = 0.1
                                    counter_check = 180
                                    
                                if counter_obstacle == 3 and sensor_data['range_back'] > 0.4:
                                    forward_velocity = -0.2
                                    lateral_velocity = 0.0
                                    if sensor_data['range_right'] < 0.4:
                                        forward_velocity = 0.0
                                        lateral_velocity = 0.1
                                        counter_obstacle = 0 
                            
                            if (sensor_data['range_front'] < 0.1 or counter_front == 1  or counter_front == 2) and counter_obstacle == 0:
                                forward_velocity = -0.15
                                lateral_velocity = 0.0
                                counter_front = 1
                                if sensor_data['range_front'] > 0.1 and sensor_data['range_front'] < 0.2:
                                    forward_velocity = 0.0
                                    lateral_velocity = 0.2
                                    counter_front = 2
                                    
                                if sensor_data['range_front'] > 0.1 and counter_front == 2 and sensor_data['range_left'] > 0.5:
                                    forward_velocity = 0.1
                                    lateral_velocity = 0.0
                                    
                                if sensor_data['range_front'] > 0.1 and sensor_data['range_left'] < 0.5 and counter_front == 2:
                                    forward_velocity = 0.0
                                    lateral_velocity = 0.2
                                    
                                if sensor_data['range_front'] > 0.1 and sensor_data['range_left'] > 0.5:
                                    counter_front = 0
                                    forward_velocity = 0.0
                                    lateral_velocity = 0.1
                            #pareille avec le capteur de derrière
                            
                            
                            if (sensor_data['range_back'] < 0.1 or counter_back == 1 or counter_back == 2) and counter_obstacle == 0:
                                forward_velocity = 0.15
                                lateral_velocity = 0.0
                                counter_back = 1
                                if sensor_data['range_back'] > 0.1 and sensor_data['range_back'] < 0.2:
                                    forward_velocity = 0.0
                                    lateral_velocity = 0.2
                                    counter_back = 2
                                    
                                if sensor_data['range_back'] > 0.1 and counter_back == 2 and sensor_data['range_left'] > 0.5:
                                    forward_velocity = 0.1
                                    lateral_velocity = 0.0
                                    
                                if sensor_data['range_back'] > 0.1 and sensor_data['range_left'] < 0.5 and counter_back == 2:
                                    forward_velocity = 0.0
                                    lateral_velocity = 0.1
                                    
                                if sensor_data['range_back'] > 0.1 and sensor_data['range_left'] > 0.5:
                                    forward_velocity = 0.0
                                    lateral_velocity = 0.1
                                    counter_back = 0
                        if 300 < counter_check <= 320:
                            lateral_velocity = 0.0
                            forward_velocity = 0.0
                            yaw_rate = 0.0
                            counter_check += 1
                            
                        if counter_check > 320 and sensor_data['y_global'] < y_max - 0.1:
                            forward_velocity = 0.0
                            lateral_velocity = 0.0
                            counter_check = 0
                                        
                            
                    else: 
                        forward_velocity=0
                        lateral_velocity=0
                        yaw_rate=0
                        if x_max-0.3 < sensor_data['x_global'] < x_max or x_goal == 4:
                            forward_velocity = 0.1
                            lateral_velocity = 0.0
                            yaw_rate = 0
                            counter_obstacle=0
                            if sensor_data['range_front'] < 0.15:
                                    forward_velocity = 0.0
                                    lateral_velocity = -0.2
                                    counter = 2
                            else:
                                forward_velocity=0
                                lateral_velocity=0
                                yaw_rate=0
                                counter = 1
                            
                        elif (x_max-0.6 < sensor_data['x_global'] < x_max-0.3) or x_goal == 3:
                            if sensor_data['x_global'] < x_max-0.305:
                                forward_velocity = 0.1
                                lateral_velocity = 0.0
                                yaw_rate = 0
                                counter_obstacle=0
                                if sensor_data['range_front'] < 0.15:
                                        forward_velocity = 0.0
                                        lateral_velocity = -0.2
                                counter = 2
                            else:
                                forward_velocity=0
                                lateral_velocity=0
                                yaw_rate=0
                                counter = 1
                                if x_goal < 4:
                                    x_goal = 4
                                #else:
                                #    x_goal = 5
                            
                        elif (x_max-0.9 < sensor_data['x_global'] < x_max-0.6 and x_goal < 4) or x_goal == 2:
                            if sensor_data['x_global'] < x_max-0.605:
                                forward_velocity = 0.1
                                lateral_velocity = 0.0
                                yaw_rate = 0
                                counter_obstacle=0
                                if sensor_data['range_front'] < 0.15:
                                        forward_velocity = 0.0
                                        lateral_velocity = -0.2
                                counter = 2
                            else:
                                forward_velocity=0
                                lateral_velocity=0
                                yaw_rate=0
                                counter = 1
                                if x_goal < 3:
                                    x_goal = 3
                                #else:
                                #    x_goal = 4
                                print("x_goal = ", x_goal)

                            
                        elif (x_max-1.2 < sensor_data['x_global'] < x_max-0.9 and x_goal < 3) or x_goal == 1:
                            if sensor_data['x_global'] < x_max-0.905:
                                forward_velocity = 0.1
                                lateral_velocity = 0.0
                                yaw_rate = 0
                                counter_obstacle=0
                                if sensor_data['range_front'] < 0.15:
                                        forward_velocity = 0.0
                                        lateral_velocity = -0.2
                                counter = 2
                            else:
                                forward_velocity=0
                                lateral_velocity=0
                                yaw_rate=0
                                counter = 1

                                if x_goal < 2:
                                    x_goal = 2
                                else: 
                                    x_goal = 3
                                
                                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]

                            
                        elif x_max-1.5 < sensor_data['x_global'] < x_max-1.2 or x_goal == 0:
                            if sensor_data['x_global'] < x_max-1.505:
                                forward_velocity = 0.1
                                lateral_velocity = 0.0
                                yaw_rate = 0
                                counter_obstacle=0
                                if sensor_data['range_front'] < 0.15:
                                        forward_velocity = 0.0
                                        lateral_velocity = -0.2
                                counter = 2
                            else: 
                                forward_velocity=0
                                lateral_velocity=0
                                yaw_rate=0
                                counter = 1
                                if x_goal < 1:
                                    x_goal = 1
                                else:
                                    x_goal = 2
                        elif x_max-1.8 < sensor_data['x_global'] < x_max-1.5:
                            if sensor_data['x_global'] < x_max-1.805:
                                forward_velocity = 0.1
                                lateral_velocity = 0.0
                                yaw_rate = 0
                                if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] < 1.5:
                                    forward_velocity = 0.0
                                    lateral_velocity = -0.1
                                if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] > 1.5:
                                        forward_velocity = 0.0
                                        lateral_velocity = 0.1
                        
                                counter = 2
                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                
                    
                
                elif counter == 1:

                    if sensor_data['y_global'] > y_min:
                        forward_velocity = 0.0
                        lateral_velocity = -0.3
                        yaw_rate = 0
                        
                        
                        if counter_check <= 40 and sensor_data['y_global'] > y_min + 0.3:
                            if 20 < counter_check < 30:
                                mode_landing = 1
                            if sensor_data['range_right'] < 0.6 or sensor_data['range_back'] < 0.2:
                                counter_check = 180
                                forward_velocity = 0.0
                                lateral_velocity = 0.0
                                yaw_rate = 0.0
                                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                            else:
                                if self.find_platform(sensor_data):
                                    if 20 < counter_check < 30:
                                        mode_landing = 1
                                    if counter_plateform < 40:
                                        counter_plateform +=1
                                        forward_velocity = -0.3
                                        return[forward_velocity, lateral_velocity, height_desired, yaw_rate]
                                    else: 
                                        self.mode = DroneModes.LANDING
                                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                                else: 
                                    counter_check += 1
                                    forward_velocity = -0.3
                                    lateral_velocity = 0.0
                                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                        if 40 < counter_check <= 100 and sensor_data['y_global'] > y_min + 0.3:
                            if 90 > counter_check > 60:

                                mode_landing = 2
                            if sensor_data['range_right'] < 0.5 or sensor_data['range_front'] < 0.2:
                                counter_check = 180
                            else: 
                                if self.find_platform(sensor_data):
                                    if 90 > counter_check > 60:

                                        mode_landing = 2
                                    if counter_plateform < 40:
                                        counter_plateform +=1
                                        forward_velocity = 0.3
                                        return[forward_velocity, lateral_velocity, height_desired, yaw_rate]
                                    else: 
                                        self.mode = DroneModes.LANDING
                                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                                counter_check += 1
                                forward_velocity = 0.525
                                lateral_velocity = 0.0
                                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                        if 100 < counter_check <= 155 and sensor_data['y_global'] > y_min + 0.3:
                            
                            counter_check += 1
                            forward_velocity = -0.42
                            lateral_velocity = 0.0
                            return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                        
                        if 155 < counter_check <= 300 and sensor_data['y_global'] > y_min + 0.3:
                            mode_landing = 0
                            counter_check += 1
                        
                        
                            if sensor_data['range_right'] < 0.5 or counter_obstacle == 1 or counter_obstacle == 2: 
                                
                                if sensor_data['range_right'] < 0.5:
        
                                    forward_velocity = 0.3
                                    lateral_velocity = 0.0
                                    counter_obstacle = 1
                                    counter_check = 180
                                
                                if sensor_data['range_right'] > 0.5 and sensor_data['range_back'] > 0.3 and counter_obstacle == 1:
                                    forward_velocity = 0.0
                                    lateral_velocity = -0.3
                                    counter_check = 180
                                        
                                if sensor_data['range_right'] > 0.5 and sensor_data['range_back'] < 0.3 and counter_obstacle == 1:
                                    counter_obstacle = 2       
                                        
                                if sensor_data['range_right'] > 0.5 and sensor_data['range_back'] < 0.3 and counter_obstacle == 2:
                                    forward_velocity = 0.0
                                    lateral_velocity = -0.3
                                    counter_check = 180
                                    
                                        
                                if sensor_data['range_back'] > 0.5 and counter_obstacle == 2 and sensor_data['range_left'] > 0.5:
                                    
                                    forward_velocity = -0.4
                                    lateral_velocity = 0.0
                                    counter_check = 180
                                    
                                if sensor_data['range_back'] > 0.4 and counter_obstacle == 2 and sensor_data['range_left'] < 0.5:
                                    counter_obstacle = 3
                                    
                                if counter_obstacle == 3 and sensor_data['range_left'] < 0.4:
                                    forward_velocity = 0.0
                                    lateral_velocity = -0.1
                                    counter_check = 180
                                    
                                if counter_obstacle == 3 and sensor_data['range_back'] > 0.4:
                                    forward_velocity = -0.1
                                    lateral_velocity = 0.0
                                    counter_check = 180
                                    if sensor_data['range_left'] < 0.4:
                                        forward_velocity = 0.0
                                        lateral_velocity = -0.1
                                        counter_obstacle = 0
                                    
                                    
                                    
                                    
                                    
                            if (sensor_data['range_front'] < 0.1 or counter_front == 1  or counter_front == 2) and counter_obstacle == 0:
                                forward_velocity = -0.15
                                lateral_velocity = 0.0
                                counter_front = 1
                                if sensor_data['range_front'] > 0.1 and sensor_data['range_front'] < 0.2:
                                    forward_velocity = 0.0
                                    lateral_velocity = -0.2
                                    counter_front = 2
                                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                                if sensor_data['range_front'] > 0.1 and counter_front == 2 and sensor_data['range_left'] > 0.5:
                                    forward_velocity = 0.1
                                    lateral_velocity = 0.0
                                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                                if sensor_data['range_front'] > 0.1 and sensor_data['range_left'] < 0.5 and counter_front == 2:
                                    forward_velocity = 0.0
                                    lateral_velocity = -0.2
                                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                                if sensor_data['range_front'] > 0.1 and sensor_data['range_left'] > 0.5:
                                    counter_front = 0
                                    forward_velocity = 0.0
                                    lateral_velocity = -0.1
                            #pareille avec le capteur de derrière
                            
                            
                            if (sensor_data['range_back'] < 0.1 or counter_back == 1 or counter_back == 2) and counter_obstacle == 0: 
                                forward_velocity = 0.15
                                lateral_velocity = 0.0
                                counter_back = 1
                                if sensor_data['range_back'] > 0.1 and sensor_data['range_back'] < 0.2:
                                    forward_velocity = 0.0
                                    lateral_velocity = -0.2
                                    counter_back = 2
                                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                                if sensor_data['range_back'] > 0.1 and counter_back == 2 and sensor_data['range_right'] > 0.5:
                                    forward_velocity = -0.1
                                    lateral_velocity = 0.0
                                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                                if sensor_data['range_back'] > 0.1 and sensor_data['range_right'] < 0.5 and counter_back == 2:
                                    forward_velocity = 0.0
                                    lateral_velocity = -0.1
                                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                                if sensor_data['range_back'] > 0.1 and sensor_data['range_right'] > 0.5:
                                    forward_velocity = 0.0
                                    lateral_velocity = -0.1
                                    counter_back = 0
                        if 300 < counter_check <= 320:
                            lateral_velocity = 0.0
                            forward_velocity = 0.0
                            yaw_rate = 0.0
                            counter_check += 1
                            
                        if counter_check > 320:
                            forward_velocity = 0.0
                            lateral_velocity = 0.0
                            counter_check = 0
                                    
                                    
                    else: 
                        forward_velocity=0
                        lateral_velocity=0
                        yaw_rate=0
                        if x_max-0.3 < sensor_data['x_global'] < x_max or x_goal == 5:
                            forward_velocity = 0.1
                            lateral_velocity = 0.0
                            yaw_rate = 0
                            counter_obstacle=0
                            if sensor_data['range_front'] < 0.15:
                                    forward_velocity = 0.0
                                    lateral_velocity = -0.2
                                    counter = 2
                            else:
                                forward_velocity=0
                                lateral_velocity=0
                                yaw_rate=0
                                counter = 0
                                x_goal = 5
                            
                        if (x_max-0.6 < sensor_data['x_global'] < x_max-0.3 and x_goal < 4) or x_goal == 4:
                            if sensor_data['x_global'] < x_max-0.305:
                                forward_velocity = 0.1
                                lateral_velocity = 0.0
                                yaw_rate = 0
                                counter_obstacle=0
                                if sensor_data['range_front'] < 0.15:
                                        forward_velocity = 0.2
                                        lateral_velocity = -0.3
                                counter = 2
                            else:
                                forward_velocity=0
                                lateral_velocity=0
                                yaw_rate=0
                                counter = 0
                                x_goal = 5
                                print("x_goal = fee", x_goal)
                                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                            
                        elif (x_max-0.9 < sensor_data['x_global'] < x_max-0.6 and x_goal < 3) or x_goal == 3:
                            if sensor_data['x_global'] < x_max-0.605:
                                forward_velocity = 0.1
                                lateral_velocity = 0.0
                                yaw_rate = 0
                                counter_obstacle=0
                                if sensor_data['range_front'] < 0.15:
                                        forward_velocity = 0.2
                                        lateral_velocity = -0.3
                                counter = 2
                            else:
                                forward_velocity=0
                                lateral_velocity=0
                                yaw_rate=0
                                counter = 0
                                x_goal = 4
                                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                            
                        elif (x_max-1.2 < sensor_data['x_global'] < x_max-0.9 and x_goal < 2) or x_goal ==2:
                            if sensor_data['x_global'] < x_max-0.905:
                                forward_velocity = 0.1
                                lateral_velocity = 0.0
                                yaw_rate = 0
                                counter_obstacle=0
                                if sensor_data['range_front'] < 0.15:
                                        forward_velocity = 0.2
                                        lateral_velocity = -0.3
                                counter = 2
                            else:
                                forward_velocity=0
                                lateral_velocity=0
                                yaw_rate=0
                                counter = 0
                                x_goal = 3
                                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                            
                        elif x_max-1.5 < sensor_data['x_global'] < x_max-1.2 and x_goal < 1 or x_goal == 1:
                            if sensor_data['x_global'] < x_max-1.205:
                                forward_velocity = 0.1
                                lateral_velocity = 0.0
                                yaw_rate = 0
                                counter_obstacle=0
                                if sensor_data['range_front'] < 0.15:
                                        forward_velocity = 0.2
                                        lateral_velocity = -0.3
                                counter = 2
                                
                            else: 
                                forward_velocity=0
                                lateral_velocity=0
                                yaw_rate=0
                                counter = 0
                                #x_goal = 0
                                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                        elif x_max-1.8 < sensor_data['x_global'] < x_max-1.5:
                            if sensor_data['x_global'] < x_max-1.805:
                                forward_velocity = 0.1
                                lateral_velocity = 0.0
                                yaw_rate = 0
                                if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] < 1.5:
                                    forward_velocity = 0.2
                                    lateral_velocity = -0.3
                                if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] > 1.5:
                                        forward_velocity = 0.2
                                        lateral_velocity = 0.3
                                counter = 2
                                
                            
                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                        
                    
                if counter == 2:                       
                    if x_max-0.3 < sensor_data['x_global'] < x_max or x_goal == 4:
                        forward_velocity = 0.1
                        lateral_velocity = 0.0
                        yaw_rate = 0
                        if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] < 1.5:
                                forward_velocity = 0.0
                                lateral_velocity = -0.1
                                x_goal = 5
                        if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] > 1.5:
                                forward_velocity = 0.0
                                lateral_velocity = 0.1
                                x_goal = 5
                        counter = 2
                    else:
                        forward_velocity=0
                        lateral_velocity=0
                        yaw_rate=0
                        counter = 0
                        #x_goal = 4
                        
                    if (x_max-0.6 < sensor_data['x_global'] < x_max-0.3 and x_goal < 4) or x_goal == 3:
                        if sensor_data['x_global'] < x_max-0.305:
                            forward_velocity = 0.1
                            lateral_velocity = 0.0
                            yaw_rate = 0
                            if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] < 1.5:
                                forward_velocity = 0.2
                                lateral_velocity = -0.3
                            if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] > 1.5:
                                    forward_velocity = 0.2
                                    lateral_velocity = 0.3
                            counter = 2
                        else:
                            forward_velocity=0
                            lateral_velocity=0
                            yaw_rate=0
                            counter = 0

                        
                    elif (x_max-0.9 < sensor_data['x_global'] < x_max-0.6 and x_goal < 3) or x_goal == 2:
                        if sensor_data['x_global'] < x_max-0.605:
                            forward_velocity = 0.1
                            lateral_velocity = 0.0
                            yaw_rate = 0
                            if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] < 1.5:
                                forward_velocity = 0.2
                                lateral_velocity = -0.3
                            if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] > 1.5:
                                    forward_velocity = 0.2
                                    lateral_velocity = 0.3
                            counter = 2
                        else:
                            forward_velocity=0
                            lateral_velocity=0
                            yaw_rate=0
                            counter = 0
                            #x_goal = 2
                        
                    elif x_max-1.2 < sensor_data['x_global'] < x_max-0.9 or x_goal == 1:
                        if sensor_data['x_global'] < x_max-0.905:
                            forward_velocity = 0.1
                            lateral_velocity = 0.0
                            yaw_rate = 0
                            if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] < 1.5:
                                print("je me décalle un peu sur la gauche, j'ai un obsatcle devant moi")
                                forward_velocity = 0.5
                                lateral_velocity = 0.5
                            if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] > 1.5:
                                    print("je me décalle un peu sur la droite, j'ai un obsatcle devant moi")
                                    forward_velocity = 0.5
                                    lateral_velocity = -0.5
                            counter = 2
                        else:
                            forward_velocity=0
                            lateral_velocity=0
                            yaw_rate=0
                            counter = 0
                        
                    elif x_max-1.2 < sensor_data['x_global'] < x_max-1.5 or x_goal == 0:
                        if sensor_data['x_global'] < x_max-1.505:
                            forward_velocity = 0.1
                            lateral_velocity = 0.0
                            yaw_rate = 0
                            if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] < 1.5:
                                print("je me décalle un peu sur la gauche, j'ai un obsatcle devant moi")
                                forward_velocity = 0.5
                                lateral_velocity = -0.5
                            if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] > 1.5:
                                    print("je me décalle un peu sur la droite, j'ai un obsatcle devant moi")
                                    forward_velocity = 0.5
                                    lateral_velocity = 0.5
                            counter = 2
                        else: 
                            forward_velocity=0
                            lateral_velocity=0
                            yaw_rate=0
                            counter = 0
                            #x_goal = 0
                            
                    elif x_max-1.5 < sensor_data['x_global'] < x_max-1.8:
                        if sensor_data['x_global'] < x_max-1.805:
                            forward_velocity = 0.1
                            lateral_velocity = 0.0
                            yaw_rate = 0
                            if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] < 1.5:
                                print("je me décalle un peu sur la gauche, j'ai un obsatcle devant moi")
                                forward_velocity = 0.0
                                lateral_velocity = -0.1
                            if sensor_data['range_front'] < 0.15 and sensor_data['y_global'] > 1.5:
                                    print("je me décalle un peu sur la droite, j'ai un obsatcle devant moi")
                                    forward_velocity = 0.0
                                    lateral_velocity = 0.1
                            counter = 2
                        else: 
                            forward_velocity=0
                            lateral_velocity=0
                            yaw_rate=0
                            counter = 0
                        
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            

                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]

    def find_platform(self, sensor_data):
        global height_desired
        if sensor_data['range_down'] < height_desired-0.05:
            print("J'ai trouvé la platform")
            # Si la valeur du capteur d'altitude est inférieure à 0.24, on considère que c'est la plateforme
            return True
        else:
            # Sinon, on considère que la plateforme n'est pas trouvée
            return False

    def obstacle_avoidance_right(self, y_global):
        if y_global-0.7 < 0.15:
            return False
        else:
            return True

    def obstacle_avoidance_left(self, y_global):
        if y_global+0.7 > 4.85:
            return False
        else:
            return True
        
    def manage_obstacle_avoidance(self, sensor_data):
        global height_desired, counter_check, counter_obstacle

        print("xglobal = ", sensor_data['x_global'], "    yglobal", sensor_data['y_global'])
        
        if sensor_data['yaw'] > 0.06:
            counter_check = 0
            return [0.0, 0.0, height_desired, 0.1]
        if sensor_data['yaw'] < -0.06:
            counter_check = 0
            return [0.0, 0.0, height_desired, -0.1]
        
        forward_velocity = 0.5  # m/s, vitesse positive pour avancer
        lateral_velocity = 0.0
        yaw_rate = 0.0
        
        if counter_check <= 40:
            time.sleep(0.01)
            if sensor_data['range_front'] < 0.7 or sensor_data['range_left'] < 0.4:
                counter_check = 170
                forward_velocity = 0.0
                lateral_velocity = 0.0
                yaw_rate = 0.0
            else:
                counter_check += 1
                forward_velocity = 0.0
                lateral_velocity = 0.3
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
        if 40 < counter_check <= 100:
            time.sleep(0.01)
            if sensor_data['range_front'] < 0.7 or sensor_data['range_right'] < 0.4:
                counter_check = 170
            else: 
                counter_check += 1
                forward_velocity = 0.0
                lateral_velocity = -0.525
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
        if 100 < counter_check <= 155:
            time.sleep(0.008)
            if sensor_data['range_front'] < 0.7 or sensor_data['range_right'] < 0.4:
                counter_check = 170
            else:
                counter_check += 1
                forward_velocity = 0.0
                lateral_velocity = 0.48
            return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
        
        if 155 < counter_check <= 280:
            time.sleep(0.005)
            counter_check += 1
            # Si un obstacle est détecté à l'avant, arrêter le drone
            if sensor_data['range_front'] < 0.4:
                counter_check = 160
                forward_velocity = 0
                lateral_velocity = 0  # m/s, déplacement vers la droite
                yaw_rate=0
                if self.obstacle_avoidance_right(sensor_data['y_global']) and sensor_data['range_right'] > 0.3 and counter_obstacle == 0:  
                    lateral_velocity = -0.3
                    counter_obstacle = 2
                    
                elif self.obstacle_avoidance_left(sensor_data['y_global']) and sensor_data['range_left'] > 0.3 and counter_obstacle == 0:
                    lateral_velocity = 0.3
                    counter_obstacle = 3


            if counter_obstacle == 2:
                if sensor_data['range_front'] > 0.5:
                    
                    forward_velocity = 0.3
                    lateral_velocity = 0.0
                    counter_check = 160
                    print("je reprends mon chemin")
                    counter_obstacle = 0
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else: 
                    counter_obstacle = 2
                    lateral_velocity = -0.3
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                    
            
            if counter_obstacle == 3:
                if sensor_data['range_front'] > 0.5:
                    forward_velocity = 0.3
                    lateral_velocity = 0.0
                    counter_check = 160
                    counter_obstacle = 0
                    print("je reprends mon chemin")
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else: 
                    counter_obstacle = 3
                    lateral_velocity = 0.3
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                    
            if sensor_data['range_left'] < 0.1:
                print("Obstacle détecté à gauche")
                forward_velocity = 0
                lateral_velocity = -0.1  # m/s, déplacement vers la droite

            # Si un obstacle est détecté à droite, déplacer le drone vers la gauche
            if sensor_data['range_right'] < 0.1:
                print("Obstacle détecté à droite")
                forward_velocity = 0
                lateral_velocity = 0.1  # m/s, déplacement vers la gauche

           
        
        if counter_check > 280:
            forward_velocity = 0.0
            lateral_velocity = 0.0
            yaw_rate=0
            counter_check = 0 
       

        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
    
    def landing_control(self, sensor_data):
        global height_desired, counter_landing, counter_s, drop
        yaw_rate = 0
        
        if counter_landing == 0:
            time.sleep(0.05)
            if counter_s < 50:
                counter_s += 1
                forward_velocity = 0.0
                lateral_velocity = 0.0
                yaw_rate = 0.0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            else: 
                counter_landing = 1
                counter_s = 0
        if counter_landing == 5:
            time.sleep(0.005)
            if counter_s <= 150:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = -0.016
                        lateral_velocity = -0.239
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = 0.08
                    lateral_velocity = 0.08
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    counter_s+=1
                    forward_velocity = 0.22
                    lateral_velocity = 0.22
                    yaw_rate = 0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 150 < counter_s <= 225:
                
                    forward_velocity = 0
                    lateral_velocity = 0
                    yaw_rate = 0
                    counter_s += 1
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 225 < counter_s <= 375:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = 0.016
                        lateral_velocity = 0.239
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = -0.08
                    lateral_velocity = -0.08
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    forward_velocity = -0.22
                    lateral_velocity = -0.22
                    yaw_rate = 0
                    counter_s +=1   
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 375 < counter_s <= 525:
                forward_velocity = 0.0
                lateral_velocity = 0.0
                yaw_rate = 0
                counter_s +=1 
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if counter_s > 525:
                counter_landing = 6
                counter_s = 0
                forward_velocity = 0
                lateral_velocity = 0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                
        if counter_landing == 6:
            time.sleep(0.005)
            if counter_s <= 150:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = -0.016
                        lateral_velocity = 0.239
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = 0.08
                    lateral_velocity = -0.08
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    counter_s+=1
                    forward_velocity = 0.22
                    lateral_velocity = -0.22
                    yaw_rate = 0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 150 < counter_s <= 225:
               
                    forward_velocity = 0
                    lateral_velocity = 0
                    counter_s += 1
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 225 < counter_s <= 375:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = 0.016
                        lateral_velocity = -0.239
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = -0.08
                    lateral_velocity = 0.08
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    forward_velocity = -0.22
                    lateral_velocity = 0.22
                    counter_s +=1 
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 375 < counter_s <= 525:
                forward_velocity = 0.0
                lateral_velocity = 0.0
                counter_s +=1 
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if counter_s > 525:
                counter_landing = 7
                counter_s = 0
                forward_velocity = 0.0
                lateral_velocity = 0.0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            
        if counter_landing == 7:
            time.sleep(0.005)
            if counter_s <= 150:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = 0.016
                        lateral_velocity = 0.239
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = -0.08
                    lateral_velocity = -0.08
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    counter_s+=1
                    forward_velocity = -0.22
                    lateral_velocity = -0.22
                    yaw_rate = 0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 150 < counter_s <= 225:
                
                    forward_velocity = 0
                    lateral_velocity = 0
                    counter_s += 1
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 225 < counter_s <= 375:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = -0.016
                        lateral_velocity = -0.239
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = 0.08
                    lateral_velocity = 0.08
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    forward_velocity = 0.22
                    lateral_velocity = 0.22
                    counter_s +=1 
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 375 < counter_s <= 525:
                forward_velocity = 0.0
                lateral_velocity = 0.0
                counter_s +=1 
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if counter_s > 525:
                counter_landing = 8
                counter_s = 0
                forward_velocity = 0.0
                lateral_velocity = 0.0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            
        if counter_landing == 8:
            time.sleep(0.005)
            if counter_s <= 150:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = 0.016
                        lateral_velocity = -0.239
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = -0.08
                    lateral_velocity = 0.08
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    counter_s+=1
                    forward_velocity = -0.22
                    lateral_velocity = 0.22
                    yaw_rate = 0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 150 < counter_s <= 225:
                
                    forward_velocity = 0
                    lateral_velocity = 0
                    counter_s += 1
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 225 < counter_s <= 375:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = 0.016
                        lateral_velocity = -0.239
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = -0.08
                    lateral_velocity = 0.08
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    forward_velocity = 0.22
                    lateral_velocity = -0.22
                    counter_s +=1 
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 375 < counter_s <= 525:
                forward_velocity = 0.0
                lateral_velocity = 0.0
                counter_s +=1 
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if counter_s > 525:
                counter_landing = 9
                counter_s = 0
                forward_velocity = 0.0
                lateral_velocity = 0.0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            
        if counter_landing == 1:
            time.sleep(0.005)
            if counter_s <= 300:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = 0.0
                        lateral_velocity = -0.239
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                elif sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = 0.0
                    lateral_velocity = 0.08
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.5
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    counter_s+=1
                    forward_velocity = 0.0
                    lateral_velocity = 0.22
                    yaw_rate = 0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 300 < counter_s <= 450:
                
                    forward_velocity = 0
                    lateral_velocity = 0
                    counter_s += 1
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 450 < counter_s <= 750:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = 0.0
                        lateral_velocity = 0.239
                        yaw_rate = 0
                        
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = -0.0
                    lateral_velocity = -0.08
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    forward_velocity = 0.0
                    lateral_velocity = -0.22
                    counter_s +=1 
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 750 < counter_s <= 1050:
                forward_velocity = 0.0
                lateral_velocity = 0.0
                counter_s +=1 
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if counter_s > 1050:
                counter_landing = 2
                counter_s = 0
                forward_velocity = 0.0
                lateral_velocity = 0.0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
        if counter_landing == 2:
            time.sleep(0.005)
            if counter_s <= 300:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = 0.239
                        lateral_velocity = -0.00
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = -0.08
                    lateral_velocity = -0.00
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    counter_s+=1
                    forward_velocity = -0.22
                    lateral_velocity = 0.0
                    yaw_rate = 0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 300 < counter_s <= 450:
                
                    forward_velocity = 0
                    lateral_velocity = 0
                    counter_s += 1
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 450 < counter_s <= 750:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = -0.016
                        lateral_velocity = 0.0
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = 0.08
                    lateral_velocity = 0.0
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    
                    forward_velocity = 0.22
                    lateral_velocity = 0.0
                    counter_s +=1 
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 750 < counter_s <= 1050:
                forward_velocity = 0.0
                lateral_velocity = 0.0
                counter_s +=1 
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if counter_s > 1050:
                counter_landing = 3
                counter_s = 0
                forward_velocity = 0.0
                lateral_velocity = 0.0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
        if counter_landing == 3:
            time.sleep(0.005)
            if counter_s <= 300:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = -0.239
                        lateral_velocity = 0.0
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = 0.08
                    lateral_velocity = -0.0
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    counter_s+=1
                    forward_velocity = 0.22
                    lateral_velocity = 0.0
                    yaw_rate = 0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 300 < counter_s <= 450:
               
                    forward_velocity = 0
                    lateral_velocity = 0
                    counter_s += 1
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 450 < counter_s <= 750:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = 0.016
                        lateral_velocity = 0.0
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = -0.08
                    lateral_velocity = 0.0
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    forward_velocity = -0.22
                    lateral_velocity = 0.0
                    counter_s +=1 
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 750 < counter_s <= 1050:
                forward_velocity = 0.0
                lateral_velocity = 0.0
                counter_s +=1 
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if counter_s > 1050:
                counter_landing = 4
                counter_s = 0
                forward_velocity = 0.0
                lateral_velocity = 0.0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
        if counter_landing == 4:
            time.sleep(0.005)
            if counter_s <= 300:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = 0.0
                        lateral_velocity = 0.239
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = 0.00
                    lateral_velocity = -0.08
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    counter_s+=1
                    forward_velocity = 0.0
                    lateral_velocity = -0.22
                    yaw_rate = 0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 300 < counter_s <= 450:
                
                    forward_velocity = 0
                    lateral_velocity = 0
                    counter_s += 1
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 450 < counter_s <= 750:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = 0.0
                        lateral_velocity = -0.239
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = -0.0
                    lateral_velocity = 0.08
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] < 0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    forward_velocity = 0.0
                    lateral_velocity = 0.22
                    counter_s +=1 
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 750 < counter_s <= 1050:
                forward_velocity = 0.0
                lateral_velocity = 0.0
                counter_s +=1 
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if counter_s > 1050:
                counter_landing = 5
                counter_s = 0
                forward_velocity = 0.0
                lateral_velocity = 0.0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
        if counter_landing == 9:
            time.sleep(0.05)
            height_desired = height_desired-0.08
            forward_velocity = 0.0
            lateral_velocity = 0.0
            if sensor_data['range_down'] < 0.02:
                        
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
            return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            """
        if sensor_data['range_down'] > 0.08:
            height_desired = -0.2
            return [0.0, 0.0, height_desired, 0.0]
        else: 
            height_desired = 0.5
            #passer en mode BACK
            self.mode = DroneModes.BACK
            return [0.0, 0.0, height_desired, 0.0]
            """
    def move_distance(self, distance_x_m, distance_y_m, distance_z_m,
                      velocity=VELOCITY):
        """
        Move in a straight line.
        positive X is forward
        positive Y is left
        positive Z is up

        :param distance_x_m: The distance to travel along the X-axis (meters)
        :param distance_y_m: The distance to travel along the Y-axis (meters)
        :param distance_z_m: The distance to travel along the Z-axis (meters)
        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        distance = math.sqrt(distance_x_m * distance_x_m +
                             distance_y_m * distance_y_m +
                             distance_z_m * distance_z_m)
        flight_time = distance / velocity

        velocity_x = velocity * distance_x_m / distance
        velocity_y = velocity * distance_y_m / distance
        velocity_z = velocity * distance_z_m / distance

        self.start_linear_motion(velocity_x, velocity_y, velocity_z)
        time.sleep(flight_time)
        return
        
    def back_control(self, sensor_data):
        global height_desired, counter_obstacle, counter_backward, counter_check, counter_landing, counter_s, drop2, drop
        yaw_rate = 0.0
        height_desired = 0.3
        """
        if drop2 == 0:
            drop2 = 1
            height_desired = 0.3
            self.takeoff_control2(sensor_data)
            self.hover_control2(sensor_data)
        """
        if sensor_data['yaw'] > 0.05:
            counter_check = 0
            return [0.0, 0.0, height_desired, 0.1]
        if sensor_data['yaw'] < -0.05:
            counter_check = 0
            return [0.0, 0.0, height_desired, -0.1]
        forward_velocity = 0.0
        lateral_velocity = 0.0
        yaw_rate = 0.0
        
        # si je suis bien en y   
        if self.startpos[1]-0.1 < sensor_data['y_global'] < self.startpos[1] + 0.1 or counter_obstacle == 2 or counter_obstacle == 3:
            #si je suis bien en x
            if self.startpos[0]-0.1 < sensor_data['x_global'] < self.startpos[0]+0.1:
                forward_velocity = 0
                lateral_velocity = 0
                counter_landing, counter_s, drop = 0, 0, 0	
                self.mode = DroneModes.END
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            #sinon, je veux reculer en evitant les obstacles
            else:
                forward_velocity = -0.4
                lateral_velocity = 0
                #si j'ai un obstacle derriere moi
                if sensor_data['range_back'] < 0.4 or counter_obstacle == 2 or counter_obstacle == 3:
                    forward_velocity = 0
                    lateral_velocity = 0  # m/s, déplacement vers la droite
                    yaw_rate=0
                    counter_obstacle = 1
                    #si dispo a droite, je pars en counter_obstacle = 2
                    if self.obstacle_avoidance_right(sensor_data['y_global']) and sensor_data['range_right'] > 0.1 and counter_obstacle == 1:
                        lateral_velocity = -0.3
                        counter_obstacle = 2
                    
                    #si dispo a gauche, je pars en counter_obstacle = 3
                    if self.obstacle_avoidance_left(sensor_data['y_global']) and sensor_data['range_left'] > 0.1 and counter_obstacle == 1:
                        lateral_velocity = 0.3
                        counter_obstacle = 3
                        
                    if counter_obstacle == 2:
                        if sensor_data['range_back'] > 0.5:
                            if counter_backward < 150:
                                forward_velocity = -0.8
                                lateral_velocity = 0.0
                                counter_backward += 1
                            else:
                                counter_backward = 0
                                counter_obstacle = 0
                            return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                        else: 
                            if counter_backward < 150:
                                forward_velocity = 0.0
                                lateral_velocity = -0.8
                                counter_backward += 1
                            else:
                                counter_backward = 0
                                counter_obstacle = 2
                                lateral_velocity = -0.3
                            return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                            
                    
                    if counter_obstacle == 3:
                        if sensor_data['range_back'] > 0.5:
                            if counter_backward < 150:
                                forward_velocity = -0.8
                                lateral_velocity = 0.0
                                counter_backward += 1
                            else : 
                                counter_obstacle = 0
                                counter_backward = 0
                            return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                        else: 
                            if counter_backward < 150:
                                forward_velocity = 0.0
                                lateral_velocity = 0.8
                                counter_backward += 1
                            else:
                                counter_backward = 0
                                counter_obstacle = 3
                                lateral_velocity = 0.3
                            return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                    
                else: 
                    forward_velocity = -0.4
                    lateral_velocity = 0
                    counter_check += 1
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                
        #si je suis pas bien en y car y trop grand et pas d'obstacle
        if sensor_data['y_global'] > self.startpos[1] and counter_obstacle == 0:
            forward_velocity = 0
            lateral_velocity = -0.2
            if sensor_data['range_right'] < 0.4:
                forward_velocity = -0.3
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            else: 
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            
        #si je suis pas grand en y car y trop petit et pas d'obstacle
            
        if sensor_data['y_global'] < self.startpos[1] and counter_obstacle == 0:
            forward_velocity = 0
            lateral_velocity = 0.2
            if sensor_data['range_left'] < 0.4:
                forward_velocity = -0.3
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            else: 
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            
        
        if self.startpos[1]-0.1 < sensor_data['y_global'] < self.startpos[1] + 0.1:
            forward_velocity = -0.4  # m/s, vitesse positive pour avancer
            lateral_velocity = 0.01
            yaw_rate = 0.0
            if counter_check <= 40 and counter_obstacle == 0:
                if sensor_data['range_back'] < 0.7 or sensor_data['range_left'] < 0.4:
                    counter_check = 170
                    forward_velocity = 0.0
                    lateral_velocity = 0.0
                    yaw_rate = 0.0
                else:

                    counter_check += 1
                    forward_velocity = 0.0
                    lateral_velocity = 0.3
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 40 < counter_check <= 100 and counter_obstacle == 0: 
                if sensor_data['range_back'] < 0.7 or sensor_data['range_right'] < 0.4:
                    counter_check = 170
                else: 
                    counter_check += 1
                    forward_velocity = 0.0
                    lateral_velocity = -0.525
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 100 < counter_check <= 155 and counter_obstacle == 0:

                counter_check += 1
                forward_velocity = 0.0
                lateral_velocity = 0.48
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            
            if 155 < counter_check <= 280 and counter_obstacle == 0:
                counter_check += 1
                # Si un obstacle est détecté à l'avant, arrêter le drone
                if sensor_data['range_back'] < 0.4:
                    counter_check = 160
                    forward_velocity = 0
                    lateral_velocity = 0  # m/s, déplacement vers la droite
                    yaw_rate=0
                    if counter_obstacle == 0:
                        counter_obstacle = 1
                    if self.obstacle_avoidance_right(sensor_data['y_global']) and sensor_data['range_right'] > 0.3 and counter_obstacle == 1:
                        lateral_velocity = -0.3
                        counter_obstacle = 2
                        
                    elif self.obstacle_avoidance_left(sensor_data['y_global']) and sensor_data['range_left'] > 0.3 and counter_obstacle == 1:
                        lateral_velocity = 0.3
                        counter_obstacle = 3


                if counter_obstacle == 2:
                    if sensor_data['range_back'] > 0.5:
                        forward_velocity = -0.4
                        lateral_velocity = 0.0
                        counter_check = 160
                        counter_obstacle = 0
                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                    else: 
                        counter_obstacle = 2
                        lateral_velocity = -0.3
                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                        
                
                if counter_obstacle == 3:
                    if sensor_data['range_back'] > 0.5:
                        forward_velocity = -0.4
                        lateral_velocity = 0.0
                        counter_check = 160
                        counter_obstacle = 0
                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                    else: 
                        counter_obstacle = 3
                        lateral_velocity = 0.3
                        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                        
                if sensor_data['range_left'] < 0.1:
                    forward_velocity = 0
                    lateral_velocity = -0.1  # m/s, déplacement vers la droite

                # Si un obstacle est détecté à droite, déplacer le drone vers la gauche
                if sensor_data['range_right'] < 0.1:
                    forward_velocity = 0
                    lateral_velocity = 0.1  # m/s, déplacement vers la gauche
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]

            
            
            if counter_check > 280:
                forward_velocity = 0.0
                lateral_velocity = 0.0
                yaw_rate=0
                counter_check = 0 
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]   

           
        
    
            
    def end_control(self, sensor_data):
        global height_desired, counter_s, counter_landing, counter_backward, counter_obstacle, counter_check, drop
        yaw_rate = 0.0
        if counter_landing == 0:
            time.sleep(0.05)
            if counter_s < 50:
                counter_s += 1
                forward_velocity = 0.0
                lateral_velocity = 0.0
                yaw_rate = 0.0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            else: 
                counter_landing = 1
                counter_s = 0
        if counter_landing == 1:
            time.sleep(0.01)
            if counter_s <= 300:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = 0.3
                        lateral_velocity = -0.0
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                elif sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = 0.08
                    lateral_velocity = 0.00
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.5
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    counter_s+=1
                    forward_velocity = -0.3
                    lateral_velocity = 0.
                    yaw_rate = 0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 300 < counter_s <= 450:
                
                    forward_velocity = 0
                    lateral_velocity = 0
                    counter_s += 1
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 450 < counter_s <= 750:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = 0.0
                        lateral_velocity = 0.239
                        yaw_rate = 0
                        
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = -0.0
                    lateral_velocity = -0.08
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    forward_velocity = 0.0
                    lateral_velocity = -0.22
                    counter_s +=1 
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 750 < counter_s <= 1050:
                forward_velocity = 0.0
                lateral_velocity = 0.0
                counter_s +=1 
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if counter_s > 1050:
                counter_landing = 2
                counter_s = 0
                forward_velocity = 0.0
                lateral_velocity = 0.0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
        if counter_landing == 2:
            time.sleep(0.01)
            if counter_s <= 300:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = 0.0
                        lateral_velocity = -0.239
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = -0.0
                    lateral_velocity = -0.08
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    counter_s+=1
                    forward_velocity = -0.22
                    lateral_velocity = 0.0
                    yaw_rate = 0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 300 < counter_s <= 450:
                
                    forward_velocity = 0
                    lateral_velocity = 0
                    counter_s += 1
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 450 < counter_s <= 750:
                print("altitude : ", sensor_data['range_down'])
                if sensor_data['range_down'] > 0.328 or 0 < drop:
                    drop += 1
                    if drop < 86:
                        forward_velocity = 0.0
                        lateral_velocity = -0.00
                    else:
                        forward_velocity = -0.016
                        lateral_velocity = 0.0
                        yaw_rate = 0
                        if height_desired > -0.18:
                            height_desired = height_desired-0.0015
                        if sensor_data['range_down'] <0.016:
                            height_desired = 0.3
                            time.sleep(1.5)
                            self.mode = DroneModes.BACK
                if sensor_data['range_down'] < 0.261 and drop == 0:
                    forward_velocity = 0.08
                    lateral_velocity = 0.0
                    yaw_rate = 0
                    height_desired = -0.2
                    if sensor_data['range_down'] <0.016:
                        height_desired = 0.3
                        time.sleep(1.5)
                        self.mode = DroneModes.BACK
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
                else:
                    
                    forward_velocity = 0.22
                    lateral_velocity = 0.0
                    counter_s +=1 
                    return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if 750 < counter_s <= 1050:
                forward_velocity = 0.0
                lateral_velocity = 0.0
                counter_s +=1 
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
            if counter_s > 1050:
                counter_landing = 3
                counter_s = 0
                forward_velocity = 0.0
                lateral_velocity = 0.0
                return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
        """
        forward_velocity = 0.0
        lateral_velocity = 0.0
        yaw_rate = 0.0
        height_desired = -0.2
        return [forward_velocity, lateral_velocity, height_desired, yaw_rate]
        """

            
        

    def control_loop(self, sensor_data):
        """ Boucle de contrôle principale gérant les transitions entre les modes. """
        if self.mode == DroneModes.TAKEOFF:
            return self.takeoff_control(sensor_data)
        elif self.mode == DroneModes.HOVER:
            return self.hover_control(sensor_data)
        elif self.mode == DroneModes.SEARCH:
            return self.search_control(sensor_data)
        elif self.mode == DroneModes.GRID:
            return self.grid_control(sensor_data)
        elif self.mode == DroneModes.AVOIDANCE:
            return self.manage_obstacle_avoidance(sensor_data)
        elif self.mode == DroneModes.LANDING:
            return self.landing_control(sensor_data)
        elif self.mode == DroneModes.BACK:
            return self.back_control(sensor_data)
        elif self.mode == DroneModes.END:
            return self.end_control(sensor_data)
        else:
            raise ValueError("Mode not recognized!")

    def check_boundaries(self, sensor_data):
        # Cette méthode vérifiera si le drone est à l'intérieur de la zone autorisée.
        pass
    
    def set_startpos(self, startpos):
        self.startpos = startpos
        
    

    # Define the individual control methods for each mode below
    # ...




# The available ground truth state measurements can be accessed by calling sensor_data[item]. All values of "item" are provided as defined in main.py lines 296-323. 
# The "item" values that you can later use in the hardware project are:
# "x_global": Global X position
# "y_global": Global Y position
# "range_down": Downward range finder distance (Used instead of Global Z distance)
# "range_front": Front range finder distance
# "range_left": Leftward range finder distance 
# "range_right": Rightward range finder distance
# "range_back": Backward range finder distance
# "roll": Roll angle (rad)
# "pitch": Pitch angle (rad)
# "yaw": Yaw angle (rad)

# This is the main function where you will implement your control algorithm

drone_controller = DroneController()
startpos = None

def get_command(sensor_data):
    global startpos
    if startpos is None:
        startpos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down']]
        drone_controller.set_startpos(startpos)
    
    control_command = drone_controller.control_loop(sensor_data)

    # Validate control_command before returning
    if control_command is None:
        control_command = [0.0, 0.0, 0.0, 0.0]  # Default command to stop the drone safely
        print("Control command is None. Stopping the drone.")

    #print("rateYaw : ", control_command[3]*180/np.pi)
    
    cf.commander.send_hover_setpoint(control_command[0], control_command[1], control_command[3]*180/np.pi, control_command[2])
    



    
#def recherche_plateforme():
#si on rentre dans recherche_plateforme, le drone doit trouver un moment ou l'altitude change et s'arreter quand elle change.
    
#Il doit aussi ralentir sa vitesse de déplacement pour ne pas rater la plateforme
#Il doit ensuite cherche un angle de la plateforme et venir se positionner au centre de la plateforme qui fait 30cm par 30 cm
    
    
    


# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
"""








uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=50)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        self._lg_stab.add_variable('range.front')
        self._lg_stab.add_variable('range.back')
        self._lg_stab.add_variable('range.left')
        self._lg_stab.add_variable('range.right')
        self._lg_stab.add_variable('range.zrange')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        # self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
        t = Timer(300, self._cf.close_link)
        t.start()

    def _stab_log_error(self, logconf, msg):

        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        global sensor_data
        """Callback from a the log API when data arrives"""
        #print(f'[{timestamp}][{logconf.name}]: ', end='')
        sensor_data['range_front'] = np.float32(data['range.front'])/1300 	
        sensor_data['range_back'] = np.float32(data['range.back'])/1300
        sensor_data['range_left'] = np.float32(data['range.left'])/1300
        sensor_data['range_right'] = np.float32(data['range.right'])/1300
        sensor_data['x_global'] = data['stateEstimate.x'] + 0.15
        sensor_data['y_global'] = data['stateEstimate.y'] + 1.5
        sensor_data['range_down'] = np.float32(data['range.zrange'])/1000
        sensor_data['yaw'] = data['stabilizer.yaw']*np.pi/180.0
        """
        for name, value in data.items():
            print(f'{name}: {value:3.3f} ', end='')
        print()
        """


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    le = LoggingExample(uri)
    cf = le._cf

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    #while le.is_connected:
    drone_controller = DroneController()
    startpos = None
    while le.is_connected:

        get_command(sensor_data)
        """
        time.sleep(0.01)
        for y in range(10):
            cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
            time.sleep(0.1)

        for _ in range(20):
            cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
            time.sleep(0.1)

        for _ in range(50):
            cf.commander.send_hover_setpoint(0.5, 0, 36 * 2, 0.4)
            time.sleep(0.1)

        for _ in range(50):
            cf.commander.send_hover_setpoint(0.5, 0, -36 * 2, 0.4)
            time.sleep(0.1)

        for _ in range(20):
            cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
            time.sleep(0.1)

        for y in range(10):
            cf.commander.send_hover_setpoint(0, 0, 0, (10 - y) / 25)
            time.sleep(0.1)

        cf.commander.send_stop_setpoint()
        """




