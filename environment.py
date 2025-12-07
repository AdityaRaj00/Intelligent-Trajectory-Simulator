import numpy as np
import math

class Airspace:
    def __init__(self, size=(30, 30, 20)):
        self.size = size
        self.grid = np.zeros(size, dtype=int)
        # Wind field: 3D grid where every point has a 3D vector (u, v, w)
        self.wind_field = np.zeros(size + (3,)) 

    def set_wind(self, z_level=10, intensity=2.0, direction_degrees=180):
        """
        Sets wind vector field based on intensity and direction.
        z_level: Altitude where wind starts
        intensity: Speed of wind
        direction_degrees: 0=East, 90=North, 180=West, 270=South
        """
        # Convert degrees to radians for math
        rad = math.radians(direction_degrees)
        
        # Calculate X and Y components of the wind vector
        u_wind = intensity * math.cos(rad) 
        v_wind = intensity * math.sin(rad)
        
        self.wind_field[:, :, z_level:, 0] = u_wind
        self.wind_field[:, :, z_level:, 1] = v_wind

    def is_valid(self, node):
        x, y, z = node
        if not (0 <= x < self.size[0] and 0 <= y < self.size[1] and 0 <= z < self.size[2]):
            return False
        if self.grid[x, y, z] == 1:
            return False
        return True