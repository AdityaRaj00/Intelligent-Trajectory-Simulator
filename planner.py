import heapq
import math

class RoutePlanner:
    def __init__(self, airspace, config={}):
        self.env = airspace
        
        # Load User Settings
        self.COST_CLIMB = config.get('climb_cost', 2.0)
        self.COST_DESCEND = config.get('descend_cost', 0.5)
        self.COST_HOVER = config.get('hover_cost', 1.0)
        self.WIND_PENALTY = config.get('wind_penalty', 2.0)
        self.LOW_FLYING_PENALTY = config.get('safety_penalty', 10.0)

    def heuristic(self, a, b):
        """Euclidean distance heuristic."""
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def get_energy_cost(self, current, next_node):
        """Calculates energy consumption based on movement physics."""
        dx = next_node[0] - current[0]
        dy = next_node[1] - current[1]
        dz = next_node[2] - current[2]
        
        # Calculate Euclidean distance of the move (1.0 for straight, ~1.41 for diagonal)
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        # 1. Base Mechanical Cost (Climb vs Descend vs Hover)
        base_cost = self.COST_HOVER
        if dz > 0:
            base_cost = self.COST_CLIMB
        elif dz < 0:
            base_cost = self.COST_DESCEND 
        
        # Multiply base cost by distance (longer moves cost more)
        total_cost = base_cost * dist

        # 2. Wind Resistance
        # We project the wind vector onto our movement vector
        wind_vector = self.env.wind_field[current[0], current[1], current[2]]
        # Dot product to find how much wind is opposing us
        wind_effect = -(wind_vector[0]*dx + wind_vector[1]*dy + wind_vector[2]*dz)
        
        if wind_effect > 0: 
            total_cost += wind_effect * self.WIND_PENALTY
            
        # 3. Ground Proximity Safety Penalty
        if next_node[2] < 3:
            total_cost += self.LOW_FLYING_PENALTY * dist
            
        return total_cost

    def find_path(self, start, goal):
        """A* Algorithm implementation with 26-connectivity (Diagonals)."""
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        
        # Generate all 26 neighbors (dxdydz from -1 to 1, excluding 0,0,0)
        neighbors = [
            (dx, dy, dz) 
            for dx in [-1, 0, 1] 
            for dy in [-1, 0, 1] 
            for dz in [-1, 0, 1] 
            if not (dx == 0 and dy == 0 and dz == 0)
        ]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy, dz in neighbors:
                neighbor = (current[0]+dx, current[1]+dy, current[2]+dz)
                
                if not self.env.is_valid(neighbor):
                    continue

                # Calculate cost
                move_cost = self.get_energy_cost(current, neighbor)
                tentative_g_score = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
        
        return None 

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]