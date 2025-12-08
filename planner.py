import heapq
import math
import time

class RoutePlanner:
    def __init__(self, airspace, config=None):
        self.env = airspace
        cfg = config or {}
        self.COST_CLIMB = cfg.get('climb_cost', 2.0)
        self.COST_DESCEND = cfg.get('descend_cost', 0.5)
        self.COST_HOVER = cfg.get('hover_cost', 1.0)
        self.WIND_PENALTY = cfg.get('wind_penalty', 2.0)
        self.LOW_FLYING_PENALTY = cfg.get('safety_penalty', 10.0)

    def heuristic(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def _segment_components(self, current, next_node):
        dx = next_node[0] - current[0]
        dy = next_node[1] - current[1]
        dz = next_node[2] - current[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        base = self.COST_HOVER
        if dz > 0:
            base = self.COST_CLIMB
        elif dz < 0:
            base = self.COST_DESCEND
        horiz_energy = dist
        climb_energy = base * dist

        # wind vector at current cell (assumes env.wind_field is valid and same shape)
        wx, wy, wz = (0.0, 0.0, 0.0)
        try:
            w = self.env.wind_field[current[0], current[1], current[2]]
            wx, wy, wz = float(w[0]), float(w[1]), float(w[2])
        except Exception:
            wx, wy, wz = 0.0, 0.0, 0.0

        # wind effect projected onto movement vector (opposing component increases cost)
        wind_effect = -(wx*dx + wy*dy + wz*dz)
        wind_cost = max(0.0, wind_effect) * self.WIND_PENALTY

        safety_cost = 0.0
        if next_node[2] < 3:
            safety_cost = self.LOW_FLYING_PENALTY * dist

        total = horiz_energy + climb_energy + wind_cost + safety_cost

        breakdown = {
            "horiz": float(horiz_energy),
            "climb": float(climb_energy),
            "wind": float(wind_cost),
            "safety": float(safety_cost),
            "total": float(total)
        }
        return total, breakdown

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def find_path(self, start, goal):
        """
        A* with 26-connectivity.
        Returns: (path_list or None, metrics_dict)
        metrics: {'time_s', 'cost', 'nodes_explored', 'breakdown'}
        """
        t0 = time.time()

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0.0}

        # track breakdown per node (cumulative components to reach that node)
        breakdown_so_far = {start: {"horiz":0.0, "climb":0.0, "wind":0.0, "safety":0.0}}

        neighbors = [
            (dx, dy, dz)
            for dx in [-1, 0, 1]
            for dy in [-1, 0, 1]
            for dz in [-1, 0, 1]
            if not (dx == 0 and dy == 0 and dz == 0)
        ]

        nodes_explored = 0
        visited = set()

        while open_set:
            _, current = heapq.heappop(open_set)

            # optional small optimization: skip already processed nodes
            if current in visited:
                continue
            visited.add(current)
            nodes_explored += 1

            if current == goal:
                path = self.reconstruct_path(came_from, current)
                total_cost = g_score.get(current, float('inf'))
                breakdown = breakdown_so_far.get(current, {"horiz":0,"climb":0,"wind":0,"safety":0})
                t1 = time.time()
                metrics = {
                    "time_s": float(t1 - t0),
                    "cost": float(total_cost),
                    "nodes_explored": int(nodes_explored),
                    "breakdown": {k: float(v) for k,v in breakdown.items()}
                }
                return path, metrics

            for dx, dy, dz in neighbors:
                neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)

                if not self.env.is_valid(neighbor):
                    continue

                move_cost, seg_break = self._segment_components(current, neighbor)
                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    # accumulate breakdown
                    prev_break = breakdown_so_far.get(current, {"horiz":0.0,"climb":0.0,"wind":0.0,"safety":0.0})
                    breakdown_so_far[neighbor] = {
                        "horiz": prev_break["horiz"] + seg_break["horiz"],
                        "climb": prev_break["climb"] + seg_break["climb"],
                        "wind": prev_break["wind"] + seg_break["wind"],
                        "safety": prev_break["safety"] + seg_break["safety"]
                    }

                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

        # no path found
        t1 = time.time()
        metrics = {
            "time_s": float(t1 - t0),
            "cost": -1.0,
            "nodes_explored": int(nodes_explored),
            "breakdown": {"horiz":0.0,"climb":0.0,"wind":0.0,"safety":0.0}
        }
        return None, metrics
