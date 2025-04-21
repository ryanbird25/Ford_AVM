import numpy as np
import open3d as o3d
import heapq



class OccupancyGrid():

    def __init__(self, point_cloud):
        self.width = 170#cm
        self.length = 235#cm
        self.resolution = .5#cm/grid
        self.grid = None
        self.update_grid(self, point_cloud)

    def update_grid(self, point_cloud):
        # Update the occupancy grid based on the point cloud
        self.grid = np.zeros((self.length/self.resolution, self.width/self.resolution), dtype=np.int8)
        for point in point_cloud:
            x, y = int(point[0] / self.resolution), int(point[1] / self.resolution)
            if 0 <= x < self.grid.shape[0] and 0 <= y < self.grid.shape[1]:
                self.grid[x, y] = 1

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

    def plan(self, start, goal):
        open_heap = []
        heapq.heappush(open_heap, (0 + self.heuristic(start, goal), 0, start))  # (f, g, position)

        came_from = {}
        g_score = {start: 0}
        visited = set()

        while open_heap:
            f, g, current = heapq.heappop(open_heap)

            if current in visited:
                continue

            visited.add(current)

            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                neighbor = (current[0] + dx, current[1] + dy)

                if (0 <= neighbor[0] < self.rows and
                    0 <= neighbor[1] < self.cols and
                    self.grid[neighbor] == 0):

                    tentative_g = g + 1
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        g_score[neighbor] = tentative_g
                        f_score = tentative_g + self.heuristic(neighbor, goal)
                        heapq.heappush(open_heap, (f_score, tentative_g, neighbor))
                        came_from[neighbor] = current

        return None  # No path found
    
    def remove_collinear(self, plan):
        pass
