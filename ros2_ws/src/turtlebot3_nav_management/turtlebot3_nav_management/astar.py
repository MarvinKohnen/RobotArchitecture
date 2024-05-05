import numpy as np
import heapq

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost to reach this node
        self.h = 0  # Heuristic cost to goal
        self.f = 0  # Total cost (g + h)

    def __lt__(self, other):
        return self.f < other.f

def astar(map_array, start, goal):
    def heuristic(a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    open_set = []
    heapq.heappush(open_set, Node(start))
    closed_set = set()

    while open_set:
        current_node = heapq.heappop(open_set)

        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]  # Return reversed path

        closed_set.add(current_node.position)

        for direction in [(0, 1), (1, 0), (0, -1), (-1, 0)]:  # 4-way connectivity
            neighbor_pos = (current_node.position[0] + direction[0], current_node.position[1] + direction[1])

            if 0 <= neighbor_pos[0] < map_array.shape[0] and 0 <= neighbor_pos[1] < map_array.shape[1]:
                if map_array[neighbor_pos[0], neighbor_pos[1]] != 0:  # Obstacle
                    continue

                if neighbor_pos in closed_set:
                    continue

                neighbor_node = Node(neighbor_pos, current_node)
                neighbor_node.g = current_node.g + 1
                neighbor_node.h = heuristic(neighbor_node.position, goal)
                neighbor_node.f = neighbor_node.g + neighbor_node.h

                if not any(node.position == neighbor_node.position and node.f <= neighbor_node.f for node in open_set):
                    heapq.heappush(open_set, neighbor_node)

    return None  # No path found
