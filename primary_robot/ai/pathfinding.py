import math
import numpy as np

GRAPH_FILE = "graph.txt"
TABLE_HEIGHT = 2000
TABLE_WIDTH = 3000
GRAPH_TABLE_RATIO = 0.2  # 1 / 5 -> ratio between the graph size and the actual table size in the table frame


class PathFinding:
    def __init__(self, robot):
        self.robot = robot

    def find_path(self, start, goal):
        raise NotImplementedError("This is an abstract class, must be implemented before use.")

class Node:
    def __init__(self, value, x, y):
        self.value = value
        self.x = x
        self.y = y
        self.parent = None
        self.H = 0
        self.G = 0

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def heuristic(self, other):
        return self.manhattan_distance(other)

    def distance(self, other):
        return math.sqrt((other.x - self.x) ** 2 + (other.y - self.y) ** 2)

    def cost(self, other):
        return self.distance(other)

    def manhattan_distance(self, other):
        return abs(other.x - self.x) + abs(other.y - self.y)

    def __hash__(self):
        return self.x.__hash__() + self.y.__hash__()

    def __str__(self):
        return "x:{} y:{} h:{} g:{}".format(self.x, self.y, self.H, self.G)

class ThetaStar(PathFinding):
    def __init__(self, robot):
        super().__init__(robot)
        self.graph = None
        self.load_graph(GRAPH_FILE)
        self.height = len(self.graph[0])
        self.width = len(self.graph)

    def load_graph(self, file):
        self.graph = np.empty((int(TABLE_WIDTH * GRAPH_TABLE_RATIO), int(TABLE_HEIGHT * GRAPH_TABLE_RATIO)), dtype=object)
        with open(file, 'r') as f:
            nb_line = 0
            for j, line in enumerate(f):
                nb_line += 1
                for i, pt in enumerate(line.strip()):
                    value = True if pt == '0' else False
                    self.graph[i,j] = Node(value, i, j)
            # self.graph = list(reversed(self.graph))
            f.close()

    def find_path(self, start, goal):
        opened = set()
        closed = set()
        start_node = self.graph[int(start[0] * GRAPH_TABLE_RATIO)][int(start[1] * GRAPH_TABLE_RATIO)]
        goal_node = self.graph[int(goal[0] * GRAPH_TABLE_RATIO)][int(goal[1] * GRAPH_TABLE_RATIO)]
        start_node.H = start_node.heuristic(goal_node)
        opened.add(start_node)
        while len(opened) != 0:
            s = min(opened, key=lambda n: n.G + n.H)
            opened.remove(s)
            if s == goal_node:
                path = []
                s_path = s
                while s_path.parent is not None:
                    path.append((s_path.x // GRAPH_TABLE_RATIO, s_path.y // GRAPH_TABLE_RATIO))
                    s_path = s_path.parent
                print("Path found")
                return list(reversed(path))
            closed.add(s)
            for s_2 in self.neighbours(s):
                if s_2 not in closed:
                    if s_2 not in opened:
                        s_2.G = -1
                        s_2.parent = None
                    self.update_node(s, s_2, opened, goal_node)
        print("No Path found")
        return

    def update_node(self, s, s_2, opened, goal_node):
        g_old = s_2.G
        self.compute_cost(s, s_2)
        if s_2.G < g_old:
            opened.remove(s_2)
        s_2.H = s_2.heuristic(goal_node)
        opened.add(s_2)

    def compute_cost(self,s, s_2):
        if s.parent is not None:
            if self.line_of_sight(s.parent, s_2):
                if s.parent.G + s_2.cost(s.parent) < s_2.G or s_2.G == -1:
                    s_2.parent = s.parent
                    s_2.G = s.parent.G + s_2.cost(s.parent)
        if s.G + s_2.cost(s) < s_2.G or s_2.G == -1:
            s_2.parent = s
            s_2.G = s.G + s_2.cost(s)

    def neighbours(self, s):
        neighbours = []
        if s.x > 0 and self.graph[s.x - 1][s.y].value:
            neighbours.append(self.graph[s.x - 1][s.y])
        if s.x < self.width - 1 and self.graph[s.x + 1][s.y].value:
            neighbours.append(self.graph[s.x + 1][s.y])
        if s.y > 0 and self.graph[s.x][s.y - 1].value:
            neighbours.append(self.graph[s.x][s.y - 1])
        if s.y < self.height - 1 and self.graph[s.x][s.y + 1].value:
            neighbours.append(self.graph[s.x][s.y + 1])
        return neighbours

    def line_of_sight(self, s, s_2):
        x0 = s.x
        y0 = s.y
        x1 = s_2.x
        y1 = s_2.y
        dy = y1 - y0
        dx = x1 - x0
        f = 0
        if dy < 0:
            dy = -dy
            sy = -1
        else:
            sy = 1
        if dx < 0:
            dx = -dx
            sx = -1
        else:
            sx = 1
        if dx >= dy:
            while x0 != x1:
                f = f + dy
                if f >= dx:
                    if not self.graph[x0 + (sx - 1) // 2][y0 + (sy - 1) // 2].value:
                        return False
                    y0 = y0 + sy
                    f = f - dx
                if f != 0 and not self.graph[x0 + (sx - 1) // 2][y0 + (sy - 1) // 2].value:
                    return False
                if dy == 0 and not self.graph[x0 + (sx - 1) // 2][y0].value and not self.graph[x0 + (sx - 1) // 2][y0 - 1].value:
                    return False
                x0 = x0 + sx
        else:
            while y0 != y1:
                f = f + dx
                if f >= dy:
                    if not self.graph[x0 + (sx - 1)//2][y0 + (sy - 1)//2].value:
                        return False
                    x0 = x0 + sx
                    f = f - dy
                if f != 0 and not self.graph[x0 + (sx - 1)//2][y0 + (sy - 1)//2].value:
                    return False
                if dx == 0 and not self.graph[x0][y0 + (sy - 1)//2].value and not self.graph[x0 - 1][y0 + (sy - 1)//2].value:
                    return False
                y0 = y0 + sy
        return True



