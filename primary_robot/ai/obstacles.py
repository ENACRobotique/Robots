from enum import Enum
import yaml


def load_obstacles_from_file(file):
    obstacles_object = []
    with open(file, "r") as f:
        obstacles = yaml.load(f)
    for obstacle in obstacles['obstacles']:
        for type, attributes in obstacle.items():
            if type == 'circle':
                circle = Circle(len(obstacles_object))
                circle.center = (attributes['center']['x'], attributes['center']['y'])
                circle.radius = attributes['radius']
                obstacles_object.append(circle)
            elif type == 'polygon':
                polygon = Polygon(len(obstacles_object))
                for pt in attributes['points']:
                    polygon.points.append((pt['x'], pt['y']))
                obstacles_object.append(polygon)
    return obstacles_object

class ObstacleType(Enum):
    POLYGON = 0
    CIRCLE = 1


class Obstacle:
    def __init__(self, id, type):
        self.id = id
        self.type = type

    def ivy_message(self):
        raise NotImplementedError("This is an abstract class")


class Polygon(Obstacle):
    def __init__(self, id):
        super().__init__(id, ObstacleType.POLYGON)
        self.points = [] # expected [(x0, y0), (x1, y1), ...]

    def ivy_message(self):
        points = ""
        for pt in self.points:
            points += "{},{};".format(pt[0], pt[1])
        return "id : {} type : POLYGON points : {}".format(self.id, points[:-1])



class Circle(Obstacle):
    def __init__(self, id):
        super().__init__(id, ObstacleType.CIRCLE)
        self.center = None  # (xc, yc)
        self.radius = None

    def ivy_message(self):
        return "id : {} type : CIRCLE center : {},{} radius : {}".format(self.id, self.center[0], self.center[1], self.radius)
