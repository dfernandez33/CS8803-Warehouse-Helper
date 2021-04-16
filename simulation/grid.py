import json
from setting import *
from utils import readCoordinatesFromTxt
import random
random.seed(RANDOM_SEED)
from shapely import geometry

SCALE = 50
# grid map class


class CozGrid:

    def __init__(self, fname):
        orientation = {'U': 0.0, 'L': 90.0, 'D': 180.0, 'R': 270.0}
        with open(fname) as configfile:
            config = json.loads(configfile.read())
            self.width = config['width']//SCALE
            self.height = config['height']//SCALE
            self.scale = config['scale']

            self.occupied = []
            self.markers = []

            for entry in config["layout"]:
                print(entry)
                self.markers.append((entry[0], entry[1], entry[2]))

            obs = readCoordinatesFromTxt('obstacle_points.txt')
            self.obstacle = geometry.Polygon([[float(p[0])/SCALE, float(p[1])/SCALE] for p in obs])
            self.obstaclePoints = [[float(p[0])/SCALE, float(p[1])/SCALE] for p in obs]

    def is_in(self, x, y):
        """ Determine whether the cell is in the grid map or not
            Argument:
            x, y - X and Y in the cell map
            Return: boolean results
        """
        xScale = x/SCALE
        yScale = y/SCALE
        point = geometry.Point(xScale,yScale)

        if xScale < 0 or yScale < 0 or xScale > self.width or yScale > self.height or self.obstacle.contains(point):

            return False
        return True

    def is_free(self, x, y):
        """ Determine whether the cell is in the *free part* of grid map or not
            Argument:
            x, y - X and Y in the cell map
            Return: boolean results
        """
        if not self.is_in(x, y):
            return False
        yy = int(y)  # self.height - int(y) - 1
        xx = int(x)
        return (xx, yy) not in self.occupied

    def random_place(self):
        """ Return a random place in the map
            Argument: None
            Return: x, y - X and Y in the cell map
        """
        x = random.uniform(0, self.width)
        y = random.uniform(0, self.height)
        return x, y

    def random_free_place(self):
        """ Return a random place in the map which is free from obstacles
            Argument: None
            Return: x, y - X and Y in the cell map
        """
        while True:
            x, y = self.random_place()
            if self.is_free(x, y):
                return x, y


# parse marker position and orientation
# input: grid position and orientation char from JSON file
# output: actual marker position (marker origin) and marker orientation
def parse_marker_info(col, row, heading_char):
    if heading_char == 'U':
        c = col + 0.5
        r = row
        heading = 90
    elif heading_char == 'D':
        c = col + 0.5
        r = row + 1
        heading = 270
    elif heading_char == 'L':
        c = col + 1
        r = row + 0.5
        heading = 180
    elif heading_char == 'R':
        c = col
        r = row + 0.5
        heading = 0
    return c, r, heading
