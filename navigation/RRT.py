from typing import Tuple, List, Set
from shapely.geometry import LineString
import math
import random
import networkx
import heapq
import pygame
import sys


class RRT:

    # These class variables are used for drawing in pygame
    SCREEN_SIZE = (1164, 800)
    SCALE_FACTOR = (1164 / 2921, 800 / 2057.4)  # since window is size 800, 800 and the environment is 2921mm x2.0066mm
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    PURPLE = (128, 0, 128)
    RED = (255, 0, 0)

    def __init__(self, obstacles_file: str, robot_radius: float, arena_width: float,
                 arena_height: float, num_iteration=1000, display=False):
        self.obstacles_file = obstacles_file
        self.num_iterations = num_iteration
        self.robot_radius = robot_radius
        self.arena_width = arena_width
        self.arena_height = arena_height
        self.obstacles = self.__get_obstacles()
        self.vertices = set()
        self.edges = set()
        self.display = display
        if display:
            self.screen = pygame.display.set_mode(self.SCREEN_SIZE)

    def get_optimal_path(self, start: Tuple, goal: Tuple, distance_tolerance=50) -> List[Tuple]:
        """
        Provides the optimal path from start to goal. All inputs should be in the global coordinate frame. Output
        waypoints are in the global coordinate frame.
        :param start: tuple with x,y coordinates of starting location (in mm).
        :param goal: tuple with x,y coordinate of goal location (in mm).
        :param display: whether to display the resulting tree or not
        :param distance_tolerance: how close to the goal the path should be to.
        :return: List of waypoints representing the path.
        """
        if self.display:
            print('Initializing Display')
            self.__init_display(start, goal)

        self.__build_tree(start)
        graph = self.__build_graph(self.vertices, self.edges)
        potential_goals = []
        for vertex in self.vertices:  # find all vertices within 5m of goal
            if self.__euclidean_dist(vertex, goal) < distance_tolerance:
                potential_goals.append(vertex)

        path_queue = []
        for potential_goal in potential_goals:  # find shortest path among potential goals
            path = networkx.shortest_path(graph, start, potential_goal, weight='weight')
            cost = networkx.path_weight(graph, path, weight='weight')
            heapq.heappush(path_queue, (cost, path))
        if len(path_queue) == 0:
            return []

        best_path = heapq.heappop(path_queue)[1]
        if self.display:
            self.__draw_optimal_path(best_path)
        return best_path

    def __build_tree(self, start_position: Tuple):
        """
        This function implements the RRT algorithm. For more info look it up online, should be straight forward.
        """
        self.vertices.add(start_position)
        for i in range(self.num_iterations):
            print('Running iteration {}'.format(i))
            x_rand = (random.uniform(self.robot_radius, self.arena_width - self.robot_radius),
                      random.uniform(self.robot_radius, self.arena_height - self.robot_radius))
            x_nearest, distance = self.__get_nearest_vertex(x_rand)
            if self.__obstacle_free(x_nearest, x_rand):
                self.vertices.add(x_rand)
                self.edges.add((x_nearest, x_rand, distance))
                if self.display:
                    self.__draw_connections((x_nearest, x_rand))

    def __obstacle_free(self, start: Tuple, end: Tuple) -> bool:
        for obstacle in self.obstacles:
            x, y, w, h = obstacle
            rect_coord = ((x - self.robot_radius, y + self.robot_radius),
                          (x + w + self.robot_radius, y + self.robot_radius),
                          (x - self.robot_radius, y - h - self.robot_radius),
                          (x + w + self.robot_radius, y - h - self.robot_radius))
            rect_top_left = rect_coord[0]
            rect_bottom_right = rect_coord[3]
            if (rect_top_left[0] <= end[0] <= rect_bottom_right[0] and  # check if point is in obstacle
                    rect_bottom_right[1] <= end[1] <= rect_top_left[1]):
                return False
            new_edge = LineString([end, start])
            if new_edge.intersects(LineString([rect_coord[0], rect_coord[2]])) \
                    or new_edge.intersects(LineString([rect_coord[0], rect_coord[1]])) \
                    or new_edge.intersects(LineString([rect_coord[1], rect_coord[3]])) \
                    or new_edge.intersects(LineString([rect_coord[2], rect_coord[3]])):
                return False
        return True

    def __get_nearest_vertex(self, x_rand: Tuple[float, float]) -> Tuple:
        nearest_dist = math.inf
        nearest_vertex = None
        for v in self.vertices:
            distance = self.__euclidean_dist(x_rand, v)
            if distance < nearest_dist:
                nearest_dist = distance
                nearest_vertex = v
        return nearest_vertex, nearest_dist

    def __get_obstacles(self) -> List[Tuple]:
        obstacles = []
        obstacles_file = open(self.obstacles_file)
        for obstacle in obstacles_file.readlines():
            obstacles.append(tuple([float(x) for x in obstacle.split(",")]))

        return obstacles

    @staticmethod
    def __build_graph(vertices: Set, edges: Set) -> networkx.Graph:
        graph = networkx.Graph()
        graph.add_nodes_from(vertices)
        ebunch = []
        for edge in edges:  # add weight as dictionary for use in networkx
            ebunch.append((edge[0], edge[1], {'weight': edge[2]}))
        graph.add_edges_from(ebunch)
        return graph

    @staticmethod
    def __euclidean_dist(x1: Tuple, x2: Tuple) -> float:
        return math.sqrt((x2[0] - x1[0]) ** 2 + (x2[1] - x1[1]) ** 2)

    ######################################################################
    # All the functions below deal with drawing in pygame, no RRT logic. #
    ######################################################################
    def __init_display(self, start: Tuple, goal: Tuple):
        self.screen.fill(self.WHITE)
        scaled_start = (start[0] * self.SCALE_FACTOR[0], start[1] * self.SCALE_FACTOR[1])
        scaled_goal = (goal[0] * self.SCALE_FACTOR[0], goal[1] * self.SCALE_FACTOR[1])
        pygame.draw.circle(self.screen, self.GREEN, [int(scaled_start[0]),int(self.SCREEN_SIZE[1] - scaled_start[1])],
                           int(76 * self.SCALE_FACTOR[0]), 0)
        pygame.draw.circle(self.screen, self.BLUE, [int(scaled_goal[0]), int(self.SCREEN_SIZE[1] - scaled_goal[1])],
                           int(76 * self.SCALE_FACTOR[0]), 0)
        for obstacle in self.obstacles:
            pygame.draw.rect(self.screen, self.BLACK, (obstacle[0] * self.SCALE_FACTOR[0],
                                                       self.SCREEN_SIZE[1] - (obstacle[1] * self.SCALE_FACTOR[1]),
                                                       obstacle[2] * self.SCALE_FACTOR[0],
                                                       obstacle[3] * self.SCALE_FACTOR[1]), 0)

        pygame.display.update()
    
    def __draw_connections(self, new_edge: Tuple[Tuple[float, float], Tuple[float, float]], color=PURPLE):
        scaled_e = ((new_edge[0][0] * self.SCALE_FACTOR[0], self.SCREEN_SIZE[1] - (new_edge[0][1] * self.SCALE_FACTOR[1])),
                    (new_edge[1][0] * self.SCALE_FACTOR[0], self.SCREEN_SIZE[1] - (new_edge[1][1] * self.SCALE_FACTOR[1])))
        pygame.draw.lines(self.screen, color, True, list(scaled_e), 5)
        pygame.display.update()

    def __draw_optimal_path(self, path: List[Tuple]):
        for i in range(len(path) - 1):
            curr_edge = (path[i], path[i + 1])
            self.__draw_connections(curr_edge, self.RED)


if __name__ == '__main__':
    rrt = RRT('obstacles.txt', 165.1, 2921, 2057.4, num_iteration=3000, display=False)
    optimal_path = rrt.get_optimal_path((500, 500), (2500, 1600))
    print(optimal_path)
