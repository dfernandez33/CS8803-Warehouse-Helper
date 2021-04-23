from typing import Tuple
from math import sqrt, inf
import networkx


class PathPlanner:

    HIGHWAY_WAYPOINTS = [(500, 500, 0), (500, 1000, 0), (500, 1574, 0), (1500, 1574, 0),
                         (2500, 1574, 0), (2500, 1000, 0), (2500, 500, 0), (1500, 500, 0)]

    def __init__(self):
        self.highway_graph = self.__build_graph()

    def get_optimal_path(self, start: Tuple, goal: Tuple):
        nearest_entry_point = self.__get_nearest_waypoint(start)
        nearest_exit_point = self.__get_nearest_waypoint(goal)
        highway_path = networkx.shortest_path(self.highway_graph, nearest_entry_point,
                                              nearest_exit_point, weight='weight')

        optimal_path = highway_path
        optimal_path.insert(0, start)
        optimal_path.append(goal)

        return optimal_path

    def __build_graph(self) -> networkx.Graph:
        highway_graph = networkx.Graph()
        highway_graph.add_nodes_from(self.HIGHWAY_WAYPOINTS)
        edge_bunch = []
        for i in range(len(self.HIGHWAY_WAYPOINTS)):
            if i == len(self.HIGHWAY_WAYPOINTS) - 1:
                curr_source = self.HIGHWAY_WAYPOINTS[i]
                curr_dest = self.HIGHWAY_WAYPOINTS[0]
                edge_bunch.append((curr_source, curr_dest,
                                   {'weight': self.__euclidean_dist(curr_source, curr_dest)}))
            else:
                curr_source = self.HIGHWAY_WAYPOINTS[i]
                curr_dest = self.HIGHWAY_WAYPOINTS[i+1]
                edge_bunch.append((curr_source, curr_dest,
                                   {'weight': self.__euclidean_dist(curr_source, curr_dest)}))
        highway_graph.add_edges_from(edge_bunch)

        return highway_graph

    def __get_nearest_waypoint(self, location: Tuple) -> Tuple:
        nearest_point = None
        nearest_dist = inf
        for waypoint in self.HIGHWAY_WAYPOINTS:
            curr_dist = self.__euclidean_dist(location, waypoint)
            if nearest_dist > curr_dist:
                nearest_point = waypoint
                nearest_dist = curr_dist

        return nearest_point

    @staticmethod
    def __euclidean_dist(x1: Tuple, x2: Tuple) -> float:
        return sqrt((x2[0] - x1[0]) ** 2 + (x2[1] - x1[1]) ** 2)


if __name__ == '__main__':
    path_planner = PathPlanner()
    start_location = (300, 600, 0)
    goal_location = (2057, 1651, 90)
    best_path = path_planner.get_optimal_path(start_location, goal_location)
    print(best_path)
