from grid import Node, NodeGrid
from math import inf
import heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """

    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        costs = [[inf for _ in range(self.node_grid.width)] for _ in range(self.node_grid.height)]
        pq = [(0., self.node_grid.get_node(start_position[0], start_position[1]), None)]
        last = None
        while pq:
            cost, node, parent = pq[0]
            heapq.heappop(pq)
            if not node.closed:

                node.closed = True
                node.parent = parent
                costs[node.i][node.j] = cost

                if node.get_position() == goal_position:
                    last = node
                    break

                for sucessor in self.node_grid.get_successors(node.i, node.j):
                    s_node = self.node_grid.get_node(sucessor[0], sucessor[1])
                    if not s_node.closed and costs[s_node.i][s_node.j] > cost + self.cost_map.get_edge_cost(
                            node.get_position(), s_node.get_position()):
                        node.f = cost + self.cost_map.get_edge_cost(node.get_position(),
                                                                    s_node.get_position())
                        costs[s_node.i][s_node.j] = node.f
                        heapq.heappush(pq, (node.f, s_node, node))

        path = []
        path_cost = costs[goal_position[0]][goal_position[1]]

        while last is not None:
            path.append(last.get_position())
            last = last.parent

        self.node_grid.reset()
        return path, path_cost

    def greedy(self, start_position, goal_position):
        """
        Plans a path using greedy search.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        cost = start_node.distance_to(goal_position[0], goal_position[1])
        pq = [(cost, start_node, None)]
        last = None

        while pq:
            cost, node, parent = pq[0]
            heapq.heappop(pq)
            if not node.closed:
                node.closed = True
                node.parent = parent

                if node.get_position() == goal_position:
                    last = node
                    break

                for sucessor in self.node_grid.get_successors(node.i, node.j):
                    s_node = self.node_grid.get_node(sucessor[0], sucessor[1])
                    if not s_node.closed:
                        s_cost = s_node.distance_to(goal_position[0], goal_position[1])
                        heapq.heappush(pq, (s_cost, s_node, node))

        path = []
        path_cost = 0

        while last is not None:
            path.append(last.get_position())
            if last.parent is not None:
                path_cost += self.cost_map.get_edge_cost(last.get_position(), last.parent.get_position())
            last = last.parent

        self.node_grid.reset()
        return path, path_cost

    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Todo: implement the A* algorithm
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        self.node_grid.reset()
        return [], inf
