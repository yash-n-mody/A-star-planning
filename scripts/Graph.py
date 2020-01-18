from __future__ import print_function
from collections import deque
import copy
import math

class Node():
    def __init__(self):
        self.g = -1
        self.h = -1
        self.Weights = {}
        self.Neighbours = []
        self.Successors = []
        self.Parent = 0
        self.Visited = False
    
    def f(self):
        return self.g + self.h

class WeightedGraph():
    def __init__(self):
        self.Nodes = {}

class OccupancyGraph():
    def __init__(self, map):
        self.length = len(map)
        self.breadth = len(map[0])
        self.Graph = BuildGraph(map)

    def Print(self):
        for i in range(self.length):
            for j in range(self.breadth):
                if (i, j) in self.Graph.Nodes:
                    print("{0:2.1f}:{1:<2.1f}".format(self.Graph.Nodes[(i, j)].g, self.Graph.Nodes[(i, j)].h), end=' ')
                else:
                    print('     ', end=' ')
            print()

    def UnvisitAllNodes(self):
        for i in range(self.length):
            for j in range(self.breadth):
                if (i, j) in self.Graph.Nodes:
                    self.Graph.Nodes[(i,j)].Visited = False

    def WaveFormExpansion(self, x, y, mode='g'):
        # closed set is empty
        closed = []
        # next queue has one element [x, y]
        next_queue = deque([ self.Graph.Nodes[(x, y)] ])
        for node in next_queue:
            node.Visited = True
        # current weight is zero
        current_weight = 0
        # while next queue is not empty
        while len(next_queue) != 0:
        #   - current queue equals next queue
            current_queue = copy.copy(next_queue)
        #   - next queue is empty
            next_queue.clear()
        #   - while current queue is non empty
            while len(current_queue) != 0:
                node = current_queue.pop()
        #       -- set weight
                if mode == 'g':
                    node.g = current_weight
                elif mode == 'h':
                    node.h = current_weight
                else:
                    print('wrong mode.')
        #       -- add all neighbours not in (closed set + current queue) to next queue
                for neighbour in node.Neighbours:
                    if not neighbour.Visited:
                        next_queue.appendleft(neighbour)
                        neighbour.Visited = True
        #       -- deque each processed element from current queue and add it to closed set
                closed.append(node)
        #   - increment current weight
            current_weight = current_weight + 1
        self.UnvisitAllNodes()

    def SetStart(self, x, y):
        self.Start = self.Graph.Nodes[(x, y)]
        # self.WaveFormExpansion(x, y)

    def SetGoal(self, x, y):
        self.Goal = self.Graph.Nodes[(x, y)]
        # self.WaveFormExpansion(x, y, mode='h')

    # greedy implementation, with costs calculated beforehand - but diagonals do not have extra cost
    def FindGreedyPath(self):
        point = self.Graph.Nodes.keys()[self.Graph.Nodes.values().index(self.Start)]
        self.WaveFormExpansion(point[0], point[1])
        point = self.Graph.Nodes.keys()[self.Graph.Nodes.values().index(self.Goal)]
        self.WaveFormExpansion(point[0], point[1], mode='h')

        path = []
        # current node is start node
        current_node = self.Start
        current_node.Visited = True
        # while current node is not goal node
        while not current_node == self.Goal:
            candidates = []
            for neighbour in current_node.Neighbours:
                if (not neighbour.Visited):
                    candidates.append(neighbour)
            # look at all neighbours, find the minimum g + h
            min_f = 1000
            for candidate in candidates:
                if candidate.f() <= min_f:
                    min_f = candidate.f()
            for candidate in list(candidates):
                if not candidate.f() == min_f:
                    candidates.remove(candidate)
            # if more than one candidates, pick the one with the smallest h
            if len(candidates) > 1:
                min_h = 1000
                for candidate in candidates:
                    if candidate.h <= min_h:
                        min_h = candidate.h
                for candidate in list(candidates):
                    if not candidate.h == min_h:
                        candidates.remove(candidate)
            # if more than one candidates, pick the first one
            candidate = candidates.pop()
            # add that node to path
            path.append(self.Graph.Nodes.keys()[self.Graph.Nodes.values().index(candidate)])
            # print(path)
            # it becomes current node
            current_node = candidate
        return path

    def FindAStarPath(self):
        open_list = []
        closed_list = []

        self.Start.g = 0
        self.Start.h = self.GetHeuristic(self.Start)
        s_key = self.Graph.Nodes.keys()[self.Graph.Nodes.values().index(self.Start)]
        g_key = self.Graph.Nodes.keys()[self.Graph.Nodes.values().index(self.Goal)]
        open_list.append(s_key)

        while not len(open_list) == 0:
            # find node with minimum f in open_list
            min_f = 100000
            for key in open_list:
                if self.Graph.Nodes[key].f() <= min_f:
                    min_f = self.Graph.Nodes[key].f()
            index = -1
            for i in range(len(open_list)):
                if self.Graph.Nodes[open_list[i]].f() == min_f:
                    index = i
                    break
            q_key = open_list.pop(index)

            if q_key == g_key:
                path = self.ReconstructPath(s_key, q_key)
                return path

            q = self.Graph.Nodes[q_key]
            closed_list.append(q_key)

            # for each neighbour of q
            for neighbour in q.Neighbours:
                n_key = self.Graph.Nodes.keys()[self.Graph.Nodes.values().index(neighbour)]
                if n_key in closed_list:
                    continue

                tent_g = q.g + q.Weights[neighbour]
                
                if n_key not in open_list:
                    open_list.append(n_key)
                elif neighbour.g > 0 and tent_g >= neighbour.g:
                    continue

                # print(tent_g)
                neighbour.Parent = q
                neighbour.g = tent_g
                neighbour.h = self.GetHeuristic(neighbour)

    def ReconstructPath(self, s_key, q_key):
        path = []
        while not q_key == s_key:
            path.append(q_key)
            node = self.Graph.Nodes[q_key].Parent
            q_key = self.Graph.Nodes.keys()[self.Graph.Nodes.values().index(node)]
        path.append(s_key)
        # print(path)
        return path[::-1]

    def GetHeuristic(self, node):
        point1 = self.Graph.Nodes.keys()[self.Graph.Nodes.values().index(node)]
        point2 = self.Graph.Nodes.keys()[self.Graph.Nodes.values().index(self.Goal)]
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def BuildGraph(map):
    graph = WeightedGraph()
    # add all nodes
    for i in range(len(map)):
        for j in range(len(map[i])):
            if map[i][j] == 0:
                node = Node()
                graph.Nodes[(i, j)] = node

    # add all edges
    for i in range(len(map)):
        for j in range(len(map[i])):
            if not (i, j) in graph.Nodes:
                continue
            
            top = False
            bottom = False
            left = False
            right = False

            # directionals
            if (i, j - 1) in graph.Nodes:
                graph.Nodes[i, j].Neighbours.append(graph.Nodes[(i, j - 1)])
                graph.Nodes[i, j].Weights[graph.Nodes[(i, j - 1)]] = 1
                left = True
            if (i + 1, j) in graph.Nodes:
                graph.Nodes[i, j].Neighbours.append(graph.Nodes[(i + 1, j)])
                graph.Nodes[i, j].Weights[graph.Nodes[(i + 1, j)]] = 1
                bottom = True
            if (i, j + 1) in graph.Nodes:
                graph.Nodes[i, j].Neighbours.append(graph.Nodes[(i, j + 1)])
                graph.Nodes[i, j].Weights[graph.Nodes[(i, j + 1)]] = 1
                right = True
            if (i - 1, j) in graph.Nodes:
                graph.Nodes[i, j].Neighbours.append(graph.Nodes[(i - 1, j)])
                graph.Nodes[i, j].Weights[graph.Nodes[(i - 1, j)]] = 1
                top = True
            # diagonals
            if ((i + 1, j - 1) in graph.Nodes) and bottom and left:
                graph.Nodes[i, j].Neighbours.append(graph.Nodes[(i + 1, j - 1)])
                graph.Nodes[i, j].Weights[graph.Nodes[(i + 1, j - 1)]] = 1.4
            if ((i - 1, j + 1) in graph.Nodes) and top and right:
                graph.Nodes[i, j].Neighbours.append(graph.Nodes[(i - 1, j + 1)])
                graph.Nodes[i, j].Weights[graph.Nodes[(i - 1, j + 1)]] = 1.4
            if ((i - 1, j - 1) in graph.Nodes) and top and left:
                graph.Nodes[i, j].Neighbours.append(graph.Nodes[(i - 1, j - 1)])
                graph.Nodes[i, j].Weights[graph.Nodes[(i - 1, j - 1)]] = 1.4
            if ((i + 1, j + 1) in graph.Nodes) and bottom and right:
                graph.Nodes[i, j].Neighbours.append(graph.Nodes[(i + 1, j + 1)])
                graph.Nodes[i, j].Weights[graph.Nodes[(i + 1, j + 1)]] = 1.4
    return graph