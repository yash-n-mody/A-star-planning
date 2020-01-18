from __future__ import print_function
import Map
from Graph import OccupancyGraph

# print(Map.ToMapCoord(4.5, 9.0))
# print(Map.ToWorldCoord(8, 13))

graph = OccupancyGraph(Map.grid)
x, y = Map.ToMapCoord(-8.0, -2.0)
graph.SetStart(y, x)
x, y = Map.ToMapCoord(4.5, 9.0)
graph.SetGoal(y, x)
# Map.PrintGrid()

# graph = OccupancyGraph(Map.grid2)
# # x, y = Map.ToMapCoord(-8.0, -2.0)
# graph.SetStart(3, 0)
# # x, y = Map.ToMapCoord(4.5, 9.0)
# graph.SetGoal(18, 10)

# quick
# path = graph.FindAStarPath()
# # graph.Print()
# print("Path: ", path)
# for i in range(len(Map.grid)):
#     for j in range(len(Map.grid[i])):
#         if Map.grid[i][j] == 1:
#             print('#', end=' ')
#         elif (i, j) in path:
#             print(".", end=' ')
#         else:
#             print(' ', end=' ')
#     print(end='\n')

# details
# path = graph.FindAStarPath()
# # graph.Print()
# print("Path: ", path)
# for i in range(len(Map.grid)):
#     for j in range(len(Map.grid[i])):
#         if Map.grid[i][j] == 1:
#             print('   ***   ', end='|')
#         elif (i, j) in path:
#             print(" {0:2.1f}:{1:<2.1f}".format(graph.Graph.Nodes[(i, j)].g, graph.Graph.Nodes[(i, j)].h), end=' ')
#             # print("{0:2d}-{1:<2d}".format(i, j), end=' ')
#         else:
#             print('         ', end=' ')
#     print(end='\n\n')